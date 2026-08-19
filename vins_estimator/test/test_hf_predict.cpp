// [SW1-1872] HF IMU 예측 경로 순수 수학부 gtest — upstream 결함 2건 수정의 회귀 방지.
// 결함① = 재적분 값 첫 샘플 고정(zero-order-hold), 결함② = 이전 샘플 공유 오염.
// 상세: doc/HF_TF_PREDICT_DEFECTS.md §3·§4
#include <gtest/gtest.h>

#include <queue>
#include <utility>
#include <vector>

#include "../src/utility/hf_predict.h"

namespace hp = hf_predict;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace
{
const Vector3d kG(0, 0, 9.81);  // 중력 (world z-up)

struct State
{
    Vector3d    P = Vector3d::Zero();
    Quaterniond Q = Quaterniond::Identity();
    Vector3d    V = Vector3d::Zero();
};
}  // namespace

// 정지 + 중력만 측정: 적분해도 상태가 움직이면 안 된다 (중력 제거 정합)
TEST(MidpointStep, StationaryGravityCompensated)
{
    State          s;
    const Vector3d acc = kG;  // 정지 시 가속도계는 +g를 읽음 (바이어스 0)
    for (int i = 0; i < 100; ++i)
        hp::midpointStep(s.P, s.Q, s.V, 0.01, acc, Vector3d::Zero(), acc, Vector3d::Zero(),
                         Vector3d::Zero(), Vector3d::Zero(), kG);
    EXPECT_NEAR(s.P.norm(), 0.0, 1e-12);
    EXPECT_NEAR(s.V.norm(), 0.0, 1e-12);
    EXPECT_NEAR(s.Q.angularDistance(Quaterniond::Identity()), 0.0, 1e-12);
}

// 등가속: 1초 뒤 V=a·t, P=½a·t² (해석해와 대조)
TEST(MidpointStep, ConstantAccelerationMatchesAnalytic)
{
    State          s;
    const Vector3d a(0.5, 0, 0);
    const Vector3d meas = a + kG;
    const double   dt = 0.001;
    for (int i = 0; i < 1000; ++i)
        hp::midpointStep(s.P, s.Q, s.V, dt, meas, Vector3d::Zero(), meas, Vector3d::Zero(),
                         Vector3d::Zero(), Vector3d::Zero(), kG);
    EXPECT_NEAR(s.V.x(), 0.5, 1e-9);
    EXPECT_NEAR(s.P.x(), 0.25, 1e-6);
}

// 등각속(z): 1초 뒤 yaw=ω·t — 회전 적분 정합
TEST(MidpointStep, ConstantYawRateMatchesAnalytic)
{
    State          s;
    const Vector3d w(0, 0, 0.5);
    const double   dt = 0.001;
    for (int i = 0; i < 1000; ++i)
        hp::midpointStep(s.P, s.Q, s.V, dt, kG, w, kG, w, Vector3d::Zero(), Vector3d::Zero(),
                         kG);
    const double yaw = Eigen::AngleAxisd(s.Q).angle();
    EXPECT_NEAR(yaw, 0.5, 1e-6);
}

// 중점 의미 회귀(결함② 가치): prev≠curr이면 결과가 "curr만 쓴 적분"과 달라야 한다.
// 이전 샘플이 오염(다른 값으로 대체)되면 결과가 변한다는 사실 자체가 분리 멤버의 근거.
TEST(MidpointStep, UsesBothPrevAndCurrentSamples)
{
    State          mid, zoh;
    const Vector3d prev = kG;                        // 직전: 정지
    const Vector3d curr = kG + Vector3d(1, 0, 0);    // 현재: 가속 시작
    hp::midpointStep(mid.P, mid.Q, mid.V, 0.01, prev, Vector3d::Zero(), curr, Vector3d::Zero(),
                     Vector3d::Zero(), Vector3d::Zero(), kG);
    hp::midpointStep(zoh.P, zoh.Q, zoh.V, 0.01, curr, Vector3d::Zero(), curr, Vector3d::Zero(),
                     Vector3d::Zero(), Vector3d::Zero(), kG);
    // 중점 = 두 샘플 평균이므로 속도 증가량이 curr-only의 절반이어야 함
    EXPECT_NEAR(mid.V.x(), 0.5 * zoh.V.x(), 1e-12);
    EXPECT_GT(zoh.V.x(), 0.0);
}

// 결함① 회귀: 표본 선택 계약 — step은 "각 원소 자신의" 시각·값을 받아야 한다.
// (기존 결함은 시각만 진행하고 값은 첫 원소로 고정 → 아래 검증이 실패했을 것)
TEST(ForEachSample, PassesPerSampleValuesNotFirstSample)
{
    std::queue<std::pair<double, std::pair<Vector3d, Vector3d>>> q;
    for (int i = 0; i < 5; ++i)
        q.push({0.1 * i, {Vector3d(i, 0, 0), Vector3d(0, i, 0)}});

    std::vector<double> times;
    std::vector<double> accs, gyrs;
    hp::forEachSample(q, [&](double t, const Vector3d &a, const Vector3d &g)
                      {
                          times.push_back(t);
                          accs.push_back(a.x());
                          gyrs.push_back(g.y());
                      });
    ASSERT_EQ(times.size(), 5u);
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_NEAR(times[i], 0.1 * i, 1e-12);
        EXPECT_NEAR(accs[i], i, 1e-12) << "값이 첫 샘플로 고정되면 여기서 실패(결함① 재발)";
        EXPECT_NEAR(gyrs[i], i, 1e-12);
    }
    // 값 복사로 소모 — 원본 큐 불변
    EXPECT_EQ(q.size(), 5u);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
