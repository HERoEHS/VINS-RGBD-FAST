// [SW1-1837] BodyNhcFactor 회귀 테스트 (planar-motion Level 2, AutoDiff)
//   NHC 잔차 계약: v_body = qio^-1·Qi^-1·V_world,
//     residual[0] = w_y·v_body.y (횡방향), residual[1] = w_z·v_body.z (수직).
//   핵심 계약 = "경사에서도 참": 차체가 pitch θ로 기울고 경사면 방향으로 전진하면
//   월드 vz(=|v|sinθ)는 0이 아니지만 바디 vz는 0 → NHC 잔차는 0이어야 한다.
//   (파라미터 blocks: pose7[q만 사용], speed_bias9[V만 사용])

#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "../src/factor/body_nhc_factor.h"

namespace
{
void Eval(const BodyNhcFactor &f, const double pose[7], const double sb[9], double res[2])
{
    ASSERT_TRUE(f(pose, sb, res));
}
}  // namespace

// 1. 수평 자세 + 전진(월드 vx만) → 잔차 0. (평지에선 월드 제약과 동일한 상황)
TEST(BodyNhcFactorTest, LevelForwardGivesZero)
{
    BodyNhcFactor f(Eigen::Quaterniond::Identity(), 10.0, 10.0);
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double sb[9]   = {0.7, 0, 0, 0, 0, 0, 0, 0, 0};  // V=(0.7,0,0)
    double r[2];
    Eval(f, pose, sb, r);
    EXPECT_NEAR(r[0], 0.0, 1e-12);
    EXPECT_NEAR(r[1], 0.0, 1e-12);
}

// 2. 횡속도(vy)→row0만, 수직속도(vz)→row1만 반응 (축 분리).
TEST(BodyNhcFactorTest, LateralVerticalSeparation)
{
    BodyNhcFactor f(Eigen::Quaterniond::Identity(), 10.0, 10.0);
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double r[2];

    double sb_y[9] = {0, 0.2, 0, 0, 0, 0, 0, 0, 0};  // 횡 슬립
    Eval(f, pose, sb_y, r);
    EXPECT_NEAR(r[0], 10.0 * 0.2, 1e-12);
    EXPECT_NEAR(r[1], 0.0, 1e-12);

    double sb_z[9] = {0, 0, 0.1, 0, 0, 0, 0, 0, 0};  // 수직 드리프트
    Eval(f, pose, sb_z, r);
    EXPECT_NEAR(r[0], 0.0, 1e-12);
    EXPECT_NEAR(r[1], 10.0 * 0.1, 1e-12);
}

// 3. ★핵심 계약 — 경사 주행: pitch θ 기운 차체가 경사면 방향으로 전진.
//    월드 vz = -|v|sinθ ≠ 0 (월드 vz 제약이라면 벌점을 매길 상황)이지만,
//    바디 속도는 (|v|,0,0) → NHC 잔차는 0 (경사에서도 참 = 지형 강건의 근거).
TEST(BodyNhcFactorTest, SlopeDrivingStillZero)
{
    const double th = 0.15;  // 경사 ~8.6°
    Eigen::Quaterniond Qi(Eigen::AngleAxisd(th, Eigen::Vector3d::UnitY()));
    // 경사면 방향 월드 속도 = Qi·(v,0,0) — 몸이 가리키는 앞 방향으로 |v|=0.5.
    Eigen::Vector3d Vw = Qi * Eigen::Vector3d(0.5, 0, 0);
    ASSERT_GT(std::abs(Vw.z()), 0.05);  // 월드 vz는 실제로 0이 아님(=|v|sinθ)을 먼저 확인

    BodyNhcFactor f(Eigen::Quaterniond::Identity(), 10.0, 10.0);
    double pose[7] = {0, 0, 0, Qi.x(), Qi.y(), Qi.z(), Qi.w()};
    double sb[9]   = {Vw.x(), Vw.y(), Vw.z(), 0, 0, 0, 0, 0, 0};
    double r[2];
    Eval(f, pose, sb, r);
    EXPECT_NEAR(r[0], 0.0, 1e-9);  // 바디 vy = 0
    EXPECT_NEAR(r[1], 0.0, 1e-9);  // 바디 vz = 0 ← 월드 vz 제약과 갈리는 지점
}

// 4. 휠 extrinsic 회전(qio) 적용: IMU가 바퀴 대비 90° 돌아 붙었으면 축 해석이 바뀐다.
TEST(BodyNhcFactorTest, WheelExtrinsicRotationApplied)
{
    // qio = z축 90° 회전: 바퀴 y축 = IMU -x축. IMU 프레임 속도 (0.3,0,0)은 바퀴 프레임 (0,-0.3,0).
    Eigen::Quaterniond qio(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    BodyNhcFactor f(qio, 10.0, 10.0);
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double sb[9]   = {0.3, 0, 0, 0, 0, 0, 0, 0, 0};
    double r[2];
    Eval(f, pose, sb, r);
    EXPECT_NEAR(r[0], 10.0 * -0.3, 1e-9);  // 바퀴 기준 횡속도로 재해석됨
    EXPECT_NEAR(r[1], 0.0, 1e-12);
}

// 5. 가중치 선형 스케일 + w=0이면 해당 축 비활성.
TEST(BodyNhcFactorTest, WeightScalesAndZeroDisables)
{
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double sb[9]   = {0, 0.2, 0.1, 0, 0, 0, 0, 0, 0};
    BodyNhcFactor f1(Eigen::Quaterniond::Identity(), 1.0, 1.0);
    BodyNhcFactor f2(Eigen::Quaterniond::Identity(), 2.0, 2.0);
    BodyNhcFactor f0(Eigen::Quaterniond::Identity(), 0.0, 5.0);
    double r1[2], r2[2], r0[2];
    Eval(f1, pose, sb, r1);
    Eval(f2, pose, sb, r2);
    Eval(f0, pose, sb, r0);
    EXPECT_NEAR(r2[0], 2.0 * r1[0], 1e-12);
    EXPECT_NEAR(r2[1], 2.0 * r1[1], 1e-12);
    EXPECT_NEAR(r0[0], 0.0, 1e-12);          // w_y=0 → 횡 제약 없음
    EXPECT_NEAR(r0[1], 5.0 * 0.1, 1e-12);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
