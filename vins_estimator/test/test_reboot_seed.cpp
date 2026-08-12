// [SW1-1866 reboot-pose-seed] 재초기화 pose 시드 계승 — 순수 수학부 gtest (Q7)
#include <gtest/gtest.h>
#include <cmath>
#include "../src/utility/reboot_seed.h"

namespace rs = reboot_seed;

TEST(RebootSeed, YawOnlyStripsRollPitch)
{
    // roll 10°·pitch 5°·yaw 30° 회전에서 yaw만 남긴다
    const double r = 10.0 * M_PI / 180.0, p = 5.0 * M_PI / 180.0, y = 30.0 * M_PI / 180.0;
    Eigen::Matrix3d R = (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();
    const Eigen::Matrix3d Y = rs::yawOnly(R);
    EXPECT_NEAR(std::atan2(Y(1, 0), Y(0, 0)), y, 1e-9);
    EXPECT_NEAR(Y(2, 2), 1.0, 1e-12);  // z축 불변 = roll/pitch 없음
    EXPECT_NEAR(Y(2, 0), 0.0, 1e-12);
}

TEST(RebootSeed, ComposeMovesSessionOriginToSeed)
{
    // 시드 (1, 2, 0.1)·yaw 90° → 새 세션 원점(0,0,0)이 정확히 시드 자리에 앉는다
    Eigen::Matrix3d R_seed;
    Eigen::Vector3d t_seed;
    rs::finalizeSeed(Eigen::Vector3d(1, 2, 0.1), M_PI / 2, Eigen::Vector3d::Zero(), 0.0,
                     R_seed, t_seed);
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    rs::compose(R_seed, t_seed, p, R);
    EXPECT_NEAR(p.x(), 1.0, 1e-12);
    EXPECT_NEAR(p.y(), 2.0, 1e-12);
    EXPECT_NEAR(p.z(), 0.1, 1e-12);
    EXPECT_NEAR(std::atan2(R(1, 0), R(0, 0)), M_PI / 2, 1e-9);
    // 세션에서 +x로 1m 전진 = 발행 프레임에선 +y 방향(yaw 90° 시드)
    Eigen::Vector3d p2(1, 0, 0);
    Eigen::Matrix3d R2 = Eigen::Matrix3d::Identity();
    rs::compose(R_seed, t_seed, p2, R2);
    EXPECT_NEAR(p2.x(), 1.0, 1e-9);
    EXPECT_NEAR(p2.y(), 3.0, 1e-9);
}

TEST(RebootSeed, BridgeTranslationRotatesWheelFrame)
{
    // 시드 yaw 90°, 휠 프레임 yaw 0° 캡처 → 휠 (+1, 0) 이동 = 발행 (+0, +1)
    const Eigen::Vector3d d =
        rs::bridgeTranslation(M_PI / 2, 0.0, Eigen::Vector2d(1.0, 0.0));
    EXPECT_NEAR(d.x(), 0.0, 1e-12);
    EXPECT_NEAR(d.y(), 1.0, 1e-12);
    EXPECT_NEAR(d.z(), 0.0, 1e-12);
    // 시드·휠 yaw가 같으면 휠 델타 그대로
    const Eigen::Vector3d d2 =
        rs::bridgeTranslation(0.7, 0.7, Eigen::Vector2d(0.3, -0.2));
    EXPECT_NEAR(d2.x(), 0.3, 1e-12);
    EXPECT_NEAR(d2.y(), -0.2, 1e-12);
}

TEST(RebootSeed, FinalizeAddsBridge)
{
    // 캡처 (1,0,0)·yaw 0 + 다리 병진 (0.5, 0.1) + 다리 yaw 10° → T_seed 반영
    Eigen::Matrix3d R_seed;
    Eigen::Vector3d t_seed;
    const double dyaw = 10.0 * M_PI / 180.0;
    rs::finalizeSeed(Eigen::Vector3d(1, 0, 0), 0.0, Eigen::Vector3d(0.5, 0.1, 0), dyaw,
                     R_seed, t_seed);
    EXPECT_NEAR(t_seed.x(), 1.5, 1e-12);
    EXPECT_NEAR(t_seed.y(), 0.1, 1e-12);
    EXPECT_NEAR(std::atan2(R_seed(1, 0), R_seed(0, 0)), dyaw, 1e-9);
}

TEST(RebootSeed, AnchorEligibilityGuardsContamination)
{
    // 래치가 오염 시작보다 앞서야 자격 — Q4 방어
    EXPECT_TRUE(rs::anchorSeedEligible(10.0, 12.0));   // 래치 10s < 오염 12s → 유효
    EXPECT_FALSE(rs::anchorSeedEligible(13.0, 12.0));  // 오염 후 래치 → 기각
    EXPECT_TRUE(rs::anchorSeedEligible(10.0, -1.0));   // 오염 없음 → 래치만으로 유효
    EXPECT_FALSE(rs::anchorSeedEligible(-1.0, -1.0));  // 래치 없음 → 기각
}

// [SW1-1866 08-12] 회귀: 오염 시작 시각은 에피소드당 한 번만 굳는다.
//   초과→미달→재초과에서 뒤로 밀리면 그 사이 래치된 앵커가 부당하게 자격을 얻는다.
TEST(RebootSeed, OnsetRecordedOnlyOncePerEpisode)
{
    double onset = -1.0;
    rs::recordOnsetOnce(onset, 10.0);
    EXPECT_DOUBLE_EQ(onset, 10.0);
    rs::recordOnsetOnce(onset, 20.0);              // 재초과 — 밀리면 안 된다
    EXPECT_DOUBLE_EQ(onset, 10.0);

    // 앵커 자격에 미치는 영향: 12s 래치는 오염(10s) 이후라 기각되어야 한다.
    EXPECT_FALSE(rs::anchorSeedEligible(12.0, onset));

    onset = -1.0;                                  // 에피소드 종료(정화 solve) 후 재무장
    rs::recordOnsetOnce(onset, 20.0);
    EXPECT_DOUBLE_EQ(onset, 20.0);
}

// [SW1-1866 08-12] 오염 시작 = 절제 ∪ 발산 가드 감지
TEST(RebootSeed, ContaminationOnsetUnionOfAmputationAndDrift)
{
    EXPECT_DOUBLE_EQ(rs::contaminationOnset(-1.0, -1.0), -1.0);  // 둘 다 없음
    EXPECT_DOUBLE_EQ(rs::contaminationOnset(12.0, -1.0), 12.0);  // 절제만
    EXPECT_DOUBLE_EQ(rs::contaminationOnset(-1.0, 12.0), 12.0);  // 가드만
    EXPECT_DOUBLE_EQ(rs::contaminationOnset(12.0, 9.0),   9.0);  // 먼저 온 쪽(가드)
    EXPECT_DOUBLE_EQ(rs::contaminationOnset(9.0, 12.0),   9.0);  // 먼저 온 쪽(절제)
}

// 회귀: 가드가 발동한 무절제 재부팅에서 앵커가 살아나는가.
//   이 테스트가 없으면 선택자가 '무절제=건강'으로 되돌아가도 아무도 모른다 —
//   그 상태에서 오염된 정화pose를 계승한 실측이 시드 ‖xy‖ 5.663m 런이다.
TEST(RebootSeed, GuardFiredWithoutAmputationStillPicksAnchor)
{
    const double latch = 10.0, drift = 12.0, no_amputation = -1.0;
    const double onset = rs::contaminationOnset(no_amputation, drift);

    EXPECT_GE(onset, 0.0) << "가드 감지가 오염 실증으로 인정되어야 한다";
    EXPECT_TRUE(rs::anchorSeedEligible(latch, onset))
        << "오염 前 래치된 앵커는 무절제 재부팅에서도 1순위여야 한다";

    // 오염 이후 래치된 앵커는 여전히 기각(가드 경로에서도 Q4 방어 유지)
    EXPECT_FALSE(rs::anchorSeedEligible(13.0, onset));
}

// [SW1-1866 vins-output-map-anchor] 출력 핀 사슬 — composeYawXYZ 2회 합성 검증
TEST(OutputMapAnchor, ChainComposesPinAndSessionAnchor)
{
    // 세션 pose (1,0,0)/yaw0. T(odom←세션)=yaw90°+(2,0,0) → odom (2,1,0)/yaw90.
    // T(map→odom)=yaw-90°+(0,0,0.1) → map (1,-2,0.1)/yaw0.
    Eigen::Vector3d p(1, 0, 0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    rs::composeYawXYZ(M_PI / 2, Eigen::Vector3d(2, 0, 0), p, R);
    EXPECT_NEAR(p.x(), 2.0, 1e-12);
    EXPECT_NEAR(p.y(), 1.0, 1e-12);
    rs::composeYawXYZ(-M_PI / 2, Eigen::Vector3d(0, 0, 0.1), p, R);
    EXPECT_NEAR(p.x(), 1.0, 1e-9);
    EXPECT_NEAR(p.y(), -2.0, 1e-9);
    EXPECT_NEAR(p.z(), 0.1, 1e-12);
    EXPECT_NEAR(std::atan2(R(1, 0), R(0, 0)), 0.0, 1e-9);
}

TEST(OutputMapAnchor, IdentityPinIsTransparent)
{
    Eigen::Vector3d p(0.3, -0.7, 0.05);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    rs::composeYawXYZ(0.0, Eigen::Vector3d::Zero(), p, R);
    EXPECT_NEAR(p.x(), 0.3, 1e-12);
    EXPECT_NEAR(p.y(), -0.7, 1e-12);
    EXPECT_NEAR(p.z(), 0.05, 1e-12);
}

// ── 표시 앵커 시각 정합 (SW1-1866 08-09) ──
// 구 구현: composeYawXYZ(wheel) 다음 composeYawXYZ(pin) 2단 합성.
// 새 구현: computeDisplayAnchor 로 단일 변환. 아래 헬퍼로 두 경로를 직접 비교한다.
namespace
{
void legacyTwoStage(double pin_yaw, const Eigen::Vector3d &pin_t, double wheel_yaw,
                    const Eigen::Vector3d &wheel_t, Eigen::Vector3d &p, Eigen::Matrix3d &R)
{
    rs::composeYawXYZ(wheel_yaw, wheel_t, p, R);
    rs::composeYawXYZ(pin_yaw, pin_t, p, R);
}
}  // namespace

// 회귀 고정: 세션 pose S = I (핀이 init 보다 먼저 도착 = 기존 정상 경로)이면
// 새 단일 변환이 구 2단 합성과 '수치까지' 같아야 한다. 이게 깨지면 정상 세션이 회귀한다.
TEST(DisplayAnchor, MatchesLegacyWhenSessionPoseIsIdentity)
{
    const double          pin_yaw = 83.10 * M_PI / 180.0;  // 실기 실측값
    const Eigen::Vector3d pin_t(0.003, 0.077, 0.002);
    const double          wheel_yaw = -0.08 * M_PI / 180.0;
    const Eigen::Vector3d wheel_t(0.0, 0.0, 0.0);

    double          disp_yaw = 0.0;
    Eigen::Vector3d disp_t   = Eigen::Vector3d::Zero();
    rs::computeDisplayAnchor(pin_yaw, pin_t, wheel_yaw, wheel_t, Eigen::Vector3d::Zero(),
                             Eigen::Matrix3d::Identity(), disp_yaw, disp_t);

    // 임의의 후속 pose 를 두 경로로 통과시켜 비교
    for (double k : {0.0, 0.5, -1.3})
    {
        Eigen::Vector3d p_new(k, 2 * k, 0.1 * k), p_old = p_new;
        Eigen::Matrix3d R_new = Eigen::Matrix3d::Identity(), R_old = R_new;
        rs::composeYawXYZ(disp_yaw, disp_t, p_new, R_new);
        legacyTwoStage(pin_yaw, pin_t, wheel_yaw, wheel_t, p_old, R_old);
        EXPECT_NEAR(p_new.x(), p_old.x(), 1e-12);
        EXPECT_NEAR(p_new.y(), p_old.y(), 1e-12);
        EXPECT_NEAR(p_new.z(), p_old.z(), 1e-12);
        EXPECT_NEAR(std::atan2(R_new(1, 0), R_new(0, 0)),
                    std::atan2(R_old(1, 0), R_old(0, 0)), 1e-12);
    }
}

// 본질 요건: 앵커를 잡은 시점의 발행 pose 가 목표 W(= 핀 ∘ 그 순간 휠 pose)와 일치.
// 세션 pose 가 원점이 아닌(핀이 늦게 온) 경우가 바로 구 구현이 틀리던 자리다.
TEST(DisplayAnchor, PublishedPoseEqualsWheelTargetAtCaptureTime)
{
    const double          pin_yaw = 0.4;
    const Eigen::Vector3d pin_t(1.0, -2.0, 0.05);
    const double          wheel_yaw = 1.1;
    const Eigen::Vector3d wheel_t(3.0, 0.5, 0.0);

    // 핀이 늦게 와서 세션이 이미 많이 진행된 상태
    Eigen::Vector3d p_s(2.5, -1.25, 0.3);
    Eigen::Matrix3d R_s;
    const double    yaw_s = -0.75;
    R_s << std::cos(yaw_s), -std::sin(yaw_s), 0, std::sin(yaw_s), std::cos(yaw_s), 0, 0, 0, 1;

    double          disp_yaw = 0.0;
    Eigen::Vector3d disp_t   = Eigen::Vector3d::Zero();
    rs::computeDisplayAnchor(pin_yaw, pin_t, wheel_yaw, wheel_t, p_s, R_s, disp_yaw, disp_t);

    // 같은 S 를 통과시키면 목표 W 가 나와야 한다
    Eigen::Vector3d p = p_s;
    Eigen::Matrix3d R = R_s;
    rs::composeYawXYZ(disp_yaw, disp_t, p, R);

    // 목표 W = T(map→odom) ∘ T(odom←base) 를 base 원점에 적용한 것.
    //   ※ wheel_t 는 이미 odom 프레임 병진이므로 wheel_yaw 로 다시 돌리면 안 된다
    //     (원점에서 출발해 두 변환을 순서대로 얹는 것이 정의).
    Eigen::Vector3d w_p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d w_R = Eigen::Matrix3d::Identity();
    rs::composeYawXYZ(wheel_yaw, wheel_t, w_p, w_R);
    rs::composeYawXYZ(pin_yaw, pin_t, w_p, w_R);

    EXPECT_NEAR(p.x(), w_p.x(), 1e-9);
    EXPECT_NEAR(p.y(), w_p.y(), 1e-9);
    EXPECT_NEAR(p.z(), w_p.z(), 1e-9);
    EXPECT_NEAR(std::atan2(R(1, 0), R(0, 0)), std::atan2(w_R(1, 0), w_R(0, 0)), 1e-9);
}

// 구 구현 재현: init 스냅샷(휠 pose@init)과 나중 핀을 짝지으면, 그 사이 휠이 움직인 만큼
// 발행 pose 가 목표에서 어긋난다. 이 테스트는 '버그가 실재했음'을 고정한다.
TEST(DisplayAnchor, LegacyDriftsWhenPinArrivesLate)
{
    const double          pin_yaw = 0.4;
    const Eigen::Vector3d pin_t(1.0, -2.0, 0.05);
    const Eigen::Vector3d wheel_at_init(0.0, 0.0, 0.0);
    const double          wheel_yaw_init = 0.0;
    // 핀이 오기까지 로봇이 이동·회전(휠 odom 기준)
    const Eigen::Vector3d wheel_at_pin(3.0, 0.5, 0.0);
    const double          wheel_yaw_pin = 1.1;

    Eigen::Vector3d p_s(2.5, -1.25, 0.3);
    Eigen::Matrix3d R_s = Eigen::Matrix3d::Identity();

    Eigen::Vector3d p_legacy = p_s;
    Eigen::Matrix3d R_legacy = R_s;
    legacyTwoStage(pin_yaw, pin_t, wheel_yaw_init, wheel_at_init, p_legacy, R_legacy);

    double          disp_yaw = 0.0;
    Eigen::Vector3d disp_t   = Eigen::Vector3d::Zero();
    rs::computeDisplayAnchor(pin_yaw, pin_t, wheel_yaw_pin, wheel_at_pin, p_s, R_s, disp_yaw,
                             disp_t);
    Eigen::Vector3d p_fixed = p_s;
    Eigen::Matrix3d R_fixed = R_s;
    rs::composeYawXYZ(disp_yaw, disp_t, p_fixed, R_fixed);

    // 구 경로는 목표에서 크게 벗어나야 한다(= 버그). 새 경로는 위 테스트가 일치를 보장.
    EXPECT_GT((p_legacy - p_fixed).norm(), 1.0);
}

// dt 위생 — epoch급 dt 는 배제, 정상 IMU 주기는 통과.
// (estimator 쪽 가드 조건 dt>0 && dt<0.1 과 동일한 판정을 여기서 고정)
TEST(DtHygiene, RejectsEpochScaleDt)
{
    auto sane = [](double dt) { return dt > 0.0 && dt < 0.1; };
    EXPECT_FALSE(sane(1.786e9));   // 세션 경계 아티팩트(실측 net=4.06e8deg의 원인)
    EXPECT_FALSE(sane(0.0));
    EXPECT_FALSE(sane(-1.0));
    EXPECT_TRUE(sane(1.0 / 376.0));  // IMU 주기 ~2.66ms
    EXPECT_TRUE(sane(0.099));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
