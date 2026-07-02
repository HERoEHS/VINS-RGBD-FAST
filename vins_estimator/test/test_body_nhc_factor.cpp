// [SW1-1837] BodyNhcFactor 회귀 테스트 (planar-motion Level 2, AutoDiff, 레버암 보상 v2)
//   NHC 잔차 계약: v_wheel = qio⁻¹·( Qi⁻¹·V + (ω_meas − Bg)×t_io ),
//     residual[0] = w_y·v_wheel.y (횡방향), residual[1] = w_z·v_wheel.z (수직).
//   핵심 계약 2개:
//     ① 경사에서도 참: pitch θ로 기울어 경사면 전진 → 월드 vz≠0이지만 잔차 0.
//     ② 제자리회전에서도 참(레버암): IMU가 회전축에서 t_io만큼 떨어져 실제 횡속도가 있어도
//        ω×t_io 보상으로 잔차 0. (v1은 이 케이스에서 100σ 모순 → xy +80% 악화 실증)
//   (파라미터 blocks: pose7[q만 사용], speed_bias9[V·Bg 사용])

#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "../src/factor/body_nhc_factor.h"

namespace
{
const Eigen::Quaterniond kQioI = Eigen::Quaterniond::Identity();
const Eigen::Vector3d    kZero = Eigen::Vector3d::Zero();

void Eval(const BodyNhcFactor &f, const double pose[7], const double sb[9], double res[2])
{
    ASSERT_TRUE(f(pose, sb, res));
}
}  // namespace

// 1. 수평 자세 + 전진(월드 vx만), 회전 없음 → 잔차 0. (평지에선 월드 제약과 동일한 상황)
TEST(BodyNhcFactorTest, LevelForwardGivesZero)
{
    BodyNhcFactor f(kQioI, kZero, kZero, 10.0, 10.0);
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
    BodyNhcFactor f(kQioI, kZero, kZero, 10.0, 10.0);
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

// 3. ★핵심 계약① — 경사 주행: pitch θ 기운 차체가 경사면 방향으로 전진.
//    월드 vz = -|v|sinθ ≠ 0 (월드 vz 제약이라면 벌점을 매길 상황)이지만,
//    바디 속도는 (|v|,0,0) → NHC 잔차는 0 (경사에서도 참 = 지형 강건의 근거).
TEST(BodyNhcFactorTest, SlopeDrivingStillZero)
{
    const double th = 0.15;  // 경사 ~8.6°
    Eigen::Quaterniond Qi(Eigen::AngleAxisd(th, Eigen::Vector3d::UnitY()));
    Eigen::Vector3d Vw = Qi * Eigen::Vector3d(0.5, 0, 0);  // 경사면 방향 전진
    ASSERT_GT(std::abs(Vw.z()), 0.05);  // 월드 vz는 실제로 0이 아님(=|v|sinθ)을 먼저 확인

    BodyNhcFactor f(kQioI, kZero, kZero, 10.0, 10.0);
    double pose[7] = {0, 0, 0, Qi.x(), Qi.y(), Qi.z(), Qi.w()};
    double sb[9]   = {Vw.x(), Vw.y(), Vw.z(), 0, 0, 0, 0, 0, 0};
    double r[2];
    Eval(f, pose, sb, r);
    EXPECT_NEAR(r[0], 0.0, 1e-9);  // 바디 vy = 0
    EXPECT_NEAR(r[1], 0.0, 1e-9);  // 바디 vz = 0 ← 월드 vz 제약과 갈리는 지점
}

// 4. ★핵심 계약② — 제자리회전 + 레버암: IMU가 회전축(바퀴 원점)에서 t_io 떨어져 있으면
//    IMU 속도 V = −R_i·(ω×t_io) ≠ 0 (실제 횡속도 존재). 레버암 보상으로 잔차 0이어야 한다.
//    (v1[보상 없음]이라면 r[0] = w·0.3 = 3.0의 대형 잔차 → xy 악화 실증 케이스)
TEST(BodyNhcFactorTest, SpinInPlaceLeverArmCompensated)
{
    const Eigen::Vector3d tio(0.1056, 0, -0.0941);  // 실제 EDIE body_T_wheel 값
    const Eigen::Vector3d omega(0, 0, 3.0);          // 제자리회전 3 rad/s
    // 바퀴 원점 고정 회전 → IMU 월드 속도 = -R_i·(ω×t_io). 자세 Identity 가정.
    const Eigen::Vector3d Vw = -omega.cross(tio);
    ASSERT_GT(Vw.norm(), 0.3);  // 실제 횡속도 ~0.32 m/s 존재 확인 (보상 없인 100σ 모순)

    BodyNhcFactor f(kQioI, tio, omega, 10.0, 10.0);
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double sb[9]   = {Vw.x(), Vw.y(), Vw.z(), 0, 0, 0, 0, 0, 0};
    double r[2];
    Eval(f, pose, sb, r);
    EXPECT_NEAR(r[0], 0.0, 1e-9);  // 레버암 항이 실제 횡속도를 정확히 상쇄
    EXPECT_NEAR(r[1], 0.0, 1e-9);
}

// 5. gyro bias 보정: ω_meas에 bias가 섞여 있어도 상태 Bg가 같으면 상쇄되어 잔차 0.
TEST(BodyNhcFactorTest, GyroBiasCorrected)
{
    const Eigen::Vector3d tio(0.1, 0, 0);
    const Eigen::Vector3d omega_true(0, 0, 2.0), bg(0.01, -0.02, 0.03);
    const Eigen::Vector3d Vw = -omega_true.cross(tio);

    BodyNhcFactor f(kQioI, tio, omega_true + bg, 10.0, 10.0);  // 측정 = 참 + bias
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double sb[9]   = {Vw.x(), Vw.y(), Vw.z(), 0, 0, 0, bg.x(), bg.y(), bg.z()};
    double r[2];
    Eval(f, pose, sb, r);
    EXPECT_NEAR(r[0], 0.0, 1e-9);
    EXPECT_NEAR(r[1], 0.0, 1e-9);
}

// 6. 휠 extrinsic 회전(qio) 적용: IMU가 바퀴 대비 90° 돌아 붙었으면 축 해석이 바뀐다.
TEST(BodyNhcFactorTest, WheelExtrinsicRotationApplied)
{
    Eigen::Quaterniond qio(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    BodyNhcFactor f(qio, kZero, kZero, 10.0, 10.0);
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double sb[9]   = {0.3, 0, 0, 0, 0, 0, 0, 0, 0};
    double r[2];
    Eval(f, pose, sb, r);
    EXPECT_NEAR(r[0], 10.0 * -0.3, 1e-9);  // 바퀴 기준 횡속도로 재해석됨
    EXPECT_NEAR(r[1], 0.0, 1e-12);
}

// 7. 가중치 선형 스케일 + w=0이면 해당 축 비활성.
TEST(BodyNhcFactorTest, WeightScalesAndZeroDisables)
{
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double sb[9]   = {0, 0.2, 0.1, 0, 0, 0, 0, 0, 0};
    BodyNhcFactor f1(kQioI, kZero, kZero, 1.0, 1.0);
    BodyNhcFactor f2(kQioI, kZero, kZero, 2.0, 2.0);
    BodyNhcFactor f0(kQioI, kZero, kZero, 0.0, 5.0);
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
