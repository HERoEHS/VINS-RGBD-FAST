// [SW1-1837] 정지 시 중력 재정렬 v2(창 전체 자세 보정) 핵심 수학 단위 검증
//   대상: utility/gravity_window_realign.h (Eigen 헤더 단독 — estimator 배선과 무관하게 검증)
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "../src/utility/gravity_window_realign.h"

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace
{
constexpr double kDeg = M_PI / 180.0;

// VINS Utility::R2ypr의 yaw 성분과 동일한 정의
double yawOf(const Matrix3d &R)
{
    return std::atan2(R(1, 0), R(0, 0));
}

// 수평축(axis.z()=0) 회전 오차 E를 만든다 — roll/pitch 오차 주입용
Matrix3d horizontalError(double angle, double axis_heading)
{
    const Vector3d axis(std::cos(axis_heading), std::sin(axis_heading), 0.0);
    return AngleAxisd(angle, axis).toRotationMatrix();
}
}  // namespace

// 자세가 이미 정확하면 ΔR = I, 오차각 0
TEST(GravityWindowRealign, AlignedGivesIdentity)
{
    const Matrix3d R_true = AngleAxisd(0.7, Vector3d::UnitZ()).toRotationMatrix();  // 임의 yaw
    const Vector3d u_meas = R_true.transpose() * Vector3d::UnitZ();
    double angle = -1.0;
    const Matrix3d dR = gravity_realign::computeDeltaR(u_meas, R_true, 3.0 * kDeg, &angle);
    EXPECT_NEAR(angle, 0.0, 1e-12);
    EXPECT_TRUE(dR.isApprox(Matrix3d::Identity(), 1e-12));
}

// 알려진 roll/pitch 오차를 주입하면 ΔR 적용 후 실측 '위'가 정확히 e_z로 복원된다
TEST(GravityWindowRealign, RestoresKnownTilt)
{
    const Matrix3d R_true = AngleAxisd(-1.2, Vector3d::UnitZ()).toRotationMatrix();
    const Vector3d u_meas = R_true.transpose() * Vector3d::UnitZ();  // IMU가 잴 '위'(바디)
    const Matrix3d E      = horizontalError(2.0 * kDeg, 0.4);        // 월드 자세 오차 주입
    const Matrix3d R_est  = E * R_true;

    double angle = 0.0;
    const Matrix3d dR = gravity_realign::computeDeltaR(u_meas, R_est, 3.0 * kDeg, &angle);
    EXPECT_NEAR(angle, 2.0 * kDeg, 1e-9);
    const Vector3d up_after = (dR * R_est) * u_meas;
    EXPECT_NEAR(up_after.dot(Vector3d::UnitZ()), 1.0, 1e-12);
}

// yaw 보존: 오차가 순수 수평축이면 ΔR은 정확히 그 역회전 → 참 자세(참 yaw 포함)를 완전 복원.
//   주의 — 'yaw(R_est) 그대로 유지'가 아니다: atan2 yaw는 수평 회전 합성만으로도
//   보정각의 2차 미소량(θ²)만큼 변한다. 보존되는 것은 '참 yaw'이고,
//   yaw 변화량 자체는 θ² 이내로 유계임을 함께 확인한다.
TEST(GravityWindowRealign, YawInvariant)
{
    const double err = 1.5 * kDeg;
    for (double yaw : {0.0, 0.9, -2.4})
    {
        const Matrix3d R_true = AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix();
        const Vector3d u_meas = R_true.transpose() * Vector3d::UnitZ();
        const Matrix3d R_est  = horizontalError(err, 1.1) * R_true;
        const Matrix3d dR = gravity_realign::computeDeltaR(u_meas, R_est, 3.0 * kDeg, nullptr);
        // 참 자세 완전 복원 = 참 yaw 보존 (가능한 최선의 결과)
        EXPECT_TRUE((dR * R_est).isApprox(R_true, 1e-10));
        EXPECT_NEAR(yawOf(dR * R_est), yaw, 1e-10);
        // atan2 yaw의 이동량은 2차 미소량 이내 (1.5° → θ²≈6.9e-4 rad)
        EXPECT_LT(std::fabs(yawOf(dR * R_est) - yawOf(R_est)), err * err);
        // 축 자체도 수평(z성분 0)
        AngleAxisd aa(dR);
        EXPECT_NEAR(aa.axis().z(), 0.0, 1e-9);
    }
}

// 상한 클램프: 10° 오차·상한 3° → 적용 회전은 정확히 3°, angle_out은 클램프 전 값 보고
TEST(GravityWindowRealign, ClampsToMaxAngle)
{
    const Matrix3d R_true = Matrix3d::Identity();
    const Vector3d u_meas = R_true.transpose() * Vector3d::UnitZ();
    const Matrix3d R_est  = horizontalError(10.0 * kDeg, -0.3) * R_true;

    double angle = 0.0;
    const Matrix3d dR = gravity_realign::computeDeltaR(u_meas, R_est, 3.0 * kDeg, &angle);
    EXPECT_NEAR(angle, 10.0 * kDeg, 1e-9);                    // 보고값 = 실제 오차
    EXPECT_NEAR(AngleAxisd(dR).angle(), 3.0 * kDeg, 1e-9);    // 적용값 = 상한
}

// pose 블록 회전: pivot 지점은 불변(텔레포트 방지), 두 pose 간 상대 기하(R_ij, t_ij)도 불변
TEST(GravityWindowRealign, PivotFixedAndRelativePoseInvariant)
{
    const Matrix3d dR = horizontalError(2.5 * kDeg, 0.8);
    const Vector3d pivot(1.0, -2.0, 0.3);

    // pose i = pivot에 위치, pose j = 임의 위치·자세
    double pose_i[7] = {pivot.x(), pivot.y(), pivot.z(), 0, 0, 0, 1};
    Quaterniond qj(AngleAxisd(0.6, Vector3d(0.2, -0.5, 0.9).normalized()));
    double pose_j[7] = {3.0, 0.5, -0.1, qj.x(), qj.y(), qj.z(), qj.w()};

    // 회전 전 상대 기하
    const Matrix3d Ri0 = Matrix3d::Identity();
    const Matrix3d Rj0 = qj.toRotationMatrix();
    const Vector3d Pi0(pose_i), Pj0(pose_j);
    const Matrix3d R_ij0 = Ri0.transpose() * Rj0;
    const Vector3d t_ij0 = Ri0.transpose() * (Pj0 - Pi0);

    gravity_realign::rotatePoseBlock(pose_i, dR, pivot);
    gravity_realign::rotatePoseBlock(pose_j, dR, pivot);

    // pivot 위치 불변
    EXPECT_NEAR((Vector3d(pose_i) - pivot).norm(), 0.0, 1e-12);
    // 상대 기하 불변 → 재투영 잔차 불변의 근거
    const Matrix3d Ri1 = Quaterniond(pose_i[6], pose_i[3], pose_i[4], pose_i[5]).toRotationMatrix();
    const Matrix3d Rj1 = Quaterniond(pose_j[6], pose_j[3], pose_j[4], pose_j[5]).toRotationMatrix();
    const Vector3d Pi1(pose_i), Pj1(pose_j);
    EXPECT_TRUE((Ri1.transpose() * Rj1).isApprox(R_ij0, 1e-10));
    EXPECT_NEAR((Ri1.transpose() * (Pj1 - Pi1) - t_ij0).norm(), 0.0, 1e-10);
}

// speed-bias 블록: 속도만 회전, Ba/Bg는 바디량이라 불변 (d_ba 생략 = 모드 2 동작)
TEST(GravityWindowRealign, SpeedBiasRotatesVelocityOnly)
{
    const Matrix3d dR = horizontalError(3.0 * kDeg, 2.0);
    double sb[9] = {0.4, -0.1, 0.02, 0.011, -0.007, 0.003, 0.001, 0.002, -0.001};
    const Vector3d v0(sb), ba0(sb + 3), bg0(sb + 6);

    gravity_realign::rotateSpeedBiasBlock(sb, dR);

    EXPECT_NEAR((Vector3d(sb) - dR * v0).norm(), 0.0, 1e-12);
    EXPECT_NEAR((Vector3d(sb + 3) - ba0).norm(), 0.0, 1e-15);
    EXPECT_NEAR((Vector3d(sb + 6) - bg0).norm(), 0.0, 1e-15);
}

// (모드 3) d_ba 지정 시 Ba만 그만큼 이동, 속도 회전·Bg 불변은 유지
TEST(GravityWindowRealign, SpeedBiasShiftsBaWhenRequested)
{
    const Matrix3d dR = horizontalError(2.0 * kDeg, -1.0);
    const Vector3d d_ba(0.03, -0.21, 0.005);
    double sb[9] = {0.4, -0.1, 0.02, 0.011, -0.007, 0.003, 0.001, 0.002, -0.001};
    const Vector3d v0(sb), ba0(sb + 3), bg0(sb + 6);

    gravity_realign::rotateSpeedBiasBlock(sb, dR, d_ba);

    EXPECT_NEAR((Vector3d(sb) - dR * v0).norm(), 0.0, 1e-12);
    EXPECT_NEAR((Vector3d(sb + 3) - (ba0 + d_ba)).norm(), 0.0, 1e-15);
    EXPECT_NEAR((Vector3d(sb + 6) - bg0).norm(), 0.0, 1e-15);
}

// (모드 3) 시나리오 통합: 자세 드리프트가 Ba로 흡수된 '틀린 균형점'에서
//   raw acc 표적 보정 + computeConsistentBa 적용 후 —
//   ① 참 자세 복원 ② 정지 IMU 잔차 = 0 ③ Ba가 참값(0)으로 복귀
TEST(GravityWindowRealign, ConsistentBaRestoresTrueEquilibrium)
{
    const double g = 9.8;
    const Matrix3d R_true = AngleAxisd(0.8, Vector3d::UnitZ()).toRotationMatrix();
    // 참 Ba = 0 규약(static init 앵커)에서 IMU가 잴 raw acc
    const Vector3d acc_mean = R_true.transpose() * (g * Vector3d::UnitZ());

    // 틀린 균형점: 자세가 E만큼 틀렸고 그 오차를 Ba가 흡수해 (acc−Ba) 잔차는 0
    const Matrix3d E      = horizontalError(2.0 * kDeg, 0.7);
    const Matrix3d R_est  = E * R_true;
    const Vector3d ba_est = acc_mean - g * (R_est.transpose() * Vector3d::UnitZ());
    // 모드 2 표적(acc−Ba_est)으로는 오차가 안 보인다 = 미발동 (실측 재현 시나리오)
    double angle2 = 0.0;
    gravity_realign::computeDeltaR(acc_mean - ba_est, R_est, 3.0 * kDeg, &angle2);
    EXPECT_LT(angle2, 1e-9);

    // 모드 3: raw acc 표적 → ΔR가 참 자세 복원, Ba는 참값 0으로 복귀
    double angle3 = 0.0;
    const Matrix3d dR    = gravity_realign::computeDeltaR(acc_mean, R_est, 3.0 * kDeg, &angle3);
    const Matrix3d R_new = dR * R_est;
    EXPECT_NEAR(angle3, 2.0 * kDeg, 1e-9);
    EXPECT_TRUE(R_new.isApprox(R_true, 1e-10));

    const Vector3d ba_new = gravity_realign::computeConsistentBa(acc_mean, R_new, g);
    EXPECT_NEAR(ba_new.norm(), 0.0, 1e-10);  // 참 Ba 복귀
    // 정지 IMU 잔차 = acc − Ba_new − g·R_newᵀ·e_z = 0 (구성상 항등이어야 함)
    const Vector3d resid = acc_mean - ba_new - g * (R_new.transpose() * Vector3d::UnitZ());
    EXPECT_NEAR(resid.norm(), 0.0, 1e-12);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
