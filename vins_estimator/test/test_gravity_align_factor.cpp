// [SW1-1837] GravityAlignFactor 단위 테스트
// 계약: ① 자세가 실측 중력과 일치하면 잔차 0  ② 자세 오차각 ≈ |잔차|/weight (소각도)
//      ③ world-yaw 불변(중력은 yaw를 관측 못 함 — 의도된 설계)  ④ weight 선형 스케일
//      ⑤ ceres 최적화로 틀린 roll/pitch가 실측 방향으로 복원(yaw는 불변 유지)
#include <gtest/gtest.h>

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <cmath>

#include "../src/factor/gravity_align_factor.h"

namespace
{
// pose 파라미터 블록 [Px,Py,Pz, qx,qy,qz,qw] 생성
void makePose(const Eigen::Quaterniond &q, double *pose)
{
    pose[0] = pose[1] = pose[2] = 0.0;
    pose[3] = q.x(); pose[4] = q.y(); pose[5] = q.z(); pose[6] = q.w();
}

Eigen::Vector3d evalResidual(const Eigen::Quaterniond &q_wb,
                             const Eigen::Vector3d &g_meas, double weight)
{
    double pose[7];
    makePose(q_wb, pose);
    ceres::CostFunction *f = GravityAlignFactor::Create(g_meas, weight);
    double r[3];
    const double *params[1] = {pose};
    f->Evaluate(params, r, nullptr);
    delete f;
    return Eigen::Vector3d(r[0], r[1], r[2]);
}
}  // namespace

// ① 몸체가 수평이고 실측도 world-up이면 잔차 0
TEST(GravityAlignFactor, ZeroResidualWhenAligned)
{
    Eigen::Vector3d r = evalResidual(Eigen::Quaterniond::Identity(),
                                     Eigen::Vector3d(0, 0, 1), 200.0);
    EXPECT_NEAR(r.norm(), 0.0, 1e-12);
}

// ② VINS 자세가 2° 틀렸을 때(실제 몸체는 수평 = 실측 중력이 up):
//    잔차 크기 ≈ weight × 오차각[rad]
TEST(GravityAlignFactor, ResidualMatchesAttitudeError)
{
    const double err = 2.0 * M_PI / 180.0;
    Eigen::Quaterniond q_wrong(Eigen::AngleAxisd(err, Eigen::Vector3d::UnitY()));
    Eigen::Vector3d r = evalResidual(q_wrong, Eigen::Vector3d(0, 0, 1), 200.0);
    EXPECT_NEAR(r.norm() / 200.0, err, err * 0.01);  // 소각도 1% 허용
}

// ③ world-yaw를 아무리 돌려도 잔차 불변 (yaw 불관측 = 설계 계약)
TEST(GravityAlignFactor, WorldYawInvariant)
{
    const double err = 3.0 * M_PI / 180.0;
    Eigen::Quaterniond q(Eigen::AngleAxisd(err, Eigen::Vector3d::UnitX()));
    Eigen::Vector3d g(0, 0, 1);
    Eigen::Vector3d r0 = evalResidual(q, g, 200.0);
    for (double yaw : {0.5, 1.7, 3.0})
    {
        Eigen::Quaterniond qy(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Vector3d r = evalResidual(qy * q, g, 200.0);
        EXPECT_NEAR((r - r0).norm(), 0.0, 1e-12);
    }
}

// ④ weight 선형 스케일
TEST(GravityAlignFactor, WeightScalesLinearly)
{
    Eigen::Quaterniond q(Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY()));
    Eigen::Vector3d r1 = evalResidual(q, Eigen::Vector3d(0, 0, 1), 100.0);
    Eigen::Vector3d r3 = evalResidual(q, Eigen::Vector3d(0, 0, 1), 300.0);
    EXPECT_NEAR(r3.norm() / r1.norm(), 3.0, 1e-9);
}

// ⑤ 최적화 복원: roll 2°+pitch 1.5° 틀린 자세가 실측(수평)으로 수렴, yaw는 그대로
TEST(GravityAlignFactor, OptimizationRestoresRollPitchKeepsYaw)
{
    const double yaw0 = 0.8;
    Eigen::Quaterniond q_true(Eigen::AngleAxisd(yaw0, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_wrong = q_true *
        Eigen::Quaterniond(Eigen::AngleAxisd(0.035, Eigen::Vector3d::UnitX())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(0.026, Eigen::Vector3d::UnitY()));

    double pose[7];
    makePose(q_wrong, pose);
    ceres::Problem problem;
    // VINS optimization과 동일하게 pose는 quaternion 파라미터라 LocalParameterization 필요
    // (VINS의 PoseLocalParameterization 대신 등가의 Eigen quaternion 파라미터화 + 위치 고정)
    problem.AddParameterBlock(pose, 7, new ceres::ProductParameterization(
        new ceres::IdentityParameterization(3), new ceres::EigenQuaternionParameterization()));
    problem.AddResidualBlock(GravityAlignFactor::Create(Eigen::Vector3d(0, 0, 1), 200.0),
                             nullptr, pose);
    ceres::Solver::Options opt;
    ceres::Solver::Summary sum;
    ceres::Solve(opt, &problem, &sum);

    Eigen::Quaterniond q_res(pose[6], pose[3], pose[4], pose[5]);
    // roll/pitch 복원: world-up의 바디 표현이 (0,0,1)로 복귀
    Eigen::Vector3d up_body = q_res.conjugate() * Eigen::Vector3d(0, 0, 1);
    EXPECT_NEAR((up_body - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, 1e-6);
    // yaw 유지: 바디 x축의 수평 방향각이 초기 yaw와 동일
    Eigen::Vector3d xb = q_res * Eigen::Vector3d(1, 0, 0);
    EXPECT_NEAR(std::atan2(xb.y(), xb.x()), yaw0, 1e-3);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
