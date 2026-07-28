// [SW1-1866] 정지 상대운동 잠금 factor 검증 (still_motion_factor.h 헤더 단독)
#include <gtest/gtest.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../src/factor/still_motion_factor.h"

static void makePose(double *p, const Eigen::Vector3d &t, const Eigen::Quaterniond &q)
{
    p[0] = t.x(); p[1] = t.y(); p[2] = t.z();
    p[3] = q.x(); p[4] = q.y(); p[5] = q.z(); p[6] = q.w();
}

TEST(StillMotionFactor, ZeroResidualWhenIdenticalPose)
{
    double pi[7], pj[7], r[4];
    makePose(pi, {1.0, 2.0, 0.5}, Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ())));
    makePose(pj, {1.0, 2.0, 0.5}, Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ())));
    StillMotionFactor f(500.0, 573.0);
    ASSERT_TRUE(f(pi, pj, r));
    for (int k = 0; k < 4; k++)
        EXPECT_NEAR(r[k], 0.0, 1e-12);
}

TEST(StillMotionFactor, DetectsRelativeYaw)
{
    // j가 i 대비 +1° 돌아 배치되면 yaw 잔차 ≈ w_y × 1°(rad) — 배치 오차 실측 규모
    double pi[7], pj[7], r[4];
    const double one_deg = 1.0 * M_PI / 180.0;
    makePose(pi, {0, 0, 0}, Eigen::Quaterniond::Identity());
    makePose(pj, {0, 0, 0}, Eigen::Quaterniond(Eigen::AngleAxisd(one_deg, Eigen::Vector3d::UnitZ())));
    StillMotionFactor f(500.0, 573.0);
    ASSERT_TRUE(f(pi, pj, r));
    EXPECT_NEAR(r[3], 573.0 * one_deg, 573.0 * one_deg * 0.01);  // 소각 근사 1% 이내
    EXPECT_NEAR(r[0], 0.0, 1e-12);
}

TEST(StillMotionFactor, DetectsRelativeTranslation)
{
    double pi[7], pj[7], r[4];
    makePose(pi, {0, 0, 0}, Eigen::Quaterniond::Identity());
    makePose(pj, {0.01, -0.02, 0.0}, Eigen::Quaterniond::Identity());
    StillMotionFactor f(500.0, 573.0);
    ASSERT_TRUE(f(pi, pj, r));
    EXPECT_NEAR(r[0], 500.0 * 0.01, 1e-9);
    EXPECT_NEAR(r[1], 500.0 * -0.02, 1e-9);
    EXPECT_NEAR(r[3], 0.0, 1e-12);
}

TEST(StillMotionFactor, RollPitchNotConstrained)
{
    // roll/pitch 상대 회전은 yaw 잔차에 (1차) 안 들어감 — 중력 관할 침범 없음
    double pi[7], pj[7], r[4];
    makePose(pi, {0, 0, 0}, Eigen::Quaterniond::Identity());
    makePose(pj, {0, 0, 0}, Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX())));
    StillMotionFactor f(500.0, 573.0);
    ASSERT_TRUE(f(pi, pj, r));
    EXPECT_NEAR(r[3], 0.0, 1e-6);
}

TEST(StillMotionFactor, AutoDiffCostFunctionEvaluates)
{
    double pi[7], pj[7];
    makePose(pi, {0, 0, 0}, Eigen::Quaterniond::Identity());
    makePose(pj, {0.005, 0, 0}, Eigen::Quaterniond(Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ())));
    ceres::CostFunction *cf = StillMotionFactor::Create(500.0, 573.0);
    const double *params[2] = {pi, pj};
    double r[4];
    ASSERT_TRUE(cf->Evaluate(params, r, nullptr));
    EXPECT_GT(std::fabs(r[0]), 0.0);
    EXPECT_GT(std::fabs(r[3]), 0.0);
    delete cf;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
