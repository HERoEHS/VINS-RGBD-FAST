// [SW1-1837] PlaneFactor 회귀 테스트 (VIW-Fusion plane_factor 이식, AutoDiff)
//   plane factor는 바디(바퀴)를 '월드 지면 평면'에 묶는 잔차다:
//     row0-1 = (Rio^T·Ri^T·Rpw^T·e3)[x,y]  (법선 정렬 오차 = pitch,roll)
//     row2   = zpw + (Rpw·(Pi+Ri·tio))[z]  (바퀴 고도 vs 평면 높이)
//   이 테스트는 그 계약(잔차 공식)을 알려진 자세/위치로 고정해 회귀를 막는다.
//   (파라미터 blocks: pose7, wheel-extr7, plane-rot4[x,y,z,w], plane-z1)

#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "../src/factor/plane_factor.h"

namespace
{
void Eval(const PlaneFactor &f, const double pose[7], const double ex[7],
          const double pr[4], const double pz[1], double res[3])
{
    ASSERT_TRUE(f(pose, ex, pr, pz, res));
}
}  // namespace

// 1. 완전 정렬(수평·평면 위, 모두 Identity) → 잔차 0.
TEST(PlaneFactorTest, AlignedGivesZero)
{
    PlaneFactor f(10.0, 10.0, 10.0);
    double pose[7] = {0, 0, 0, 0, 0, 0, 1}, ex[7] = {0, 0, 0, 0, 0, 0, 1};
    double pr[4] = {0, 0, 0, 1}, pz[1] = {0};
    double r[3];
    Eval(f, pose, ex, pr, pz, r);
    EXPECT_NEAR(r[0], 0.0, 1e-12);
    EXPECT_NEAR(r[1], 0.0, 1e-12);
    EXPECT_NEAR(r[2], 0.0, 1e-12);
}

// 2. z 오프셋(고도 0.1) → row2 = zpw_inv*(zpw + 고도). 자세 잔차는 0. zpw=-0.1이면 상쇄.
TEST(PlaneFactorTest, ZOffset)
{
    PlaneFactor f(10.0, 10.0, 5.0);
    double pose[7] = {0, 0, 0.1, 0, 0, 0, 1}, ex[7] = {0, 0, 0, 0, 0, 0, 1};
    double pr[4] = {0, 0, 0, 1}, pz[1] = {0};
    double r[3];
    Eval(f, pose, ex, pr, pz, r);
    EXPECT_NEAR(r[0], 0.0, 1e-12);
    EXPECT_NEAR(r[1], 0.0, 1e-12);
    EXPECT_NEAR(r[2], 5.0 * 0.1, 1e-9);  // zpw_inv*(0 + 0.1)

    double pz2[1] = {-0.1};
    Eval(f, pose, ex, pr, pz2, r);
    EXPECT_NEAR(r[2], 0.0, 1e-9);        // 평면 높이 -0.1이 고도 0.1 상쇄
}

// 3. pitch(y축) 틸트 → row0만 반응, row1≈0.  roll(x축) 틸트 → row1만 반응, row0≈0.
TEST(PlaneFactorTest, PitchRollSeparation)
{
    PlaneFactor f(10.0, 10.0, 10.0);
    double ex[7] = {0, 0, 0, 0, 0, 0, 1}, pr[4] = {0, 0, 0, 1}, pz[1] = {0};

    Eigen::Quaterniond qp(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));  // pitch
    double posep[7] = {0, 0, 0, qp.x(), qp.y(), qp.z(), qp.w()};
    double rp[3];
    Eval(f, posep, ex, pr, pz, rp);
    EXPECT_GT(std::abs(rp[0]), 0.1);   // pitch → row0 크게 반응
    EXPECT_NEAR(rp[1], 0.0, 1e-9);

    Eigen::Quaterniond qr(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));  // roll
    double poser[7] = {0, 0, 0, qr.x(), qr.y(), qr.z(), qr.w()};
    double rr[3];
    Eval(f, poser, ex, pr, pz, rr);
    EXPECT_NEAR(rr[0], 0.0, 1e-9);
    EXPECT_GT(std::abs(rr[1]), 0.1);   // roll → row1 크게 반응
}

// 4. 가중치(_n_inv) 선형 스케일.
TEST(PlaneFactorTest, WeightScales)
{
    Eigen::Quaterniond qp(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));
    double posep[7] = {0, 0, 0.1, qp.x(), qp.y(), qp.z(), qp.w()};
    double ex[7] = {0, 0, 0, 0, 0, 0, 1}, pr[4] = {0, 0, 0, 1}, pz[1] = {0};
    PlaneFactor f1(1.0, 1.0, 1.0), f2(2.0, 2.0, 2.0);
    double r1[3], r2[3];
    Eval(f1, posep, ex, pr, pz, r1);
    Eval(f2, posep, ex, pr, pz, r2);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(r2[i], 2.0 * r1[i], 1e-9);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
