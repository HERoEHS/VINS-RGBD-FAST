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
    // 래치가 오염(첫 절제)보다 앞서야 자격 — Q4 방어
    EXPECT_TRUE(rs::anchorSeedEligible(10.0, 12.0));   // 래치 10s < 절제 12s → 유효
    EXPECT_FALSE(rs::anchorSeedEligible(13.0, 12.0));  // 오염 후 래치 → 기각
    EXPECT_TRUE(rs::anchorSeedEligible(10.0, -1.0));   // 절제 없음 → 래치만으로 유효
    EXPECT_FALSE(rs::anchorSeedEligible(-1.0, -1.0));  // 래치 없음 → 기각
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
