// [SW1-1866] yaw 게이지 슬라이드 가드 핵심 수학 검증 (yaw_slide_guard.h 헤더 단독)
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "../src/utility/yaw_slide_guard.h"

namespace ysg = yaw_slide_guard;

// yaw 추출 헬퍼 (Utility::R2ypr와 동일 규약: atan2 기반, degree)
static double yawDeg(const Eigen::Matrix3d &R)
{
    return std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
}

TEST(YawSlideGuard, WrappedDeltaBasic)
{
    EXPECT_NEAR(ysg::wrappedDeltaDeg(10.0, 5.0), 5.0, 1e-12);
    EXPECT_NEAR(ysg::wrappedDeltaDeg(-20.0, -5.0), -15.0, 1e-12);
}

TEST(YawSlideGuard, WrappedDeltaAcrossPi)
{
    // ±180 경계 통과: 179° → -179°는 +2°지 -358°가 아님 (경계서 359° 오검출 방지)
    EXPECT_NEAR(ysg::wrappedDeltaDeg(-179.0, 179.0), 2.0, 1e-12);
    EXPECT_NEAR(ysg::wrappedDeltaDeg(179.0, -179.0), -2.0, 1e-12);
}

TEST(YawSlideGuard, ThresholdSeparatesNormalFromSlide)
{
    const double thresh = 3.0;
    // 정상 재선형화 이동(0.0x° 실측)은 미발동
    EXPECT_FALSE(ysg::isSlide(0.05, thresh));
    EXPECT_FALSE(ysg::isSlide(-0.3, thresh));
    // 실측 슬라이드(13~16°/solve)는 양방향 발동
    EXPECT_TRUE(ysg::isSlide(-16.0, thresh));
    EXPECT_TRUE(ysg::isSlide(13.0, thresh));
}

TEST(YawSlideGuard, CounterRotationUndoesSlide)
{
    // 임의 자세를 슬라이드만큼 z회전 후 counterRotation 적용 → 원 yaw 복원
    const Eigen::Matrix3d R0 =
        (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()))
            .toRotationMatrix();
    const double slide = -15.5;  // 실측 대표값
    const Eigen::Matrix3d R_slid =
        Eigen::AngleAxisd(slide * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R0;
    const Eigen::Matrix3d R_fixed = ysg::counterRotation(slide) * R_slid;
    EXPECT_NEAR(yawDeg(R_fixed), yawDeg(R0), 1e-9);
}

TEST(YawSlideGuard, PosSlideThreshold)
{
    // 정상 재선형화(mm)는 미발동, 실측 병진 슬라이드(0.03~0.15m/solve)는 발동
    EXPECT_FALSE(ysg::isPosSlide(Eigen::Vector3d(0.002, -0.001, 0.0), 0.03));
    EXPECT_TRUE(ysg::isPosSlide(Eigen::Vector3d(0.10, 0.10, 0.0), 0.03));
    EXPECT_TRUE(ysg::isPosSlide(Eigen::Vector3d(0.0, 0.0, -0.05), 0.03));
}

TEST(YawSlideGuard, TranslateBlockUndoesPosSlide)
{
    double pose[7] = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};
    const Eigen::Vector3d slide(0.1, -0.2, 0.05);
    ysg::translatePoseBlock(pose, -slide);
    EXPECT_NEAR(pose[0], 0.9, 1e-12);
    EXPECT_NEAR(pose[1], 2.2, 1e-12);
    EXPECT_NEAR(pose[2], 2.95, 1e-12);
    EXPECT_NEAR(pose[6], 1.0, 1e-12);  // 자세 성분 불변
}

TEST(YawSlideGuard, CounterRotationPreservesRelativePose)
{
    // 처치의 전제: 강체 역회전은 창 내 상대 pose를 바꾸지 않는다 (pivot 기준 회전)
    const Eigen::Vector3d pivot(1.0, -2.0, 0.5);
    Eigen::Matrix3d Ra = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d Rb = Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d Pa(0.2, 0.1, 0.0), Pb(0.5, -0.3, 0.1);

    const Eigen::Matrix3d rel_R  = Ra.transpose() * Rb;
    const Eigen::Vector3d rel_t  = Ra.transpose() * (Pb - Pa);

    const Eigen::Matrix3d dR = ysg::counterRotation(-14.0);
    Eigen::Matrix3d Ra2 = dR * Ra, Rb2 = dR * Rb;
    Eigen::Vector3d Pa2 = pivot + dR * (Pa - pivot), Pb2 = pivot + dR * (Pb - pivot);

    EXPECT_TRUE((Ra2.transpose() * Rb2 - rel_R).norm() < 1e-12);
    EXPECT_TRUE((Ra2.transpose() * (Pb2 - Pa2) - rel_t).norm() < 1e-12);
}

// ── 누적 변위 클램프 (07-30) ──

TEST(CumClamp, InsideBandNoCorrection)
{
    // 상한 이내(치유 자유 대역)는 보정 0
    EXPECT_NEAR(ysg::cumClampCorrection({0.02, 0.01, 0.0}, 0.03).norm(), 0.0, 1e-12);
    EXPECT_NEAR(ysg::cumClampCorrection({0.03, 0.0, 0.0}, 0.03).norm(), 0.0, 1e-12);  // 경계=통과
}

TEST(CumClamp, ExcessReturnsExactlyToBoundary)
{
    // obs_v2 실측급 이탈(0.32m) → 보정 후 정확히 경계(0.03m) 위
    const Eigen::Vector3d dev(0.25, -0.20, 0.0);
    const Eigen::Vector3d corr = ysg::cumClampCorrection(dev, 0.03);
    EXPECT_NEAR((dev + corr).head<2>().norm(), 0.03, 1e-12);
    // 보정은 이탈의 역방향(방향 보존)
    EXPECT_LT(corr.head<2>().dot(dev.head<2>()), 0.0);
}

TEST(CumClamp, ZUntouched)
{
    // z는 중력·plane 관할 — xy가 초과여도 z 보정 0
    const Eigen::Vector3d corr = ysg::cumClampCorrection({0.10, 0.0, 0.5}, 0.03);
    EXPECT_NEAR(corr.z(), 0.0, 1e-12);
    EXPECT_NEAR(corr.x(), -0.07, 1e-12);
}

TEST(CumClamp, PureZDeviationIgnored)
{
    // xy 성분이 상한 이내면 z가 아무리 커도 무보정
    EXPECT_NEAR(ysg::cumClampCorrection({0.0, 0.0, 2.0}, 0.03).norm(), 0.0, 1e-12);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
