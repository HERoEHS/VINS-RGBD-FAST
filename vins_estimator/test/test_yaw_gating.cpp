// [SW1-1837] 고속 회전 비전 게이팅 판정 로직 단위 검증
//   대상: utility/yaw_gating.h (Eigen 헤더 단독)
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>

#include "../src/utility/yaw_gating.h"

using Eigen::Vector3d;

namespace
{
std::vector<Vector3d> constSamples(const Vector3d &w, int n = 10)
{
    return std::vector<Vector3d>(n, w);
}
}  // namespace

// 평균 각속도 = bias 뺀 크기. 상수 샘플이면 그 값.
TEST(YawGating, FrameAngularSpeedSubtractsBias)
{
    const Vector3d bg(0.01, -0.02, 0.03);
    const Vector3d w(0.01, -0.02, 0.53);  // z에 0.5 rad/s + bias
    EXPECT_NEAR(yaw_gating::frameAngularSpeed(constSamples(w), bg), 0.5, 1e-12);
}

// 빈 버퍼 → 0 (게이트 안 걸림)
TEST(YawGating, EmptyBufferGivesZeroAndNoGate)
{
    const Vector3d bg = Vector3d::Zero();
    EXPECT_EQ(yaw_gating::frameAngularSpeed({}, bg), 0.0);
    EXPECT_FALSE(yaw_gating::isFastRotation({}, bg, 0.6));
}

// 임계 초과/미달 판정
TEST(YawGating, ThresholdBoundary)
{
    const Vector3d bg = Vector3d::Zero();
    const auto slow = constSamples(Vector3d(0, 0, 0.4));  // 0.4 rad/s
    const auto fast = constSamples(Vector3d(0, 0, 0.8));  // 0.8 rad/s
    EXPECT_FALSE(yaw_gating::isFastRotation(slow, bg, 0.6));
    EXPECT_TRUE(yaw_gating::isFastRotation(fast, bg, 0.6));
}

// 임계 <= 0 이면 항상 미게이트(안전 — 사실상 비활성)
TEST(YawGating, NonPositiveThresholdNeverGates)
{
    const Vector3d bg = Vector3d::Zero();
    const auto fast = constSamples(Vector3d(0, 0, 5.0));  // 매우 빠름
    EXPECT_FALSE(yaw_gating::isFastRotation(fast, bg, 0.0));
    EXPECT_FALSE(yaw_gating::isFastRotation(fast, bg, -1.0));
}

// 평균이 임계를 정한다 — 순간 스파이크가 있어도 평균이 낮으면 미게이트
TEST(YawGating, UsesMeanNotPeak)
{
    const Vector3d bg = Vector3d::Zero();
    std::vector<Vector3d> mixed(9, Vector3d(0, 0, 0.1));
    mixed.push_back(Vector3d(0, 0, 5.0));  // 한 샘플만 큼 → 평균 ~0.59
    EXPECT_FALSE(yaw_gating::isFastRotation(mixed, bg, 0.6));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
