// [SW1-1883 후속] IMU/휠 스탬프 간극 판정(imu_gap.h) 회귀 테스트 — 헤더 단독
#include <gtest/gtest.h>

#include "../src/utility/imu_gap.h"

using imu_gap::Verdict;
using imu_gap::classify;
using imu_gap::needsRestart;

TEST(ImuGap, FirstSampleAlwaysPasses)
{
    EXPECT_EQ(classify(0.0, 100.0, 1.0), Verdict::kFirst);
    EXPECT_EQ(classify(-1.0, 100.0, 1.0), Verdict::kFirst) << "이미지 불연속 리셋 직후(prev=0)도 첫 샘플";
    EXPECT_EQ(classify(100.0, 0.0, 1.0), Verdict::kDisorder) << "stamp 0 글리치는 폐기(재시작 아님)";
    EXPECT_FALSE(needsRestart(Verdict::kFirst));
}

TEST(ImuGap, NormalAdvanceAndSmallDisorder)
{
    EXPECT_EQ(classify(100.0, 100.008, 1.0), Verdict::kOk);
    EXPECT_EQ(classify(100.0, 100.0, 1.0), Verdict::kDisorder) << "중복 스탬프는 폐기(기존 동작)";
    EXPECT_EQ(classify(100.0, 99.9, 1.0), Verdict::kDisorder) << "소폭 역행은 폐기(기존 동작)";
    EXPECT_FALSE(needsRestart(Verdict::kOk));
    EXPECT_FALSE(needsRestart(Verdict::kDisorder));
}

TEST(ImuGap, ForwardGapBoundary)
{
    EXPECT_EQ(classify(100.0, 101.0, 1.0), Verdict::kOk) << "경계값(=상한)은 통과";
    EXPECT_EQ(classify(100.0, 101.001, 1.0), Verdict::kForwardGap);
    EXPECT_EQ(classify(100.0, 137.0, 1.0), Verdict::kForwardGap) << "09-04 사례: +37 s 점프";
    EXPECT_TRUE(needsRestart(Verdict::kForwardGap));
}

TEST(ImuGap, BackwardJump)
{
    EXPECT_EQ(classify(100.0, 99.0, 1.0), Verdict::kDisorder) << "−상한 경계는 폐기";
    EXPECT_EQ(classify(100.0, 98.999, 1.0), Verdict::kBackwardJump);
    EXPECT_EQ(classify(137.0, 100.0, 1.0), Verdict::kBackwardJump) << "리더 재앵커 후방 점프";
    EXPECT_TRUE(needsRestart(Verdict::kBackwardJump));
}

TEST(ImuGap, DisabledKeepsLegacyBehavior)
{
    EXPECT_EQ(classify(100.0, 137.0, 0.0), Verdict::kOk) << "끔: 전방 점프도 통과(기존)";
    EXPECT_EQ(classify(137.0, 100.0, 0.0), Verdict::kDisorder) << "끔: 역행은 폐기(기존)";
    EXPECT_EQ(classify(100.0, 137.0, -1.0), Verdict::kOk);
}
