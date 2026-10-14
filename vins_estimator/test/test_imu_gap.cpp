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

// [SW1-1964 V1] 휠 기준 다시 잡기 판정
using imu_gap::shouldRebaseWheel;

TEST(WheelRebase, NoRequestKeepsLegacyBehavior)
{
    EXPECT_FALSE(shouldRebaseWheel(false, 137.0, 110.0, 1.0)) << "재시작 요청 없으면 큰 역행도 그대로(kDisorder 폐기)";
}

TEST(WheelRebase, ConsumesOnlyLargeBackwardSample)
{
    EXPECT_TRUE(shouldRebaseWheel(true, 137.0, 110.0, 1.0)) << "10-14 실측 −27 s 계단";
    EXPECT_FALSE(shouldRebaseWheel(true, 137.0, 136.5, 1.0)) << "소폭 역행(지터·중복)은 깃발을 먹지 않음";
    EXPECT_FALSE(shouldRebaseWheel(true, 137.0, 136.0, 1.0)) << "−문턱 경계는 먹지 않음";
    EXPECT_TRUE(shouldRebaseWheel(true, 137.0, 135.999, 1.0));
}

TEST(WheelRebase, ForwardOrLateOldSampleDoesNotConsume)
{
    // 계단 직전 옛 odom 이 늦게 와도(기준보다 앞) 깃발을 남겨 둔다 — 뒤이은 진짜 역행에서 먹도록.
    EXPECT_FALSE(shouldRebaseWheel(true, 137.0, 137.01, 1.0));
    EXPECT_FALSE(shouldRebaseWheel(true, 137.0, 164.0, 1.0)) << "앞으로 뛴 재시작 뒤 앞쪽 표본도 미소비";
}

TEST(WheelRebase, FirstSampleAndZeroStampAreNotTargets)
{
    EXPECT_FALSE(shouldRebaseWheel(true, 0.0, 100.0, 1.0)) << "기준 없음(첫 표본)은 classify 가 kFirst 로 처리";
    EXPECT_FALSE(shouldRebaseWheel(true, 137.0, 0.0, 1.0)) << "stamp 0 글리치로 기준을 지우지 않음";
}

// 아래는 노드 wheel_callback 이 실제로 쓰는 WheelStampGate 를 그대로 돌린다(배포값 wheel_gap_max_s=0).
namespace
{
struct Count
{
    int accepted = 0, dropped = 0, rebased = 0;
};
// 42 Hz 휠 표본을 n 개 흘린다. imu_t 는 같은 계단을 탄 IMU 시각(휠과 같은 시계).
Count feed(imu_gap::WheelStampGate &g, double t0, int n, bool imu_follows = true)
{
    Count c;
    for (int i = 0; i < n; ++i)
    {
        const double t     = t0 + i / 42.0;
        const double imu_t = imu_follows ? t : 0.0;
        bool         rb    = false;
        const auto   v     = g.check(t, imu_t, 0.0, 1.0, &rb);
        c.rebased += rb;
        if (v == Verdict::kDisorder)
        {
            ++c.dropped;
            continue;
        }
        g.accept(t);
        ++c.accepted;
    }
    return c;
}
}  // namespace

TEST(WheelStampGate, BackwardStepAfterRestartIsAcceptedImmediately)
{
    imu_gap::WheelStampGate g;
    feed(g, 100.0, 42 * 37);  // 100~137 s 정상
    g.requestRebase();         // IMU 가드가 −27 s 계단을 보고 재시작 요청
    const Count c = feed(g, 110.0, 42);
    EXPECT_EQ(c.rebased, 1) << "첫 역행 표본에서 한 번만 다시 잡음";
    EXPECT_EQ(c.dropped, 0) << "V1 없으면 42 개 모두 폐기";
    EXPECT_FALSE(g.rebaseRequested());
}

TEST(WheelStampGate, WithoutRequestLegacyDropsEverything)
{
    imu_gap::WheelStampGate g;
    feed(g, 100.0, 42 * 37);
    const Count c = feed(g, 110.0, 42);  // 재시작 요청 없음 = V1 전 동작
    EXPECT_EQ(c.rebased, 0);
    EXPECT_EQ(c.dropped, 42);
}

TEST(WheelStampGate, OverlappingForwardThenBackwardStep)
{
    // 10-14 실측 모양: +27 s 계단(앞) → 약 0.4 s 뒤 −27 s(뒤). 앞 계단 재시작이 깃발을 세워도 앞쪽 표본은 안 먹는다.
    imu_gap::WheelStampGate g;
    feed(g, 100.0, 42 * 10);
    g.requestRebase();                       // +27 s 로 IMU 재시작
    Count fwd = feed(g, 137.0, 17);           // 0.4 s 동안 앞쪽 표본
    EXPECT_EQ(fwd.rebased, 0);
    EXPECT_EQ(fwd.dropped, 0);
    EXPECT_TRUE(g.rebaseRequested()) << "앞쪽 표본은 깃발을 남겨 둔다";
    g.requestRebase();                       // −27 s 로 IMU 재시작
    Count back = feed(g, 110.4, 42);
    EXPECT_EQ(back.rebased, 1);
    EXPECT_EQ(back.dropped, 0);
}

TEST(WheelStampGate, WheelOnlyBackwardDoesNotConsumeStaleFlag)
{
    // critic V1 R1: 깃발이 남아 있어도 IMU 는 계단을 안 탔는데 휠만 뒤로 가면(재시작 없음) 다시 잡지 않는다.
    imu_gap::WheelStampGate g;
    feed(g, 100.0, 42 * 37);
    g.requestRebase();
    bool       rb = false;
    const auto v  = g.check(110.0, /*imu_t=*/137.0, 0.0, 1.0, &rb);
    EXPECT_FALSE(rb);
    EXPECT_EQ(v, Verdict::kDisorder) << "기존처럼 이 표본만 폐기";
    EXPECT_TRUE(g.rebaseRequested());
}

TEST(WheelStampGate, UnknownImuTimeStillRebases)
{
    // 이미지 불연속 경로가 last_imu_t 를 0 으로 둔 직후(IMU 시각 모름)에도 다시 잡을 수 있어야 한다.
    imu_gap::WheelStampGate g;
    feed(g, 100.0, 42 * 37);
    g.requestRebase();
    const Count c = feed(g, 110.0, 42, /*imu_follows=*/false);
    EXPECT_EQ(c.rebased, 1);
    EXPECT_EQ(c.dropped, 0);
}

TEST(WheelStampGate, SmallJitterKeepsFlagAndDropsOne)
{
    imu_gap::WheelStampGate g;
    feed(g, 100.0, 42);
    g.requestRebase();
    bool       rb = false;
    const auto v  = g.check(100.5, 100.5, 0.0, 1.0, &rb);  // 기준(~100.98)보다 0.5 s 과거
    EXPECT_FALSE(rb);
    EXPECT_EQ(v, Verdict::kDisorder);
    EXPECT_TRUE(g.rebaseRequested());
}
