// [SW1-1959 도킹 위치 나이] 영상 끊김 판정(hf_image_gate.h) — 진리표 + 경고 시점
#include <gtest/gtest.h>

#include <chrono>
#include <limits>

#include "../src/utility/hf_image_gate.h"

using namespace hf_image_gate;

namespace
{
const TimePoint kBase = kNoImageYet + std::chrono::hours(1);

TimePoint at(double s)
{
    return kBase + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
}
}  // namespace

struct FreshRow
{
    const char *why;
    double      now_s;
    double      arrival_s;
    double      timeout_s;
    bool        fresh;
};

TEST(HfImageFresh, TruthTable)
{
    const FreshRow rows[] = {
        {"방금 받음", 10.0, 10.0, 1.0, true},
        {"상한 미만", 10.9, 10.0, 1.0, true},
        {"정확히 상한 — '넘어야' 차단", 11.0, 10.0, 1.0, true},
        {"상한 초과", 11.001, 10.0, 1.0, false},
        {"오래 끊김", 3610.0, 10.0, 1.0, false},
        {"timeout 0 = 끔", 3610.0, 10.0, 0.0, true},
        {"음수 timeout = 끔", 3610.0, 10.0, -1.0, true},
        {"다른 스레드가 now 뒤에 갱신(음수 나이)", 10.0, 10.5, 1.0, true},
    };
    for (const auto &r : rows)
        EXPECT_EQ(hfImageFresh(at(r.now_s), at(r.arrival_s), r.timeout_s), r.fresh) << r.why;
}

TEST(HfImageFresh, NoImageYetNeverBlocks)
{
    EXPECT_TRUE(hfImageFresh(at(3600.0), kNoImageYet, 1.0));
    EXPECT_TRUE(hfImageFresh(at(3600.0), kNoImageYet, 0.0));
}

TEST(HfImageFresh, NanTimeoutDoesNotBlock)
{
    // 시작 로그가 "끔"으로 찍는 값(nan > 0 은 거짓)이므로 동작도 끔이어야 한다.
    EXPECT_TRUE(hfImageFresh(at(3600.0), at(0.0), std::numeric_limits<double>::quiet_NaN()));
}

TEST(CutTracker, FreshStreamIsSilent)
{
    CutTracker c;
    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(c.update(true, at(i)), CutEvent::kNone);
    EXPECT_EQ(c.cutCount(), 0);
    EXPECT_EQ(c.blockedSamples(), 0);
}

TEST(CutTracker, WarnsOnceOnCutThenEveryPeriodThenOnceOnRecovery)
{
    CutTracker c;
    EXPECT_EQ(c.update(false, at(0.0)), CutEvent::kCut);
    EXPECT_TRUE(c.isCut());
    EXPECT_EQ(c.update(false, at(0.01)), CutEvent::kNone) << "끊김 경고는 전이 때 1회만";
    EXPECT_EQ(c.update(false, at(9.99)), CutEvent::kNone);
    EXPECT_EQ(c.update(false, at(10.0)), CutEvent::kStillCut);
    EXPECT_EQ(c.update(false, at(10.01)), CutEvent::kNone) << "주기 기준이 다시 잡혀야 한다";
    EXPECT_EQ(c.update(false, at(20.0)), CutEvent::kStillCut);
    EXPECT_EQ(c.blockedSamples(), 6);

    EXPECT_EQ(c.update(true, at(21.5)), CutEvent::kRecovered);
    EXPECT_FALSE(c.isCut());
    EXPECT_DOUBLE_EQ(c.cutDurationS(at(21.5)), 21.5);
    EXPECT_EQ(c.update(true, at(21.6)), CutEvent::kNone) << "복귀 경고도 1회만";
    EXPECT_EQ(c.blockedSamples(), 6) << "신선한 표본은 세지 않는다";
    EXPECT_EQ(c.cutCount(), 1);
}

TEST(CutTracker, SecondCutCountsAndRestartsPeriod)
{
    CutTracker c;
    c.update(false, at(0.0));
    c.update(true, at(1.0));
    EXPECT_EQ(c.update(false, at(5.0)), CutEvent::kCut);
    EXPECT_EQ(c.cutCount(), 2);
    EXPECT_EQ(c.update(false, at(14.0)), CutEvent::kNone) << "주기는 이번 끊김부터 다시 잰다";
    EXPECT_EQ(c.update(false, at(15.0)), CutEvent::kStillCut);
}
