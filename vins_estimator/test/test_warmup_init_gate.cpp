// [SW1-1866] 워밍업 init 게이트 단위 테스트
// 계약 검증:
//   ① 정상 경로: 정지 실증(1단) → IMU 표본 축적(2단) → READY, 폴백 없음
//   ② 실증 중단 시 카운터 리셋(오염 창 폐기) — READY 지연
//   ③ 시작부터 주행(v13 무휴식 시작 재현) → 조기 폴백
//   ④ 간헐 움직임(조기 폴백 못 잡는 혼합 패턴) → 예산 폴백
//   ⑤ 폴백 후 정지 도래 → READY 전이(재정렬 기회) + fellBack() 유지
//   ⑥ reset() = 전 상태 초기화 (GUARD_RESET_PATH_CHECKLIST Q3)
//   ⑦ 비활성(still_samples<=0) = 항상 READY(기존 동작, 안전 기본)
#include <gtest/gtest.h>

#include "../src/utility/warmup_init_gate.h"

using warmup_init_gate::Gate;
using warmup_init_gate::Params;
using warmup_init_gate::Verdict;

namespace
{
// 테스트 공용 파라미터 — 실증 10샘플 + 축적 20샘플 + 조기 폴백 15샘플 + 예산 200샘플
Params testParams()
{
    Params p;
    p.still_samples  = 10;
    p.imu_samples    = 20;
    p.moving_samples = 15;
    p.budget_samples = 200;
    return p;
}

void feed(Gate &g, int n, bool wheel_still, bool leg_stable)
{
    for (int i = 0; i < n; ++i)
        g.onSample(wheel_still, leg_stable);
}
}  // namespace

// ① 정상 경로: 정지만 계속 → 실증(10) + 축적(20) = 30샘플째 READY
TEST(WarmupInitGate, StillPathReachesReady)
{
    Gate g;
    g.setParams(testParams());
    feed(g, 29, true, true);
    EXPECT_EQ(g.verdict(), Verdict::WAIT);   // 29샘플: 축적 19개 — 아직
    feed(g, 1, true, true);
    EXPECT_EQ(g.verdict(), Verdict::READY);  // 30샘플: 축적 20개 완료
    EXPECT_FALSE(g.fellBack());
}

// ② 축적 도중 다리가 움직이면 오염 창 폐기(둘 다 리셋) — 처음부터 다시 실증
TEST(WarmupInitGate, MotionDuringCollectionResetsBothCounters)
{
    Gate g;
    g.setParams(testParams());
    feed(g, 25, true, true);                  // 실증 완료 + 축적 15개 진행 중
    feed(g, 1, true, false);                  // 다리 흔들림 1샘플 → 폐기
    EXPECT_EQ(g.imuCollected(), 0);
    EXPECT_EQ(g.stillStreak(), 0);
    feed(g, 29, true, true);                  // 리셋 후 29샘플 — 축적 19개
    EXPECT_EQ(g.verdict(), Verdict::WAIT);
    feed(g, 1, true, true);
    EXPECT_EQ(g.verdict(), Verdict::READY);
}

// ③ 시작부터 주행(ⓑ v13 무휴식 시작) → 휠 주행 연속 실증 15샘플에서 조기 폴백
TEST(WarmupInitGate, DrivingFromStartEarlyFallback)
{
    Gate g;
    g.setParams(testParams());
    feed(g, 14, false, true);
    EXPECT_EQ(g.verdict(), Verdict::WAIT);
    feed(g, 1, false, true);
    EXPECT_EQ(g.verdict(), Verdict::FALLBACK);  // 15샘플 연속 주행 = 상태 실증
    EXPECT_TRUE(g.fellBack());
}

// ③-b 주행 연속성이 끊기면 조기 폴백 스트릭도 리셋(간헐 정지는 '확실한 주행' 아님)
TEST(WarmupInitGate, IntermittentMotionDoesNotEarlyFallback)
{
    Gate g;
    g.setParams(testParams());
    for (int k = 0; k < 10; ++k)
    {
        feed(g, 14, false, true);  // 조기 폴백 문턱(15) 직전까지 주행
        feed(g, 1, true, true);    // 1샘플 정지 → 주행 스트릭 리셋
    }
    EXPECT_EQ(g.verdict(), Verdict::WAIT);  // 150샘플 — 예산(200) 전, 폴백 없음
}

// ④ 혼합 패턴(조기 폴백도 READY도 미달) → 예산 소진 시 폴백 = 무한 대기 금지 보장
TEST(WarmupInitGate, MixedPatternBudgetFallback)
{
    Gate g;
    g.setParams(testParams());
    // 정지 9샘플(실증 문턱 10 미달) + 주행 9샘플(조기 폴백 문턱 15 미달) 반복
    while (g.totalSamples() < 200 && g.verdict() == Verdict::WAIT)
    {
        feed(g, 9, true, true);
        feed(g, 9, false, true);
    }
    EXPECT_EQ(g.verdict(), Verdict::FALLBACK);
    EXPECT_TRUE(g.fellBack());
}

// ⑤ 폴백 후에도 관측 지속 → 정지 도래 시 READY 전이(재정렬 기회), fellBack()은 유지
TEST(WarmupInitGate, FallbackThenStillPromotesToReady)
{
    Gate g;
    g.setParams(testParams());
    feed(g, 15, false, true);                    // 조기 폴백
    ASSERT_EQ(g.verdict(), Verdict::FALLBACK);
    feed(g, 30, true, true);                     // 정지 도래 — 실증 10 + 축적 20
    EXPECT_EQ(g.verdict(), Verdict::READY);
    EXPECT_TRUE(g.fellBack());                   // 잠정 init이었음은 계속 구분 가능
}

// ⑤-b 예산 소진 후에도 정지가 오면 READY 전이 가능(예산은 폴백 발동용이지 관측 중단이 아님)
TEST(WarmupInitGate, BudgetFallbackStillPromotesToReady)
{
    Gate g;
    g.setParams(testParams());
    while (g.verdict() == Verdict::WAIT)
    {
        feed(g, 9, true, true);
        feed(g, 9, false, true);
    }
    ASSERT_EQ(g.verdict(), Verdict::FALLBACK);
    feed(g, 30, true, true);
    EXPECT_EQ(g.verdict(), Verdict::READY);
}

// ⑥ reset() — 재초기화 경로에서 전 상태 초기화(Q3: 병리 중 리셋의 게이트 우회 차단)
TEST(WarmupInitGate, ResetClearsAllState)
{
    Gate g;
    g.setParams(testParams());
    feed(g, 15, false, true);                    // 폴백 상태 도달
    ASSERT_TRUE(g.fellBack());
    g.reset();
    EXPECT_EQ(g.verdict(), Verdict::WAIT);       // 폴백·READY 흔적 소거
    EXPECT_FALSE(g.fellBack());
    EXPECT_EQ(g.totalSamples(), 0);
    feed(g, 30, true, true);                     // 리셋 후 정상 경로 재실증 가능
    EXPECT_EQ(g.verdict(), Verdict::READY);
}

// ⑥-b READY 도달 후 reset() → 다시 WAIT(재초기화 시 게이트 재가동)
TEST(WarmupInitGate, ResetAfterReadyRearms)
{
    Gate g;
    g.setParams(testParams());
    feed(g, 30, true, true);
    ASSERT_EQ(g.verdict(), Verdict::READY);
    g.reset();
    EXPECT_EQ(g.verdict(), Verdict::WAIT);
}

// ⑦ 비활성(still_samples<=0) — 항상 READY = 기존 동작 보존(안전 기본)
TEST(WarmupInitGate, DisabledAlwaysReady)
{
    Gate g;
    Params p = testParams();
    p.still_samples = 0;
    g.setParams(p);
    EXPECT_EQ(g.verdict(), Verdict::READY);
    feed(g, 100, false, false);                  // 어떤 입력에도 게이트 개입 없음
    EXPECT_EQ(g.verdict(), Verdict::READY);
    EXPECT_FALSE(g.fellBack());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
