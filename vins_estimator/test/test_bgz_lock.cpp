// [SW1-1837] Bg_z 잠금 — 상태 기반 발동 + 정지 실측 가드 + 재잠금 단위 테스트
#include <gtest/gtest.h>

#include "../src/utility/bgz_lock.h"

namespace
{
// 검증 bag 실측에 맞춘 기준 파라미터: 최소 대기 3s(1s 스파이크 회피),
// 정지 2s + 변동폭 2e-4, 폴백 8s, 실측 거리 가드 1e-3, 재잠금 문턱 5e-4
bgz_lock::Params baseParams()
{
    return bgz_lock::Params{3.0, 2.0, 2e-4, 8.0, 1e-3, 5e-4, 10.0, 10.0};
}

// t0부터 dur초 동안 10Hz로 일정 조건을 흘려 넣고 발동 여부(마지막 반환값) 반환.
// rest 실측은 "정지 중 참 bias = rest" 인 이상적 상황을 모사.
bool feed(bgz_lock::Tracker &tr, const bgz_lock::Params &p, double t0, double dur,
          double bgz, bool still, double rest)
{
    bool fired = false;
    for (double t = t0; t < t0 + dur; t += 0.1)
        fired = tr.update(t, bgz, still, /*rest_ready=*/still, rest, p);
    return fired;
}
}  // namespace

// ───────── 발동(Tracker) ─────────

// 정지 + 안정 + 실측 일치면 min_wait·still_sec 경과 후 발동한다.
TEST(BgzLock, LocksAfterStillAndStable)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_TRUE(feed(tr, p, 0.0, 5.0, 5e-4, true, 5e-4));
}

// ★warm 세션: 참 bias가 커도(1.9e-3, 구 절대 가드였다면 거부) 실측과 일치하면 잠근다.
//   근거: 07-23 실기 — 워밍업 참 bias는 인플레와 크기로 구분 불가, 실측 거리로만 구분됨.
TEST(BgzLock, LocksWarmTrueBiasMatchingRestMeasurement)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_TRUE(feed(tr, p, 0.0, 5.0, -1.9e-3, true, -1.9e-3));
}

// 인플레 값(실측과 괴리)은 크기가 작아도 잠그지 않는다 — 실측 거리 가드.
TEST(BgzLock, RefusesEstimateFarFromRestMeasurement)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    // 추정 2.4e-3 vs 실측 0 → 괴리 2.4e-3 > 1e-3 → 거부
    EXPECT_FALSE(feed(tr, p, 0.0, 30.0, 2.4e-3, true, 0.0));
}

// 최소 대기 전에는 조건이 완벽해도 잠그지 않는다 (1s 스파이크 회피 벨트).
TEST(BgzLock, WaitsForMinWait)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_FALSE(feed(tr, p, 0.0, 2.9, 5e-4, true, 5e-4));
}

// 움직이는 동안(still=false)은 절대 잠그지 않는다.
TEST(BgzLock, RefusesWhileMoving)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_FALSE(feed(tr, p, 0.0, 30.0, 5e-4, false, 5e-4));
}

// 움직임이 정지 연속 구간을 리셋한다 — 재정지 후 still_sec을 다시 채워야 발동.
TEST(BgzLock, MotionResetsStillness)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    feed(tr, p, 0.0, 4.0, 5e-4, true, 5e-4);
    EXPECT_FALSE(tr.update(4.0, 5e-4, false, false, 0.0, p));
    EXPECT_FALSE(feed(tr, p, 4.1, 1.5, 5e-4, true, 5e-4));
    EXPECT_TRUE(feed(tr, p, 5.6, 1.0, 5e-4, true, 5e-4));
}

// 요동 추정(±3e-4)은 빠른 경로로는 잠기지 않고, 폴백(8s 누적)으로만 잠긴다.
TEST(BgzLock, UnstableLocksViaFallbackOnly)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    auto osc = [](double t) { return (int(t * 10) % 2 == 0) ? 3e-4 : -3e-4; };
    bool fired = false;
    for (double t = 0.0; t < 7.9; t += 0.1)
        fired = tr.update(t, osc(t), true, true, 0.0, p);
    EXPECT_FALSE(fired);
    for (double t = 7.9; t < 9.0 && !fired; t += 0.1)
        fired = tr.update(t, osc(t), true, true, 0.0, p);
    EXPECT_TRUE(fired);
}

// 인플레 수준(중앙값 2.4e-3, 실측 0)에서 요동하는 추정은 폴백으로도 잠그지 않는다.
TEST(BgzLock, FallbackRejectsInflatedOscillation)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    for (double t = 0.0; t < 30.0; t += 0.1)
    {
        double bgz = 2.4e-3 + ((int(t * 10) % 2 == 0) ? 3e-4 : -3e-4);
        ASSERT_FALSE(tr.update(t, bgz, true, true, 0.0, p)) << "t=" << t;
    }
}

// 실측 준비 전(rest_ready=false)에는 잠그지 않는다.
TEST(BgzLock, RefusesWithoutRestMeasurement)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    bool fired = false;
    for (double t = 0.0; t < 10.0; t += 0.1)
        fired = tr.update(t, 5e-4, true, /*rest_ready=*/false, 0.0, p);
    EXPECT_FALSE(fired);
}

// 실측이 물리 상한(0.02 rad/s)을 넘으면(센서 이상/미정지 의심) 잠그지 않는다.
TEST(BgzLock, RefusesPhysicallyImplausibleRest)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_FALSE(feed(tr, p, 0.0, 30.0, 0.03, true, 0.03));
}

// max<=0이면 항상 거부(기능 사실상 비활성, 안전 기본값 방향).
TEST(BgzLock, DisabledByNonPositiveMax)
{
    bgz_lock::Params p{3.0, 2.0, 2e-4, 8.0, 0.0, 5e-4, 10.0, 10.0};
    bgz_lock::Tracker tr;
    EXPECT_FALSE(feed(tr, p, 0.0, 30.0, 0.0, true, 0.0));
    p.max_radps = -1.0;
    bgz_lock::Tracker tr2;
    EXPECT_FALSE(feed(tr2, p, 0.0, 30.0, 0.0, true, 0.0));
}

// ───────── 정지 실측(RestBias) ─────────

// 정지 중 노이즈 낀 gyro의 중앙값을 안정적으로 추정한다.
TEST(RestBias, MedianOfNoisyStationaryGyro)
{
    bgz_lock::RestBias rb;
    double bias = -1.9e-3;
    for (double t = 0.0; t < 3.0; t += 0.05)
    {
        double noise = ((int(t * 20) % 3) - 1) * 4e-3;  // ±4e-3 요동(실측 σ 수준)
        rb.update(t, bias + noise, true, 10.0);
    }
    ASSERT_TRUE(rb.ready(2.0));
    EXPECT_NEAR(rb.median(2.0), bias, 5e-4);
}

// 움직이면 이력이 리셋되어 ready가 풀린다.
TEST(RestBias, MotionResets)
{
    bgz_lock::RestBias rb;
    for (double t = 0.0; t < 3.0; t += 0.05)
        rb.update(t, 1e-3, true, 10.0);
    ASSERT_TRUE(rb.ready(2.0));
    rb.update(3.0, 1e-3, false, 10.0);
    EXPECT_FALSE(rb.ready(2.0));
}

// 재잠금 창(10s)은 짧은 준정지(1~3s)로는 절대 ready 안 됨 — 주행 중 준정지에서의
// 오염 재잠금(회귀 실측: v7 6회/run, 값 -0.65e-3 오염)을 구조적으로 배제.
TEST(RestBias, RelockWindowRequiresLongStillness)
{
    constexpr double kWin = 10.0;  // 재잠금 창(파라미터 BGZ_RELOCK_WIN_SEC 기본값)
    bgz_lock::RestBias rb;
    for (double t = 0.0; t < 3.0; t += 0.05)
        rb.update(t, 1e-3, true, kWin);
    EXPECT_TRUE(rb.ready(2.0));            // 초기 잠금 창은 충족
    EXPECT_FALSE(rb.ready(kWin));          // 재잠금 창은 미충족
    for (double t = 3.0; t < 11.0; t += 0.05)
        rb.update(t, 1e-3, true, kWin);
    EXPECT_TRUE(rb.ready(kWin));           // 연속 정지 10s 후 충족
}

// ───────── 재잠금(shouldRelock) ─────────

// 실측이 잠금값에서 relock_delta 이상 벌어지면(온도 표류) 재잠금한다.
TEST(Relock, FiresOnThermalDrift)
{
    auto p = baseParams();
    // 잠금 -0.13e-3, 실측 -1.9e-3 (07-23 실기 시나리오) → 괴리 1.77e-3 > 5e-4
    EXPECT_TRUE(bgz_lock::shouldRelock(-1.9e-3, -0.13e-3, 100.0, 0.0, p));
}

// 괴리가 문턱 이내면 재잠금하지 않는다 (v5/v7/v8 bag 재현성 보장).
TEST(Relock, QuietWithinDelta)
{
    auto p = baseParams();
    EXPECT_FALSE(bgz_lock::shouldRelock(-0.05e-3, -0.13e-3, 100.0, 0.0, p));
}

// 쿨다운(10s) 내 재발동 금지 + relock_delta<=0이면 기능 off + 물리 상한 가드.
TEST(Relock, CooldownDisableAndPhysGuard)
{
    auto p = baseParams();
    EXPECT_FALSE(bgz_lock::shouldRelock(-1.9e-3, -0.13e-3, 5.0, 0.0, p));  // 쿨다운
    p.relock_delta = 0.0;
    EXPECT_FALSE(bgz_lock::shouldRelock(-1.9e-3, -0.13e-3, 100.0, 0.0, p));  // off
    p.relock_delta = 5e-4;
    EXPECT_FALSE(bgz_lock::shouldRelock(0.03, 0.0, 100.0, 0.0, p));  // 물리 상한
}
