// [SW1-1837] Bg_z 잠금 상태 기반 발동 판정 단위 테스트
#include <gtest/gtest.h>

#include "../src/utility/bgz_lock.h"

namespace
{
// 검증 bag 실측에 맞춘 기준 파라미터: 최소 대기 3s(1s 스파이크 회피),
// 정지 2s + 변동폭 2e-4, 폴백 8s, 가드 1e-3
bgz_lock::Params baseParams()
{
    return bgz_lock::Params{3.0, 2.0, 2e-4, 8.0, 1e-3};
}

// t0부터 dur초 동안 10Hz로 일정 조건을 흘려 넣고 발동 여부(마지막 반환값) 반환
bool feed(bgz_lock::Tracker &tr, const bgz_lock::Params &p, double t0, double dur,
          double bgz, bool still)
{
    bool fired = false;
    for (double t = t0; t < t0 + dur; t += 0.1)
        fired = tr.update(t, bgz, still, p);
    return fired;
}
}  // namespace

// 정지 + 안정 + 정상 크기면 min_wait·still_sec 경과 후 발동한다.
TEST(BgzLock, LocksAfterStillAndStable)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_TRUE(feed(tr, p, 0.0, 5.0, 5e-4, true));
}

// 최소 대기 전에는 조건이 완벽해도 잠그지 않는다 (1s 스파이크 회피 벨트).
TEST(BgzLock, WaitsForMinWait)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_FALSE(feed(tr, p, 0.0, 2.9, 5e-4, true));
}

// 움직이는 동안(still=false)은 절대 잠그지 않는다.
TEST(BgzLock, RefusesWhileMoving)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_FALSE(feed(tr, p, 0.0, 30.0, 5e-4, false));
}

// 움직임이 정지 연속 구간을 리셋한다 — 재정지 후 still_sec을 다시 채워야 발동.
TEST(BgzLock, MotionResetsStillness)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    feed(tr, p, 0.0, 4.0, 5e-4, true);              // 정지 4s (min_wait 3s 초과)
    EXPECT_FALSE(tr.update(4.0, 5e-4, false, p));    // 순간 움직임 → 리셋
    EXPECT_FALSE(feed(tr, p, 4.1, 1.5, 5e-4, true)); // 재정지 1.5s < still_sec 2s
    EXPECT_TRUE(feed(tr, p, 5.6, 1.0, 5e-4, true));  // 재정지 누적 2.5s → 발동
}

// 요동 추정(±3e-4)은 빠른 경로(안정)로는 잠기지 않고, 정지 누적 fallback_sec 후
// 중앙값 폴백으로만 잠긴다 — v5류 오염 세션 실측(정지 중에도 ±5e-4 요동) 대응.
TEST(BgzLock, UnstableLocksViaFallbackOnly)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    auto osc = [](double t) { return (int(t * 10) % 2 == 0) ? 3e-4 : -3e-4; };
    bool fired = false;
    for (double t = 0.0; t < 7.9; t += 0.1)
        fired = tr.update(t, osc(t), true, p);
    EXPECT_FALSE(fired);  // 폴백 시간(8s) 전엔 잠기지 않음
    for (double t = 7.9; t < 9.0 && !fired; t += 0.1)
        fired = tr.update(t, osc(t), true, p);
    EXPECT_TRUE(fired);  // 8s 누적 후 중앙값 근방 프레임에서 발동
}

// 인플레 수준(중앙값 2.4e-3)에서 요동하는 추정은 폴백으로도 잠그지 않는다 —
// 가드가 중앙값에도 적용(spin-first 세션 최후 방어선 유지).
TEST(BgzLock, FallbackRejectsInflatedOscillation)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    bool fired = false;
    for (double t = 0.0; t < 30.0; t += 0.1)
    {
        double bgz = 2.4e-3 + ((int(t * 10) % 2 == 0) ? 3e-4 : -3e-4);
        fired = tr.update(t, bgz, true, p);
        ASSERT_FALSE(fired) << "t=" << t;
    }
}

// fallback_sec<=0이면 폴백 경로 자체가 꺼진다(엄격 모드).
TEST(BgzLock, FallbackDisabled)
{
    bgz_lock::Tracker tr;
    auto p         = baseParams();
    p.fallback_sec = 0.0;
    bool fired = false;
    for (double t = 0.0; t < 30.0; t += 0.1)
    {
        double bgz = (int(t * 10) % 2 == 0) ? 3e-4 : -3e-4;
        fired = tr.update(t, bgz, true, p);
        ASSERT_FALSE(fired) << "t=" << t;
    }
}

// 이미 부풀어버린 값(실측 인플레 2.4e-3 수준)은 안정적이어도 잠그지 않는다 — 크기 가드.
TEST(BgzLock, RefusesInflatedBias)
{
    bgz_lock::Tracker tr;
    auto p = baseParams();
    EXPECT_FALSE(feed(tr, p, 0.0, 30.0, 2.4e-3, true));
}

// 음수 bias도 크기로 판정한다.
TEST(BgzLock, NegativeBiasByMagnitude)
{
    bgz_lock::Tracker tr1;
    auto p = baseParams();
    EXPECT_TRUE(feed(tr1, p, 0.0, 5.0, -5e-4, true));
    bgz_lock::Tracker tr2;
    EXPECT_FALSE(feed(tr2, p, 0.0, 30.0, -2.4e-3, true));
}

// max<=0이면 항상 거부(기능 사실상 비활성, 안전 기본값 방향).
TEST(BgzLock, DisabledByNonPositiveMax)
{
    bgz_lock::Params p{3.0, 2.0, 2e-4, 8.0, 0.0};
    bgz_lock::Tracker tr;
    EXPECT_FALSE(feed(tr, p, 0.0, 30.0, 0.0, true));
    p.max_radps = -1.0;
    bgz_lock::Tracker tr2;
    EXPECT_FALSE(feed(tr2, p, 0.0, 30.0, 0.0, true));
}
