// [SW1-1837] Bg_z 잠금 발동 판정 단위 테스트
#include <gtest/gtest.h>

#include "../src/utility/bgz_lock.h"

// 수렴 대기 시간 전에는 잠그지 않는다.
TEST(BgzLock, WaitsForDelay)
{
    EXPECT_FALSE(bgz_lock::shouldLock(5.0, 0.0, 10.0, 1e-3));
}

// 대기 경과 + 정상 크기 bias면 잠근다.
TEST(BgzLock, LocksAfterDelayWhenSmall)
{
    EXPECT_TRUE(bgz_lock::shouldLock(10.1, 5e-4, 10.0, 1e-3));
}

// 이미 부풀어버린 bias(실측 인플레 2.4e-3 수준)는 잠그지 않는다 — 안전 가드.
TEST(BgzLock, RefusesInflatedBias)
{
    EXPECT_FALSE(bgz_lock::shouldLock(20.0, 2.4e-3, 10.0, 1e-3));
}

// 음수 bias도 크기로 판정한다.
TEST(BgzLock, NegativeBiasByMagnitude)
{
    EXPECT_TRUE(bgz_lock::shouldLock(15.0, -5e-4, 10.0, 1e-3));
    EXPECT_FALSE(bgz_lock::shouldLock(15.0, -2.4e-3, 10.0, 1e-3));
}

// max<=0이면 항상 거부(기능 사실상 비활성, 안전 기본값 방향).
TEST(BgzLock, DisabledByNonPositiveMax)
{
    EXPECT_FALSE(bgz_lock::shouldLock(999.0, 0.0, 10.0, 0.0));
    EXPECT_FALSE(bgz_lock::shouldLock(999.0, 0.0, 10.0, -1.0));
}
