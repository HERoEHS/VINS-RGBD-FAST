// [SW1-1837] 스핀 중 accel 신뢰 강등 — 판정 로직 단위 테스트
//   대상: utility/spin_acc_deweight.h (순수 함수, 파라미터 전역과 무관)
#include <gtest/gtest.h>

#include "../src/utility/spin_acc_deweight.h"

using spin_acc_deweight::accNoiseScale;

// 임계 이하 각속도 → 배율 1 (평상 주행·정지에 완전 무영향)
TEST(SpinAccDeweight, BelowThresholdNoScale)
{
    EXPECT_DOUBLE_EQ(accNoiseScale(0.0, 0.6, 10.0), 1.0);
    EXPECT_DOUBLE_EQ(accNoiseScale(0.3, 0.6, 10.0), 1.0);
    EXPECT_DOUBLE_EQ(accNoiseScale(0.6, 0.6, 10.0), 1.0);  // 경계값 = 미발동 (초과만)
}

// 임계 초과 → factor 그대로 반환
TEST(SpinAccDeweight, AboveThresholdReturnsFactor)
{
    EXPECT_DOUBLE_EQ(accNoiseScale(0.61, 0.6, 10.0), 10.0);
    EXPECT_DOUBLE_EQ(accNoiseScale(1.66, 0.6, 10.0), 10.0);  // v10 스핀 95°/s
    EXPECT_DOUBLE_EQ(accNoiseScale(2.30, 0.6, 4.0), 4.0);    // v8 스핀 132°/s
}

// 안전 가드: 임계 <=0 → 항상 비활성 (설정 실수 방어)
TEST(SpinAccDeweight, NonPositiveThresholdDisables)
{
    EXPECT_DOUBLE_EQ(accNoiseScale(5.0, 0.0, 10.0), 1.0);
    EXPECT_DOUBLE_EQ(accNoiseScale(5.0, -1.0, 10.0), 1.0);
}

// 안전 가드: factor <=1 → 항상 비활성 (강등이 아니라 강화가 되는 오설정 방어)
TEST(SpinAccDeweight, FactorAtMostOneDisables)
{
    EXPECT_DOUBLE_EQ(accNoiseScale(5.0, 0.6, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(accNoiseScale(5.0, 0.6, 0.5), 1.0);
    EXPECT_DOUBLE_EQ(accNoiseScale(5.0, 0.6, -3.0), 1.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
