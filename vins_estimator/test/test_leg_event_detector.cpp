// [SW1-1837] LegEventDetector 단위 테스트
// 다리 이벤트 게이팅 구간 마킹의 계약을 검증한다:
//   ① 무동작 → 게이팅 없음  ② 동작 → [시작-pre, 종료+post] 마킹
//   ③ 명령 선행 트리거  ④ 연속 상한 강제 종료(발산 방지) 후 재무장
//   ⑤ 엔코더 1-LSB 플리커 면역(07-13 A/B 1차 오발동 회귀)  ⑥ 오래된 구간 정리
#include <gtest/gtest.h>

#include <cmath>

#include "../src/utility/leg_event_detector.h"

namespace
{
// 100Hz로 [t0, t1) 구간에 일정 다리각 샘플 주입
void feedStill(LegEventDetector &d, double t0, double t1, double theta)
{
    for (double t = t0; t < t1; t += 0.01)
        d.onMeasurement(t, theta, theta);
}

// 100Hz로 [t0, t1) 구간에 선형 램프(th0→th1) 주입
void feedRamp(LegEventDetector &d, double t0, double t1, double th0, double th1)
{
    for (double t = t0; t < t1; t += 0.01)
    {
        const double th = th0 + (th1 - th0) * (t - t0) / (t1 - t0);
        d.onMeasurement(t, th, th);
    }
}
}  // namespace

// ① 정지 상태만 계속 → 이벤트 0회, 어떤 구간도 게이팅되지 않음
TEST(LegEventDetector, NoMotionNoGating)
{
    LegEventDetector d;
    d.setParams({});
    feedStill(d, 0.0, 3.0, 0.017);
    EXPECT_EQ(d.activationCount(), 0);
    EXPECT_FALSE(d.overlaps(0.0, 3.0));
}

// ② 실측 동작(t=1.0~1.5, 0.3rad 스윙) → [~0.75, ~2.0] 마킹 (pre 0.3 / post 0.5)
TEST(LegEventDetector, MotionMarksIntervalWithMargins)
{
    LegEventDetector d;
    d.setParams({});
    feedStill(d, 0.0, 1.0, 0.0);
    feedRamp(d, 1.0, 1.5, 0.0, 0.3);   // 변위가 임계(0.03) 넘는 시점 ~1.05에 개시
    feedStill(d, 1.5, 3.0, 0.3);
    EXPECT_EQ(d.activationCount(), 1);
    EXPECT_TRUE(d.overlaps(1.2, 1.3));   // 동작 한가운데
    EXPECT_TRUE(d.overlaps(0.8, 0.85));  // 소급 마진 안 (개시 1.05 - 0.3 = 0.75)
    EXPECT_TRUE(d.overlaps(1.9, 1.95));  // 종료 후 유지 마진 안 (~1.5 + 0.5)
    EXPECT_FALSE(d.overlaps(0.3, 0.6));  // 마진 밖(앞)
    EXPECT_FALSE(d.overlaps(2.2, 2.5));  // 마진 밖(뒤)
}

// ③ 명령이 실측보다 선행: '목표 변경' 수신 즉시 게이트 개시(물리 반응 전 구간도 커버)
TEST(LegEventDetector, CommandOpensGateBeforeMotion)
{
    LegEventDetector d;
    d.setParams({});
    feedStill(d, 0.0, 1.0, 0.0);
    d.onCommand(0.5, 0.0, /*left=*/true);  // 첫 수신 = 기준 설정(트리거 없음)
    d.onCommand(1.0, 0.3, /*left=*/true);  // 목표 0.0→0.3 변경 → 즉시 개시
    feedStill(d, 1.0, 3.0, 0.0);           // 실측은 계속 정지(반응 지연 가정)
    EXPECT_EQ(d.activationCount(), 1);
    EXPECT_TRUE(d.overlaps(0.8, 0.9));     // 1.0 - pre(0.3) = 0.7부터 커버
    EXPECT_FALSE(d.overlaps(0.3, 0.6));
}

// ③-b 목표 변경이 없거나 임계 미만인 명령은 게이트를 열지 않음
TEST(LegEventDetector, NoopCommandIgnored)
{
    LegEventDetector d;
    d.setParams({});
    feedStill(d, 0.0, 1.0, 0.3);
    d.onCommand(0.9, 0.3, true);     // 기준 설정
    d.onCommand(1.0, 0.3, true);     // 동일 목표 반복 → 무시
    d.onCommand(0.9, 0.3, false);    // R 기준 설정
    d.onCommand(1.0, 0.305, false);  // 변경량 0.005 ≤ 임계(0.02) → 무시
    feedStill(d, 1.0, 2.0, 0.3);
    EXPECT_EQ(d.activationCount(), 0);
}

// ③-c 지속 추종 오차 + 반복 명령 스트림 면역 — 07-14 edie_gate_verify bag 실증 회귀:
//    다리가 목표(0.0)에 2°(0.0349rad) 어긋난 채 정착 + 동일 목표가 13.5Hz로 계속 옴.
//    구버전(|목표-실측| 비교)은 이를 매번 새 이벤트로 오인 → 150s 중 45% 과게이팅.
TEST(LegEventDetector, PersistentOffsetCommandStreamIgnored)
{
    LegEventDetector d;
    d.setParams({});
    double next_cmd = 0.0;
    for (double t = 0.0; t < 10.0; t += 0.01)
    {
        d.onMeasurement(t, 0.0349, 0.0349);  // 실측: 2° 스탠드오프에 정착
        if (t >= next_cmd)
        {
            d.onCommand(t, 0.0, true);        // 목표 0.0 반복 스트림
            d.onCommand(t, 0.0, false);
            next_cmd += 0.074;                // ~13.5Hz
        }
    }
    EXPECT_EQ(d.activationCount(), 0);
    EXPECT_FALSE(d.overlaps(0.0, 10.0));
}

// ④ 연속 동작이 상한(2s)을 넘으면 강제 종료 — 휠 앵커 상실 발산 방지.
//    정착 관측 전엔 재점화하지 않고, 정착 후 새 동작은 새 이벤트로 잡음.
TEST(LegEventDetector, MaxDurationForceCloseAndRearm)
{
    LegEventDetector d;
    d.setParams({});
    feedStill(d, 0.0, 0.5, 0.0);
    feedRamp(d, 0.5, 4.0, 0.0, 1.0);   // 3.5s 연속 동작(비정상 시나리오) → 개시 ~0.61
    EXPECT_TRUE(d.everForceClosed());
    EXPECT_EQ(d.activationCount(), 1);      // 강제종료 후 즉시 재점화 금지
    EXPECT_TRUE(d.overlaps(1.0, 1.1));      // 상한(0.61+2.0≈2.61)까지는 마킹
    EXPECT_FALSE(d.overlaps(3.5, 3.6));     // 상한 초과 구간은 게이팅 해제(문서화된 한계)
    feedStill(d, 4.0, 5.0, 1.0);            // 정착 → 기준점 재고정 + 재무장
    feedRamp(d, 5.0, 5.5, 1.0, 1.3);        // 새 동작
    feedStill(d, 5.5, 6.5, 1.3);
    EXPECT_EQ(d.activationCount(), 2);      // 새 이벤트로 인식
    EXPECT_TRUE(d.overlaps(5.2, 5.3));
}

// ⑤ 엔코더 1-LSB(0.017rad) 플리커 면역 — 07-13 A/B 1차에서 순간 변화율 기반이
//    이걸 1.7rad/s로 오인해 가짜 이벤트 6건을 만든 회귀 케이스.
TEST(LegEventDetector, EncoderLsbFlickerIgnored)
{
    LegEventDetector d;
    d.setParams({});
    for (double t = 0.0; t < 20.0; t += 0.01)
    {
        // 1초마다 한 샘플씩 1-LSB 튀는 계단 노이즈
        const double th = 0.017 + (std::fmod(t, 1.0) < 0.01 ? 0.017 : 0.0);
        d.onMeasurement(t, th, th);
    }
    EXPECT_EQ(d.activationCount(), 0);
    EXPECT_FALSE(d.overlaps(0.0, 20.0));
}

// ⑥ history(30s)보다 오래된 종료 구간은 정리되어 더 이상 게이팅하지 않음
TEST(LegEventDetector, OldIntervalsPruned)
{
    LegEventDetector d;
    d.setParams({});
    feedStill(d, 0.0, 1.0, 0.0);
    feedRamp(d, 1.0, 1.5, 0.0, 0.3);
    feedStill(d, 1.5, 40.0, 0.3);   // 40s까지 진행 → t=1 부근 구간은 창(30s) 밖
    EXPECT_FALSE(d.overlaps(1.0, 1.5));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
