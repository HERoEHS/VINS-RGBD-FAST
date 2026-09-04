// [SW1-1883 후속] 슬롯 적분 상한 강제 키프레임 회귀 테스트
//
// 배경: 정지 중 SECOND_NEW가 반복되면 버려지는 프레임의 IMU·휠 적분이 슬롯 WINDOW_SIZE-1에 무한 누적되고,
//   게이트(preint_max_dt_s)를 넘는 순간 그 구간 factor가 빠져 창이 끊긴다(분단). 09-04 분단 실험에서 분단만
//   없애면 v16 재부팅 3/3→0/3, Bg 요동 10~30배 감소. 처방 = 누적이 상한을 넘기 전에 키프레임 강제.
#include <gtest/gtest.h>

#include "../src/estimator/estimator.h"
#include "../src/utility/preint_gate.h"

// ---- 순수 판정 ----
TEST(PreintSlotCap, PureDecisionBoundaries)
{
    using preint_gate::shouldForceKeyframe;
    EXPECT_FALSE(shouldForceKeyframe(59.9, 60.0)) << "누적이 상한 이하면 미발동";
    EXPECT_FALSE(shouldForceKeyframe(60.0, 60.0)) << "경계값(=)은 미발동(> 기준)";
    EXPECT_TRUE(shouldForceKeyframe(60.1, 60.0)) << "누적이 상한을 넘었으면 발동(다음 프레임)";
    EXPECT_TRUE(shouldForceKeyframe(167.0, 60.0)) << "크게 넘어 있어도 발동";
    EXPECT_FALSE(shouldForceKeyframe(1000.0, 0.0)) << "cap 0 = 끔";
    EXPECT_FALSE(shouldForceKeyframe(1000.0, -1.0)) << "cap 음수 = 끔";
}

namespace
{
void setup(Estimator &est, double prev_dt)
{
    const Eigen::Vector3d z = Eigen::Vector3d::Zero();
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        est.pre_integrations[i] = new IntegrationBase{z, z, z, z};
        est.Headers[i]          = 100.0 + i;
    }
    est.pre_integrations[WINDOW_SIZE - 1]->sum_dt = prev_dt;  // 판정은 슬롯 9 누적만 본다(이번 프레임 적분 전 호출)
    est.frame_count = WINDOW_SIZE;
    est.solver_flag = Estimator::NON_LINEAR;
}
}  // namespace

// ---- Estimator 바인딩 ----
TEST(PreintSlotCap, ForcesKeyframeWhenAccumulationExceedsCap)
{
    const double saved = KEYFRAME_FORCE_PREINT_DT_S;
    KEYFRAME_FORCE_PREINT_DT_S = 60.0;
    Estimator est;
    setup(est, 60.1);  // 슬롯 9가 이미 상한 초과, 이번 프레임 적분은 아직 0(processImage 호출 시점 재현)
    EXPECT_TRUE(est.applyPreintSlotCap());
    EXPECT_EQ(est.preint_cap_force_cnt_, 1);
    KEYFRAME_FORCE_PREINT_DT_S = saved;
}

TEST(PreintSlotCap, NoForceBelowCapOrWhenDisabled)
{
    const double saved = KEYFRAME_FORCE_PREINT_DT_S;
    KEYFRAME_FORCE_PREINT_DT_S = 60.0;
    Estimator est;
    setup(est, 30.0);
    EXPECT_FALSE(est.applyPreintSlotCap());
    est.pre_integrations[WINDOW_SIZE - 1]->sum_dt = 60.0;
    EXPECT_FALSE(est.applyPreintSlotCap()) << "경계값 미발동";
    KEYFRAME_FORCE_PREINT_DT_S = 0.0;  // 끔
    est.pre_integrations[WINDOW_SIZE - 1]->sum_dt = 500.0;
    EXPECT_FALSE(est.applyPreintSlotCap());
    EXPECT_EQ(est.preint_cap_force_cnt_, 0);
    KEYFRAME_FORCE_PREINT_DT_S = saved;
}

TEST(PreintSlotCap, NotAppliedInInitialOrPartialWindow)
{
    const double saved = KEYFRAME_FORCE_PREINT_DT_S;
    KEYFRAME_FORCE_PREINT_DT_S = 60.0;
    Estimator est;
    setup(est, 100.0);
    est.solver_flag = Estimator::INITIAL;
    EXPECT_FALSE(est.applyPreintSlotCap()) << "INITIAL 미적용(보수)";
    est.solver_flag = Estimator::NON_LINEAR;
    est.frame_count = WINDOW_SIZE - 1;
    EXPECT_FALSE(est.applyPreintSlotCap()) << "창 미충만 미적용";
    est.frame_count = WINDOW_SIZE;
    delete est.pre_integrations[WINDOW_SIZE - 1];
    est.pre_integrations[WINDOW_SIZE - 1] = nullptr;
    EXPECT_FALSE(est.applyPreintSlotCap()) << "nullptr 슬롯 안전";
    KEYFRAME_FORCE_PREINT_DT_S = saved;
}
