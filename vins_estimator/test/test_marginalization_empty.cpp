// [SW1-1883] 마지널라이즈 빈 행렬 가드 회귀 테스트 (SW1-1881과 동일 결함)
//
// 결함: MARGIN_OLD에서 Pose[0]/SB[0]을 drop하는 factor가 0개면 m=0(keep도 없으면 n=0)이 되어
//   marginalize()의 SelfAdjointEigenSolver가 0×0 행렬에서 Eigen assert("empty matrix")로 abort.
//   실기 2건(09-02 정지 중 사람 통과 코어: factors 0·m=0·n=0·prior null·sum_dt[1]=167 s /
//   09-04 도킹 접점 가림 후 후진). 수정 전엔 T1·T2가 프로세스 abort(exit −6)였다 — 음성 검증은
//   marginalize()의 n==0 조기 복귀와 m>0 분기를 되돌려 실행(EXEC_PLAN 기록).
// 수정: n==0 → false 반환(호출부가 prior 비움), m==0·n>0 → Schur 생략 통과, 나머지 불변.
#include <gtest/gtest.h>

#include "../src/estimator/estimator.h"
#include "../src/factor/marginalization_factor.h"

namespace
{
// 1-변수 잔차 r = x - target (야코비안 1). marg 정보행렬을 최소 크기로 만들기 위한 도구.
class OneVarCost : public ceres::SizedCostFunction<1, 1>
{
public:
    explicit OneVarCost(double target) : target_(target) {}
    bool Evaluate(double const *const *p, double *r, double **jac) const override
    {
        r[0] = p[0][0] - target_;
        if (jac && jac[0])
            jac[0][0] = 1.0;
        return true;
    }

private:
    double target_;
};

// 2-변수 잔차 r = x - y (야코비안 [1, -1]). drop/keep 혼합(m>0·n>0) 정상 경로용.
class TwoVarCost : public ceres::SizedCostFunction<1, 1, 1>
{
public:
    bool Evaluate(double const *const *p, double *r, double **jac) const override
    {
        r[0] = p[0][0] - p[1][0];
        if (jac)
        {
            if (jac[0]) jac[0][0] = 1.0;
            if (jac[1]) jac[1][0] = -1.0;
        }
        return true;
    }
};
}  // namespace

// T1: factor 0개 = 실기 코어 상태(m=0, n=0). 수정 전 abort → 수정 후 false 복귀.
TEST(MarginalizationEmptyGuard, NoFactorsReturnsFalseInsteadOfAbort)
{
    MarginalizationInfo info;
    info.preMarginalize();
    EXPECT_FALSE(info.marginalize());
    EXPECT_EQ(info.m, 0);
    EXPECT_EQ(info.n, 0);
}

// T2: 전 블록 drop(m>0, n=0) — keep이 없어 prior를 만들 수 없다. 수정 전엔 saes2(0×0) abort.
TEST(MarginalizationEmptyGuard, AllBlocksDroppedReturnsFalse)
{
    MarginalizationInfo info;
    double x = 3.0;
    info.addResidualBlockInfo(new ResidualBlockInfo(new OneVarCost(1.0), nullptr,
                                                    std::vector<double *>{&x},
                                                    std::vector<int>{0}));
    info.preMarginalize();
    EXPECT_FALSE(info.marginalize());
    EXPECT_EQ(info.m, 1);
    EXPECT_EQ(info.n, 0);
}

// T3: drop 없음(m=0, n>0) — Schur 보완 없이 통과 재선형화. 정보 보존(야코비안·잔차 유한, 크기 n).
TEST(MarginalizationEmptyGuard, NoDropPassesThroughPreservingInfo)
{
    MarginalizationInfo info;
    double x = 3.0;
    info.addResidualBlockInfo(new ResidualBlockInfo(new OneVarCost(1.0), nullptr,
                                                    std::vector<double *>{&x},
                                                    std::vector<int>{}));
    info.preMarginalize();
    ASSERT_TRUE(info.marginalize());
    EXPECT_EQ(info.m, 0);
    EXPECT_EQ(info.n, 1);
    ASSERT_EQ(info.linearized_jacobians.rows(), 1);
    ASSERT_EQ(info.linearized_jacobians.cols(), 1);
    EXPECT_TRUE(std::isfinite(info.linearized_jacobians(0, 0)));
    EXPECT_TRUE(std::isfinite(info.linearized_residuals(0)));
    // A = JᵀJ = 1 → sqrt 정보 1, 잔차 (x-1)=2 → 재선형화 잔차 크기 2 (정보 손실 없음)
    EXPECT_NEAR(std::fabs(info.linearized_jacobians(0, 0)), 1.0, 1e-9);
    EXPECT_NEAR(std::fabs(info.linearized_residuals(0)), 2.0, 1e-9);
}

// T4: 정상 경로(m>0, n>0) 불변 — x drop, y keep. Schur 보완 후 y에 대한 prior 잔존.
TEST(MarginalizationEmptyGuard, NormalPathUnchanged)
{
    MarginalizationInfo info;
    double x = 2.0, y = 5.0;
    // x를 target 1에 묶고, x-y 결합 → x를 지우면 y에 대한 정보가 남아야 한다
    info.addResidualBlockInfo(new ResidualBlockInfo(new OneVarCost(1.0), nullptr,
                                                    std::vector<double *>{&x},
                                                    std::vector<int>{0}));
    info.addResidualBlockInfo(new ResidualBlockInfo(new TwoVarCost(), nullptr,
                                                    std::vector<double *>{&x, &y},
                                                    std::vector<int>{0}));
    info.preMarginalize();
    ASSERT_TRUE(info.marginalize());
    EXPECT_EQ(info.m, 1);
    EXPECT_EQ(info.n, 1);
    ASSERT_EQ(info.linearized_jacobians.rows(), 1);
    EXPECT_GT(std::fabs(info.linearized_jacobians(0, 0)), 0.0) << "y에 대한 정보가 남아야 한다";
    EXPECT_TRUE(std::isfinite(info.linearized_residuals(0)));
}

// ---- 창 분단 판정(가드 절제 보류 조건) — Estimator 바인딩 ----
namespace
{
void allocPreints(Estimator &est)
{
    est.frame_count = WINDOW_SIZE;
    const Eigen::Vector3d z = Eigen::Vector3d::Zero();
    for (int i = 0; i <= WINDOW_SIZE; i++)
        est.pre_integrations[i] = new IntegrationBase{z, z, z, z};
}
}  // namespace

// T5: 09-02 코어 재현 — 슬롯 1 적분 167 s, 나머지 짧음 → 분단 슬롯 1
TEST(PreintGapSlot, CoreDumpShapeDetectsSlotOne)
{
    Estimator est;
    allocPreints(est);
    est.pre_integrations[1]->sum_dt = 167.03;
    for (int i = 2; i <= WINDOW_SIZE; i++)
        est.pre_integrations[i]->sum_dt = 0.2;
    EXPECT_EQ(est.preintGapSlot(10.0), 1);
}

// T6: 전 슬롯 짧음 → 분단 없음(-1). 슬롯 0은 미사용이라 길어도 무시. nullptr 슬롯 안전.
TEST(PreintGapSlot, NoGapAndSlotZeroIgnored)
{
    Estimator est;
    allocPreints(est);
    for (int i = 0; i <= WINDOW_SIZE; i++)
        est.pre_integrations[i]->sum_dt = 0.5;
    est.pre_integrations[0]->sum_dt = 500.0;  // 슬롯 0은 검사 대상 아님
    EXPECT_EQ(est.preintGapSlot(10.0), -1);

    delete est.pre_integrations[WINDOW_SIZE];
    est.pre_integrations[WINDOW_SIZE] = nullptr;  // 미할당 슬롯 허용
    EXPECT_EQ(est.preintGapSlot(10.0), -1);
    est.pre_integrations[WINDOW_SIZE - 1]->sum_dt = 12.0;
    EXPECT_EQ(est.preintGapSlot(10.0), WINDOW_SIZE - 1);
}
