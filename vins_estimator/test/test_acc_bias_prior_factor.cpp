// [SW1-1836] AccBiasPriorFactor 회귀 테스트
//   acc-bias prior factor는 "acc bias(Ba)를 target(보통 0)으로 축별 가중치만큼 당기는" 잔차다.
//   이 테스트는 그 계약(잔차 공식 + 자코비안 구조)을 고정해 회귀를 막는다.
//   - 잔차: residual_i = weight_i * (Ba_i - target_i)  (Ba=인덱스 3:6, 속도·gyro bias는 무시)
//   - 자코비안: (i, 3+i) 대각 3개만 weight_i, 나머지 0
//
//   배경: 평지 주행 시 수평 acc bias 관측성 약함 → vins 과대추정 → pitch bias → z drift.
//        부팅 시 IMU 드라이버가 bias를 ~0 보정 → target=0 에 당겨 과대추정 억제.
//        검증/가중치는 doc/ACC_BIAS_PRIOR.md 참조.

#include <gtest/gtest.h>

#include "../src/factor/acc_bias_prior_factor.h"

namespace
{
// SizedCostFunction<3,9> 규약대로 Evaluate를 호출하는 헬퍼.
//   speed_bias: [Vx,Vy,Vz, Bax,Bay,Baz, Bgx,Bgy,Bgz] (9차원 1블록)
void EvaluateFactor(const AccBiasPriorFactor &factor, const double speed_bias[9],
                    double residuals[3], double jacobian[27], bool want_jacobian)
{
    const double *params[1]    = {speed_bias};
    double       *jacobians[1] = {jacobian};
    const bool ok = factor.Evaluate(params, residuals, want_jacobian ? jacobians : nullptr);
    ASSERT_TRUE(ok);
}
}  // namespace

// 잔차 = weight*(Ba - target). 속도(0:3)·gyro bias(6:9)가 큰 값이어도 잔차에 새지 않아야 한다.
TEST(AccBiasPriorFactorTest, ResidualEqualsWeightTimesBiasMinusTargetIgnoringOthers)
{
    const Eigen::Vector3d w(50.0, 50.0, 0.0);
    const Eigen::Vector3d target(0.0, 0.0, 0.0);
    AccBiasPriorFactor factor(w, target);

    // V·Bg는 일부러 큰 값 → 잔차에 안 새는지 확인. Ba만 의미 있어야 함.
    const double speed_bias[9] = {9.9, -8.8, 7.7,    // V (무시돼야)
                                  0.02, -0.05, 0.03,  // Ba
                                  6.6, -5.5, 4.4};    // Bg (무시돼야)
    double residuals[3];
    EvaluateFactor(factor, speed_bias, residuals, nullptr, /*want_jacobian=*/false);

    EXPECT_DOUBLE_EQ(residuals[0], 50.0 * 0.02);
    EXPECT_DOUBLE_EQ(residuals[1], 50.0 * -0.05);
    EXPECT_DOUBLE_EQ(residuals[2], 0.0 * 0.03);  // w_z=0 → az는 제약 안 됨
}

// Ba == target 이면 잔차 0 (이미 원하는 값 = 제약 충족).
TEST(AccBiasPriorFactorTest, BiasAtTargetGivesZeroResidual)
{
    const Eigen::Vector3d w(50.0, 50.0, 10.0);
    const Eigen::Vector3d target(0.01, -0.02, 0.03);
    AccBiasPriorFactor factor(w, target);

    const double speed_bias[9] = {0.5, 0.5, 0.5, 0.01, -0.02, 0.03, 1.0, 1.0, 1.0};
    double residuals[3];
    EvaluateFactor(factor, speed_bias, residuals, nullptr, false);

    EXPECT_DOUBLE_EQ(residuals[0], 0.0);
    EXPECT_DOUBLE_EQ(residuals[1], 0.0);
    EXPECT_DOUBLE_EQ(residuals[2], 0.0);
}

// 자코비안 구조: (i, 3+i) 대각만 weight_i, 나머지 24개는 0.
TEST(AccBiasPriorFactorTest, JacobianHasOnlyAccBiasDiagonal)
{
    const Eigen::Vector3d w(50.0, 30.0, 5.0);  // 축별 다른 값으로 스케일·매핑 확인
    const Eigen::Vector3d target(0.0, 0.0, 0.0);
    AccBiasPriorFactor factor(w, target);

    const double speed_bias[9] = {0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    double residuals[3];
    double jacobian[27];
    EvaluateFactor(factor, speed_bias, residuals, jacobian, /*want_jacobian=*/true);

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 9; ++c)
        {
            const double expected = (c == r + 3) ? w(r) : 0.0;  // acc bias 대각(열 3,4,5)만
            EXPECT_DOUBLE_EQ(jacobian[r * 9 + c], expected)
                << "jacobian(" << r << "," << c << ") 불일치";
        }
    }
}

// 가중치를 키우면 잔차가 선형 스케일된다.
TEST(AccBiasPriorFactorTest, WeightScalesResidualLinearly)
{
    const Eigen::Vector3d target(0.0, 0.0, 0.0);
    const double speed_bias[9] = {0, 0, 0, 0.02, -0.01, 0.04, 0, 0, 0};
    double r1[3], r10[3];
    AccBiasPriorFactor factor1(Eigen::Vector3d(1.0, 1.0, 1.0), target);
    AccBiasPriorFactor factor10(Eigen::Vector3d(10.0, 10.0, 10.0), target);
    EvaluateFactor(factor1, speed_bias, r1, nullptr, /*want_jacobian=*/false);
    EvaluateFactor(factor10, speed_bias, r10, nullptr, /*want_jacobian=*/false);

    for (int i = 0; i < 3; ++i)
        EXPECT_DOUBLE_EQ(r10[i], 10.0 * r1[i]);
}

// jacobians == nullptr (cost만 평가) 시에도 크래시 없이 true 반환.
TEST(AccBiasPriorFactorTest, NullJacobianIsSafe)
{
    AccBiasPriorFactor factor(Eigen::Vector3d(50.0, 50.0, 0.0), Eigen::Vector3d::Zero());
    const double speed_bias[9] = {0.1, 0.1, 0.1, 0.02, 0.02, 0.02, 0, 0, 0};
    double residuals[3];
    const double *params[1] = {speed_bias};
    EXPECT_TRUE(factor.Evaluate(params, residuals, nullptr));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
