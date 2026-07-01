// [SW1-1837] VerticalVelocityFactor 회귀 테스트
//   vertical-velocity factor는 "월드 수직속도 Vz를 0으로 가중치만큼 당기는" 잔차다.
//   이 테스트는 그 계약(잔차 공식 + 자코비안 구조)을 고정해 회귀를 막는다.
//   - 잔차: residual = weight * Vz  (Vz=인덱스 2. Vx/Vy·bias는 무시)
//   - 자코비안: (0,2)만 weight, 나머지 8개는 0
//
//   배경: 지면 로봇 평지 주행 시 월드 Vz≈0. z-drift=∫Vz 억제 목적. 휠(평면만)·ZUPT(정지만)이
//        못 잡는 z를 주행 중 상시 제약. 상세는 factor 헤더 주석 참조.

#include <gtest/gtest.h>

#include "../src/factor/vertical_velocity_factor.h"

namespace
{
// SizedCostFunction<1,9> 규약대로 Evaluate를 호출하는 헬퍼.
//   speed_bias: [Vx,Vy,Vz, Bax,Bay,Baz, Bgx,Bgy,Bgz] (9차원 1블록)
void EvaluateFactor(const VerticalVelocityFactor &factor, const double speed_bias[9],
                    double residuals[1], double jacobian[9], bool want_jacobian)
{
    const double *params[1]    = {speed_bias};
    double       *jacobians[1] = {jacobian};
    const bool ok = factor.Evaluate(params, residuals, want_jacobian ? jacobians : nullptr);
    ASSERT_TRUE(ok);
}
}  // namespace

// 잔차 = weight*Vz. Vx/Vy·bias가 큰 값이어도 잔차에 새지 않아야 한다(Vz만 의미).
TEST(VerticalVelocityFactorTest, ResidualEqualsWeightTimesVzIgnoringOthers)
{
    VerticalVelocityFactor factor(20.0);
    // Vx,Vy,Bias는 일부러 큰 값 → 잔차에 안 새는지 확인. Vz만 의미 있어야 함.
    const double speed_bias[9] = {9.9, -8.8, 0.07,    // V (Vz=0.07만 유효)
                                  6.6, -5.5, 4.4,       // Ba (무시돼야)
                                  3.3, -2.2, 1.1};      // Bg (무시돼야)
    double residuals[1];
    EvaluateFactor(factor, speed_bias, residuals, nullptr, /*want_jacobian=*/false);

    EXPECT_DOUBLE_EQ(residuals[0], 20.0 * 0.07);
}

// Vz == 0 이면 잔차 0 (이미 제약 충족).
TEST(VerticalVelocityFactorTest, ZeroVzGivesZeroResidual)
{
    VerticalVelocityFactor factor(20.0);
    const double speed_bias[9] = {0.5, -0.3, 0.0, 0.01, -0.02, 0.03, 1.0, 1.0, 1.0};
    double residuals[1];
    EvaluateFactor(factor, speed_bias, residuals, nullptr, false);

    EXPECT_DOUBLE_EQ(residuals[0], 0.0);
}

// 자코비안 구조: (0,2)만 weight, 나머지 8개는 0.
TEST(VerticalVelocityFactorTest, JacobianHasOnlyVzColumn)
{
    VerticalVelocityFactor factor(30.0);
    const double speed_bias[9] = {0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    double residuals[1];
    double jacobian[9];
    EvaluateFactor(factor, speed_bias, residuals, jacobian, /*want_jacobian=*/true);

    for (int c = 0; c < 9; ++c)
    {
        const double expected = (c == 2) ? 30.0 : 0.0;  // Vz 열(인덱스 2)만
        EXPECT_DOUBLE_EQ(jacobian[c], expected) << "jacobian(0," << c << ") 불일치";
    }
}

// 가중치를 키우면 잔차가 선형 스케일된다.
TEST(VerticalVelocityFactorTest, WeightScalesResidualLinearly)
{
    const double speed_bias[9] = {0, 0, 0.04, 0.02, -0.01, 0.04, 0, 0, 0};
    double r1[1], r10[1];
    VerticalVelocityFactor factor1(1.0);
    VerticalVelocityFactor factor10(10.0);
    EvaluateFactor(factor1, speed_bias, r1, nullptr, /*want_jacobian=*/false);
    EvaluateFactor(factor10, speed_bias, r10, nullptr, /*want_jacobian=*/false);

    EXPECT_DOUBLE_EQ(r10[0], 10.0 * r1[0]);
}

// jacobians == nullptr (cost만 평가) 시에도 크래시 없이 true 반환.
TEST(VerticalVelocityFactorTest, NullJacobianIsSafe)
{
    VerticalVelocityFactor factor(20.0);
    const double speed_bias[9] = {0.1, 0.1, 0.05, 0.02, 0.02, 0.02, 0, 0, 0};
    double residuals[1];
    const double *params[1] = {speed_bias};
    EXPECT_TRUE(factor.Evaluate(params, residuals, nullptr));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
