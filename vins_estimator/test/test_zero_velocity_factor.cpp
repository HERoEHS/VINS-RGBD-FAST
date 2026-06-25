// [SW1-1837] ZeroVelocityFactor(ZUPT) 회귀 테스트
//   ZUPT factor는 "정지 구간에서 body 속도(z 포함)를 0으로 당기는" 잔차다.
//   이 테스트는 그 계약(잔차 공식 + 자코비안 구조)을 고정해 회귀를 막는다.
//   - 잔차: residual = weight * V (속도 0:3 성분만, bias 성분은 무시)
//   - 자코비안: 속도 대각 3개만 weight, 나머지 0
//
//   배경: 평지 주행 시 VINS가 수평 acc bias를 오추정 → pitch bias → z drift 누적.
//        정지 구간마다 속도를 0으로 끊어 누적을 완화한다(휠 factor가 못 잡는 z velocity 보완).
//        A/B 검증 결과·운영 의미는 doc/ZUPT.md 참조.

#include <gtest/gtest.h>

#include "../src/factor/zero_velocity_factor.h"

namespace
{
// SizedCostFunction<3,9> 규약대로 Evaluate를 호출하는 헬퍼.
//   speed_bias: [Vx,Vy,Vz, Bax,Bay,Baz, Bgx,Bgy,Bgz] (9차원 1블록)
//   residuals: 3, jacobian: 3x9 row-major(27). want_jacobian=false면 nullptr 전달.
void EvaluateFactor(const ZeroVelocityFactor &factor, const double speed_bias[9],
                    double residuals[3], double jacobian[27], bool want_jacobian)
{
    const double *params[1] = {speed_bias};
    double       *jacobians[1] = {jacobian};
    const bool ok = factor.Evaluate(params, residuals, want_jacobian ? jacobians : nullptr);
    ASSERT_TRUE(ok);
}
}  // namespace

// 잔차 = weight * 속도(0:3). bias 성분이 큰 값이어도 잔차에 영향 없어야 한다.
TEST(ZeroVelocityFactorTest, ResidualEqualsWeightTimesVelocityIgnoringBias)
{
    const double weight = 100.0;
    ZeroVelocityFactor factor(weight);

    // 속도는 작은 값, bias는 일부러 큰 값으로 → bias가 잔차에 새지 않는지 확인
    const double speed_bias[9] = {0.10, -0.20, 0.05,   // V
                                  9.9, -8.8, 7.7,       // Ba (의도적으로 큰 값)
                                  6.6, -5.5, 4.4};      // Bg (의도적으로 큰 값)
    double residuals[3];
    EvaluateFactor(factor, speed_bias, residuals, nullptr, /*want_jacobian=*/false);

    EXPECT_DOUBLE_EQ(residuals[0], weight * 0.10);
    EXPECT_DOUBLE_EQ(residuals[1], weight * -0.20);
    EXPECT_DOUBLE_EQ(residuals[2], weight * 0.05);
}

// 속도가 0이면 잔차도 0 (정지 = 제약 충족).
TEST(ZeroVelocityFactorTest, ZeroVelocityGivesZeroResidual)
{
    ZeroVelocityFactor factor(100.0);
    const double speed_bias[9] = {0.0, 0.0, 0.0, 0.01, -0.02, 0.03, 0.0, 0.0, 0.0};
    double residuals[3];
    EvaluateFactor(factor, speed_bias, residuals, nullptr, false);

    EXPECT_DOUBLE_EQ(residuals[0], 0.0);
    EXPECT_DOUBLE_EQ(residuals[1], 0.0);
    EXPECT_DOUBLE_EQ(residuals[2], 0.0);
}

// 자코비안 구조: 속도 대각(0,0)(1,1)(2,2)만 weight, 나머지 24개 원소는 0.
TEST(ZeroVelocityFactorTest, JacobianHasOnlyVelocityDiagonal)
{
    const double weight = 37.5;  // 임의 값으로 스케일도 함께 확인
    ZeroVelocityFactor factor(weight);
    const double speed_bias[9] = {0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    double residuals[3];
    double jacobian[27];
    EvaluateFactor(factor, speed_bias, residuals, jacobian, /*want_jacobian=*/true);

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 9; ++c)
        {
            const double expected = (c == r) ? weight : 0.0;  // 속도 대각만 weight
            EXPECT_DOUBLE_EQ(jacobian[r * 9 + c], expected)
                << "jacobian(" << r << "," << c << ") 불일치";
        }
    }
}

// weight를 바꾸면 잔차·자코비안이 선형 스케일된다.
TEST(ZeroVelocityFactorTest, WeightScalesResidualLinearly)
{
    const double speed_bias[9] = {0.2, -0.1, 0.4, 0, 0, 0, 0, 0, 0};
    double r1[3], r10[3];
    ZeroVelocityFactor factor1(1.0);
    ZeroVelocityFactor factor10(10.0);
    EvaluateFactor(factor1, speed_bias, r1, nullptr, /*want_jacobian=*/false);
    EvaluateFactor(factor10, speed_bias, r10, nullptr, /*want_jacobian=*/false);

    for (int i = 0; i < 3; ++i)
        EXPECT_DOUBLE_EQ(r10[i], 10.0 * r1[i]);
}

// jacobians == nullptr (cost만 평가) 시에도 크래시 없이 true 반환.
TEST(ZeroVelocityFactorTest, NullJacobianIsSafe)
{
    ZeroVelocityFactor factor(100.0);
    const double speed_bias[9] = {0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0};
    double residuals[3];
    const double *params[1] = {speed_bias};
    EXPECT_TRUE(factor.Evaluate(params, residuals, nullptr));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
