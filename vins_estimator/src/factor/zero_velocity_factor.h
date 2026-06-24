#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// [SW1-1837] Zero-velocity Update (ZUPT) factor
//   정지 구간에서 body 속도를 0으로 제약하는 잔차.
//   목적: 평지 주행 시 VINS가 수평 acc bias를 잘못 추정 → 자세(pitch) bias → z drift가 누적되는 것을
//        정지 구간마다 끊어준다(휠 factor는 평면 속도만 제약하고 z는 못 잡으므로 ZUPT가 z까지 보완).
//   대상 블록: para_SpeedBias[i] = [Vx,Vy,Vz, Bax,Bay,Baz, Bgx,Bgy,Bgz] (9차원)
//             그중 속도(0:3)만 0으로 당긴다. residual = weight * V.
class ZeroVelocityFactor : public ceres::SizedCostFunction<3, 9>
{
  public:
    explicit ZeroVelocityFactor(double weight) : weight_(weight) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const
    {
        // 속도 성분(인덱스 0,1,2)만 0으로 제약
        residuals[0] = weight_ * parameters[0][0];
        residuals[1] = weight_ * parameters[0][1];
        residuals[2] = weight_ * parameters[0][2];

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            J(0, 0) = weight_;
            J(1, 1) = weight_;
            J(2, 2) = weight_;
        }
        return true;
    }

  private:
    double weight_;
};
