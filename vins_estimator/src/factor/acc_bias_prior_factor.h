#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// [SW1-1836] Accelerometer-bias prior factor
//   문제: 평지 주행 시 수평 acc bias(ax/ay)의 관측성이 약해 VINS가 이를 과대추정한다
//        (실측: 진짜 ay≈-0.056인데 vins -0.21로 ~3.75배). 그 오차가 body pitch로 새고,
//        전진 시 중력이 z로 누설되어 z drift가 누적된다.
//   해법: IMU 드라이버(imu-ros2-wrapper)가 부팅 시 정지 상태에서 acc bias를 ~0으로 보정하므로,
//        세션 시작 시 참 bias≈0이다. 따라서 acc bias를 target(기본 0)으로 약하게 당기는 prior를
//        줘서 과대추정만 억제한다. (acc_w를 줄이는 config-only 흉내와 달리, 변화율이 아니라
//        절대값을 올바른 값에 앵커하므로 수평적분 손상이 적다.)
//   축별 분리: az(수직)는 이미 정확히 추정되므로(실측 0.031 vs 진짜 0.032) 가중치를 작게/0으로 두고,
//             과대추정되는 수평(ax/ay)만 강하게 당긴다. → weight를 Vector3d로 받는다.
//   대상 블록: para_SpeedBias[i] = [Vx,Vy,Vz, Bax,Bay,Baz, Bgx,Bgy,Bgz] (9차원), 그중 Ba(인덱스 3:6).
//   잔차: residual_i = weight_i * (Ba_i - target_i),  i ∈ {x,y,z}
class AccBiasPriorFactor : public ceres::SizedCostFunction<3, 9>
{
  public:
    AccBiasPriorFactor(const Eigen::Vector3d &weight, const Eigen::Vector3d &target)
        : weight_(weight), target_(target) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const
    {
        // acc bias 성분(인덱스 3,4,5)만 target 으로 당긴다. 속도(0:3)·gyro bias(6:9)는 무시.
        for (int i = 0; i < 3; ++i)
            residuals[i] = weight_(i) * (parameters[0][3 + i] - target_(i));

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            for (int i = 0; i < 3; ++i)
                J(i, 3 + i) = weight_(i);  // d residual_i / d Ba_i = weight_i
        }
        return true;
    }

  private:
    Eigen::Vector3d weight_;
    Eigen::Vector3d target_;
};
