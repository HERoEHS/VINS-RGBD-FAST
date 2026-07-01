#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// [SW1-1837] Vertical-velocity soft constraint (planar-motion Level 1 PoC)
//   목적: 지면 로봇은 평지 주행 시 월드 프레임 수직속도 Vz ≈ 0 이다. z-drift = ∫Vz 이므로
//        Vz를 0으로 약하게 상시 당기면 z 누적 자체가 억제된다.
//   왜 필요: 휠 factor는 평면 속도(vx,vy)만 제약하고 z는 못 잡는다. ZUPT는 '정지 프레임'에서만
//           속도 3D를 0으로 당긴다(주행 중엔 안 걸림). → 주행 중에도 상시 걸리는 z 전용 제약이 공백.
//           (acc_bias_prior는 z-drift에 무효로 확정됨 2026-07-01 → 그 대체 실험)
//   대상 블록: para_SpeedBias[i] = [Vx,Vy,Vz, Bax,Bay,Baz, Bgx,Bgy,Bgz] (9차원)
//             그중 Vz(인덱스 2)만 0으로 제약. residual = weight * Vz.
//   ※ ZeroVelocityFactor(선속도 3D)의 z 성분만 뽑아 '정지 게이트 없이 상시' 적용한 것.
//   ⚠️ 평지 가정: 경사로에선 진짜 Vz≠0을 거스른다(소프트라 완화). 지형변화 게이팅 + 바디프레임
//      NHC + plane_factor(자세까지)는 후속 단계(Level 2 / VIW-Fusion 이식). 검증은 배포조건
//      다회 A/B(scripts/ab_compare_multi.py + z_drift_motion.py).
class VerticalVelocityFactor : public ceres::SizedCostFunction<1, 9>
{
  public:
    explicit VerticalVelocityFactor(double weight) : weight_(weight) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const
    {
        // 월드 수직속도 Vz(인덱스 2)만 0으로 당긴다. 나머지 성분은 무시.
        residuals[0] = weight_ * parameters[0][2];

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 9, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            J(0, 2) = weight_;  // d residual / d Vz = weight
        }
        return true;
    }

  private:
    double weight_;
};
