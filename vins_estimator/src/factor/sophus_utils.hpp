#pragma once
// VIW-Fusion의 sophus_utils.hpp(원본: Basalt, BSD-3)에서 휠 factor에 필요한
// SO(3) right Jacobian 2개만 추출. 번들 Sophus(SO3/Constants)에 의존.
#include <sophus/so3.h>
#include <eigen3/Eigen/Dense>
#include <cassert>
#include <cmath>

namespace Sophus {

// J: exp(phi + eps) ≈ exp(phi) exp(J eps) 의 right Jacobian
template <typename Derived1, typename Derived2>
inline void rightJacobianSO3(const Eigen::MatrixBase<Derived1> &phi,
                             const Eigen::MatrixBase<Derived2> &J_phi) {
  using Scalar = typename Derived1::Scalar;
  Eigen::MatrixBase<Derived2> &J =
      const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

  Scalar phi_norm2 = phi.squaredNorm();
  Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
  Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

  J.setIdentity();
  if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
    Scalar phi_norm = std::sqrt(phi_norm2);
    Scalar phi_norm3 = phi_norm2 * phi_norm;
    J -= phi_hat * (1 - std::cos(phi_norm)) / phi_norm2;
    J += phi_hat2 * (phi_norm - std::sin(phi_norm)) / phi_norm3;
  } else {
    // 0 근방 Taylor 전개
    J -= phi_hat / 2;
    J += phi_hat2 / 6;
  }
}

// J: log(exp(phi) exp(eps)) ≈ phi + J eps 의 right inverse Jacobian
template <typename Derived1, typename Derived2>
inline void rightJacobianInvSO3(const Eigen::MatrixBase<Derived1> &phi,
                                const Eigen::MatrixBase<Derived2> &J_phi) {
  using Scalar = typename Derived1::Scalar;
  Eigen::MatrixBase<Derived2> &J =
      const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

  Scalar phi_norm2 = phi.squaredNorm();
  Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
  Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

  J.setIdentity();
  J += phi_hat / 2;

  if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
    Scalar phi_norm = std::sqrt(phi_norm2);
    // 입력 각도는 [0, pi] 범위 가정 (보통 Log() 결과라 충족)
    assert(phi_norm <= M_PI + Sophus::Constants<Scalar>::epsilon());
    if (phi_norm < M_PI - Sophus::Constants<Scalar>::epsilonSqrt()) {
      J += phi_hat2 * (1 / phi_norm2 - (1 + std::cos(phi_norm)) /
                                           (2 * phi_norm * std::sin(phi_norm)));
    } else {
      // pi 근방 0차 Taylor
      J += phi_hat2 / (M_PI * M_PI);
    }
  } else {
    // 0 근방 Taylor 전개
    J += phi_hat2 / 12;
  }
}

}  // namespace Sophus
