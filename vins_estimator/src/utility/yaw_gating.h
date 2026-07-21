#pragma once
// [SW1-1837] 고속 회전 시 비전 게이팅 — 핵심 판정 로직 (헤더 단독, gtest 가능)
//
// [왜]
//   GT 확정(v5/v7): gyro 단독 yaw는 −0.11%로 거의 완벽한데 VINS 융합이 −2%(−20°)까지
//   망친다. 원인 = 고속 스핀 중 특징점이 FOV 이탈·모션블러로 추적 실패 → 비전 재투영
//   factor가 회전을 under-count → 좋은 gyro yaw를 오염. 사용자 EKF 실측(비전 없는 gyro+휠
//   융합이 yaw 양호)이 독립 확증. ⇒ 고속 회전 프레임의 비전 관측을 최적화·marg에서 제외해
//   그 구간을 IMU preintegration(=gyro, 거의 완벽)에 맡긴다.
//
// [게이트 조건]
//   한 프레임의 자이로 샘플 평균에서 자이로 bias를 뺀 각속도 크기 |mean(ω) − Bg| 가
//   임계 초과면 그 프레임 = '고속 회전' → 그 프레임을 관측 끝점으로 하는 재투영 factor skip.
#include <Eigen/Dense>
#include <vector>

namespace yaw_gating
{

// 프레임의 평균 각속도 크기 [rad/s] (bias 보정). 샘플 없으면 0.
inline double frameAngularSpeed(const std::vector<Eigen::Vector3d> &gyr_samples,
                               const Eigen::Vector3d &bg)
{
    if (gyr_samples.empty())
        return 0.0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &w : gyr_samples)
        mean += w;
    mean /= static_cast<double>(gyr_samples.size());
    return (mean - bg).norm();
}

// 고속 회전 프레임인가 = 평균 각속도 크기 > 임계.
//   thresh_radps <= 0 이면 항상 false(게이트 사실상 비활성, 안전).
inline bool isFastRotation(const std::vector<Eigen::Vector3d> &gyr_samples,
                          const Eigen::Vector3d &bg, double thresh_radps)
{
    if (thresh_radps <= 0.0)
        return false;
    return frameAngularSpeed(gyr_samples, bg) > thresh_radps;
}

}  // namespace yaw_gating
