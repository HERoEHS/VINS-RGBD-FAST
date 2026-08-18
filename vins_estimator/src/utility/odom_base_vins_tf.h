#pragma once
// [docking] odom → base_vins TF 합성 — 순수 함수 (ROS 의존 없음, gtest 대상)
//
// 왜 이 간선이 필요한가:
//   도킹 앵커(odom→station_docking_obs)와 전파 오도메트리가 "같은 추정기"를 써야
//   두 시점 pose의 차에서 드리프트가 상쇄된다. VINS 추정을 odom 트리 안에서
//   소비할 수 있도록, body pose에 body→base_footprint 환산을 합성해
//   odom→base_vins 로 발행한다 (EKF 시절 odom→base_ekf 병행 발행 선례와 동일 패턴).
//
// ⚠️ child가 base_vins(새 이름)인 이유: odom→body 로 쏘면 body의 부모가
//   map(기존)과 odom 둘이 되어 TF 트리가 플리핑한다 (프레임당 부모 1개 규칙).

#include <Eigen/Dense>

namespace vins_tf
{

// URDF base_joint 역방향: base_link는 바퀴축 높이(base_z = wheel_radius 0.04)에 있음.
// registerPub()의 vins/base_footprint 정적 TF와 반드시 같은 값이어야 한다.
constexpr double kBaseZWheelRadius = 0.04;

/// body pose(map 기준이든 무엇이든)에 body→base_footprint 환산을 오른쪽 합성.
///   T_out = T_pose ∘ T_body_base_footprint
///   T_body_base_footprint = (R_io, t_io + R_io·[0,0,-base_z])
///     — registerPub()의 body→vins/base_link(RIO,TIO)→vins/base_footprint(-z) 체인과 동일.
/// @param P_body   body 위치
/// @param Q_body   body 자세
/// @param R_io     휠 extrinsic 회전 (parameters RIO)
/// @param t_io     휠 extrinsic 병진 (parameters TIO)
/// @param base_z   바퀴 반지름 (기본 kBaseZWheelRadius)
/// @param P_out    [out] base_footprint 위치
/// @param Q_out    [out] base_footprint 자세
inline void composeBaseFootprint(const Eigen::Vector3d &P_body,
                                 const Eigen::Quaterniond &Q_body,
                                 const Eigen::Matrix3d &R_io,
                                 const Eigen::Vector3d &t_io,
                                 double base_z,
                                 Eigen::Vector3d &P_out,
                                 Eigen::Quaterniond &Q_out)
{
    // body→base_footprint 를 한 번에: 병진 = t_io + R_io·[0,0,-z], 회전 = R_io
    const Eigen::Vector3d t_body_bfp =
        t_io + R_io * Eigen::Vector3d(0.0, 0.0, -base_z);
    P_out = P_body + Q_body * t_body_bfp;
    Q_out = (Q_body * Eigen::Quaterniond(R_io)).normalized();
}

}  // namespace vins_tf
