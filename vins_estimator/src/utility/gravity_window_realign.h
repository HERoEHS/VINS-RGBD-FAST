#pragma once
// [SW1-1837] 정지 시 중력 재정렬 v2 — '창 전체 자세 보정'의 핵심 수학 (헤더 단독, gtest 가능)
//
// soft factor(1차, use_gravity_align:1 — w200/w1000 A/B로 기각)와의 차이:
//   1차는 최적화 '안'에서 vision·marg prior와 줄다리기 → 자세를 못 돌리고 오차가 xy로 전가.
//   v2는 최적화 '밖'에서 정지 확정 시 1회, 윈도우 전 상태(Ps·Rs·Vs)와 marg prior
//   선형화점을 같은 ΔR로 일괄 회전한다. 균일 강체 변환이라 상대 pose(R_ij, t_ij)가
//   불변이고, 특징점은 앵커 프레임 역깊이 기반이라 재투영 잔차도 불변 = xy 전가 원리 차단.
//
// 소각 근사 주의(검증 관문 ⑤): marg prior의 야코비안은 회전하지 않는다 — 그 오차는
//   상태-선형화점 차(작음)와 보정각(작음)의 곱으로 2차 미소량이며, 보정각 상한(≤3°)과
//   세트로만 정당하다. 상한 없이 큰 각을 돌리면 근사가 깨진다.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace gravity_realign
{

// 실측 중력 방향으로부터 월드 프레임 보정 회전 ΔR을 계산한다.
//   u_meas_body: 정지 중 (acc 평균 − Ba) 정규화 = 바디 프레임에서 잰 '위' 방향
//   R_wb:        현재 자세 추정 (body→world)
//   max_angle:   1회 보정 상한 [rad] — 초과분은 클램프(다음 정지에서 마저 보정)
//   angle_out:   클램프 전 자세 오차 [rad] (발동 임계 판단용)
// 적용은 R' = ΔR·R. 회전축 = (R·u_meas)×e_z 는 z성분이 구조적으로 0(수평축)이라
// 참 yaw를 건드리지 못한다(수평 오차가 정확히 역회전됨). 단 atan2 기준 yaw 좌표값은
// 보정각의 2차 미소량(θ²)만큼 움직일 수 있다 — ≤3° 상한에서 ≤0.16° 수준.
inline Eigen::Matrix3d computeDeltaR(const Eigen::Vector3d &u_meas_body,
                                     const Eigen::Matrix3d &R_wb,
                                     double max_angle,
                                     double *angle_out = nullptr)
{
    // 자세가 정확하다면 실측 '위'를 월드로 옮겼을 때 e_z와 일치해야 한다
    const Eigen::Vector3d w_meas = (R_wb * u_meas_body.normalized()).normalized();
    const Eigen::Vector3d e_z(0.0, 0.0, 1.0);
    // std::clamp 미사용 — 테스트 타깃이 C++17 미만으로 빌드될 수 있음
    const double cos_a = std::max(-1.0, std::min(1.0, w_meas.dot(e_z)));
    const double angle = std::acos(cos_a);
    if (angle_out)
        *angle_out = angle;

    Eigen::Vector3d axis = w_meas.cross(e_z);
    const double axis_norm = axis.norm();
    if (axis_norm < 1e-12)
        return Eigen::Matrix3d::Identity();  // 이미 정렬(반평행은 상류 발동 임계가 걸러냄)
    axis /= axis_norm;
    return Eigen::AngleAxisd(std::min(angle, max_angle), axis).toRotationMatrix();
}

// ceres pose 블록(x y z qx qy qz qw)을 pivot 기준 ΔR로 강체 회전.
//   위치: p' = pivot + ΔR(p − pivot) — pivot 지점은 불변(현재 위치 텔레포트 방지)
//   자세: q' = ΔR ⊗ q
inline void rotatePoseBlock(double *pose, const Eigen::Matrix3d &dR, const Eigen::Vector3d &pivot)
{
    Eigen::Map<Eigen::Vector3d> p(pose);
    p = pivot + dR * (p - pivot);
    // Eigen 내부 계수 순서 [x,y,z,w] = para_Pose[3..6] 배치와 동일
    Eigen::Map<Eigen::Quaterniond> q(pose + 3);
    q = (Eigen::Quaterniond(dR) * q).normalized();
}

// ceres speed-bias 블록(vx vy vz | ba×3 | bg×3): 속도는 월드량이라 회전, Bg는 불변.
//   d_ba는 모드 3(Ba 동시 재설정) 전용 — 창의 Ba를 균일 이동할 때 marg prior 선형화점도
//   같은 양만큼 이동해야 prior가 옛 Ba로 되돌리는 힘을 만들지 않는다. 기본 0(모드 2 동작).
inline void rotateSpeedBiasBlock(double *sb, const Eigen::Matrix3d &dR,
                                 const Eigen::Vector3d &d_ba = Eigen::Vector3d::Zero())
{
    Eigen::Map<Eigen::Vector3d> v(sb);
    v = dR * v;
    Eigen::Map<Eigen::Vector3d> ba(sb + 3);
    ba += d_ba;
}

// (모드 3) 창 회전 후에도 '정지 IMU 잔차 = 0'이 유지되게 하는 Ba 재설정값.
//   정지 모델: acc_mean = R_wbᵀ·(g·e_z) + Ba  →  Ba_new = acc_mean − g·R_newᵀ·e_z
//   모드 2는 (acc−Ba)를 표적으로 삼아 Ba가 이미 흡수한 자세 오차를 원리적으로 못 본다
//   (odom_fix_check A/B 실증: 긴 정지의 raw acc 오차 ~3°가 (acc−Ba)로는 <0.5°로 보임).
//   모드 3은 raw acc를 표적으로 자세를 돌리고, 그 자세와 정합하는 값으로 Ba를 명시 이동한다.
//   전제(참 Ba 수평성분≈0)의 근거: static init이 초기 정지 acc로 중력을 앵커하므로
//   이 시스템의 중력 규약에서 초기 자세-acc 정합 오차는 0(실측 0.00~0.07°) — 이후 벌어진
//   차이는 자세 드리프트가 Ba로 흡수된 것이다.
inline Eigen::Vector3d computeConsistentBa(const Eigen::Vector3d &acc_mean_body,
                                           const Eigen::Matrix3d &R_wb_new,
                                           double gravity_norm)
{
    return acc_mean_body - gravity_norm * (R_wb_new.transpose() * Eigen::Vector3d(0, 0, 1));
}

}  // namespace gravity_realign
