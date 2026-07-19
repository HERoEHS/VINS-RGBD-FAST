#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// [SW1-1837] 정지 시 중력 재정렬 (gravity realignment at standstill)
//   목적: 다리 이벤트 게이팅(예방)이 못 막은 잔여 자세(roll/pitch) 오차의 사후 교정.
//        게이팅은 감쇠이지 차단이 아니라(07-14 실측: 정지 자세 오차 3.5→3.0°만 개선),
//        주입된 오차는 평지 약관측(자세↔acc-bias 뒤엉킴) 탓에 스스로 안 풀린다.
//   원리: 정지 중 가속도계는 중력만 감지 → 몸체의 절대 roll/pitch를 드리프트 없이 직접 관측.
//        부팅 시 IMU 드라이버가 bias·장착 기울기를 교정(imu_offsets.yaml)하므로
//        정지 창 acc 평균 ≈ 순수 중력 방향(잔여 Ba는 호출측이 현재 추정치로 차감).
//
//   잔차(3D, world-yaw 불변 — 유효 자유도 2):
//     pred = Qi^-1 · e_z   (world-up을 바디로 회전 — 정지 시 acc 방향의 예측치)
//     r = weight · (pred − g_meas_body)
//     단위벡터 차이므로 소각도에서 |r|/weight ≈ 자세 오차각[rad] → weight = 1/σ_angle.
//   world-yaw 불변 증명: Qi ← Rz(ψ)·Qi 여도 Qi^-1·Rz(−ψ)·e_z = Qi^-1·e_z (e_z는 Rz 불변).
//     → yaw는 이 factor로 관측 불가(중력과 수직) = 의도된 설계(자세만, yaw는 별도 과제).
//
//   ⚠️ 과제약 회피 설계: 파라미터는 pose 하나뿐 — bias(para_SpeedBias)를 넣지 않는다.
//     bias를 파라미터로 넣으면 자세-bias 모호성을 factor 안에 그대로 재수입해 정보가 0이 되고,
//     bias를 0으로 못 박으면 ZUPT+acc_bias_prior 동시 강제의 과제약 사고(2026-06-26)를 재현한다.
//     측정치의 Ba 차감은 호출측에서 '상수'로 처리(매 optimization마다 최신 추정으로 갱신됨).
class GravityAlignFactor
{
  public:
    GravityAlignFactor(const Eigen::Vector3d &g_meas_body, double weight)
        : g_meas_(g_meas_body.normalized()), weight_(weight) {}

    template <typename T>
    bool operator()(const T *const pose, T *residual) const
    {
        // 파라미터 저장 순서: [Px,Py,Pz, qx,qy,qz,qw] → Eigen 생성자는 (w,x,y,z)
        const Eigen::Quaternion<T> Qi(pose[6], pose[3], pose[4], pose[5]);
        const Eigen::Matrix<T, 3, 1> ez(T(0), T(0), T(1));
        const Eigen::Matrix<T, 3, 1> pred = Qi.conjugate() * ez;  // world-up의 바디 표현
        residual[0] = T(weight_) * (pred[0] - T(g_meas_[0]));
        residual[1] = T(weight_) * (pred[1] - T(g_meas_[1]));
        residual[2] = T(weight_) * (pred[2] - T(g_meas_[2]));
        return true;
    }

    // ceres CostFunction 생성 (AutoDiff<잔차3, pose7>)
    static ceres::CostFunction *Create(const Eigen::Vector3d &g_meas_body, double weight)
    {
        return new ceres::AutoDiffCostFunction<GravityAlignFactor, 3, 7>(
            new GravityAlignFactor(g_meas_body, weight));
    }

  private:
    Eigen::Vector3d g_meas_;  // 정지 창 acc 평균(−Ba, 정규화) = 바디 기준 중력(위) 방향 실측
    double weight_;           // 1/σ_angle [1/rad]
};
