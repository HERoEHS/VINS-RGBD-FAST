#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// [SW1-1866] 정지 상대운동 잠금 factor — 동적 장애물 오염의 '새 프레임 배치 오차' 차단.
//
//   배경(8차 판별로 실측 확정): 초근접 동적 장애물이 화면을 점령하면 오염된 비전 잔차의
//   정보량 합이 IMU의 상대회전 제약(σ≈0.036°/구간, 수십σ)을 이겨 새 프레임을 매 solve
//   ~1° 돌아간 채로 배치한다(과거 프레임 슬라이드는 전부 0으로 실측 — 사후 가드의 사각).
//   정지 확정(휠+gyro 합의) 중에는 'Δpose=0'이라는 외부 사실이 있으므로, 인접 프레임 간
//   상대 위치·상대 yaw를 0으로 당기는 관측을 최적화 안에 직접 주입해 배치 오차를
//   태어나는 순간 저지한다.
//
//   ⚠️ 과제약 회피 설계(ZUPT 2026-06-26 사고·gravity_align 교훈 계승):
//     - 파라미터는 pose 두 개뿐 — bias/속도를 안 넣는다(모호성 재수입 방지)
//     - roll/pitch는 제약하지 않는다(중력 관측=IMU 관할, 이중 제약 회피). yaw+위치만.
//     - 정지 판정이 깨지면(주행 재개) 호출측이 factor를 안 넣으므로 자동 해제.
//   잔차 4차원: [w_p·(R_iᵀ(P_j−P_i)) (3), w_y·상대yaw (1)]
//   상대 yaw는 소각 근사(상대 쿼터니언 z성분×2) — 정지 중 상대 회전은 ≤수° 전제라 충분.
class StillMotionFactor
{
  public:
    StillMotionFactor(double pos_weight, double yaw_weight)
        : w_p_(pos_weight), w_y_(yaw_weight) {}

    template <typename T>
    bool operator()(const T *const pose_i, const T *const pose_j, T *residual) const
    {
        // 파라미터 저장 순서: [Px,Py,Pz, qx,qy,qz,qw] → Eigen 생성자는 (w,x,y,z)
        const Eigen::Matrix<T, 3, 1> Pi(pose_i[0], pose_i[1], pose_i[2]);
        const Eigen::Matrix<T, 3, 1> Pj(pose_j[0], pose_j[1], pose_j[2]);
        const Eigen::Quaternion<T>   Qi(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);
        const Eigen::Quaternion<T>   Qj(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);

        // 상대 위치 (i 바디 기준) = 0 이어야 함
        const Eigen::Matrix<T, 3, 1> dp = Qi.conjugate() * (Pj - Pi);
        residual[0] = T(w_p_) * dp[0];
        residual[1] = T(w_p_) * dp[1];
        residual[2] = T(w_p_) * dp[2];

        // 상대 yaw = 0 (소각: 상대 쿼터니언 z성분×2 ≈ 회전벡터 z성분 [rad])
        const Eigen::Quaternion<T> rel = Qi.conjugate() * Qj;
        residual[3] = T(w_y_) * T(2.0) * rel.z();
        return true;
    }

    // ceres CostFunction 생성 (AutoDiff<잔차4, pose7, pose7>)
    static ceres::CostFunction *Create(double pos_weight, double yaw_weight)
    {
        return new ceres::AutoDiffCostFunction<StillMotionFactor, 4, 7, 7>(
            new StillMotionFactor(pos_weight, yaw_weight));
    }

  private:
    double w_p_;  // 1/σ_p [1/m]   — 근거는 센서 노이즈가 아니라 정지 합의(외부 사실)
    double w_y_;  // 1/σ_yaw [1/rad]
};
