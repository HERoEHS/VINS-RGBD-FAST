#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// [SW1-1837] Ground-plane constraint — VIW-Fusion plane_factor 이식 (AutoDiff 재구현)
//   바디(바퀴)를 '추정된 월드 지면 평면'에 묶는다:
//     ① 자세: 바퀴 프레임 z축(법선)이 평면 법선과 정렬 → roll/pitch 수평화 (근본 pitch 교정)
//     ② z:   바퀴 고도가 평면 높이 zpw와 일치 → z 절대 묶음
//   → vz-only(VerticalVelocityFactor)가 xy로 오차 전가한 것과 달리, 자세까지 잡아 근본 교정.
//   ⚠️ 단일 전역평면 가정(경사/다층 미대응) → 지형변화 게이팅은 후속.
//
//   파라미터 블록 (VIW-Fusion 원본과 동일 순서·차원):
//     [0] pose       (7: Px,Py,Pz, qx,qy,qz,qw)   para_Pose[i]
//     [1] wheel extr (7: tx,ty,tz, qx,qy,qz,qw)   para_Ex_Pose_wheel[0]
//     [2] plane rot  (4: qx,qy,qz,qw)             para_plane_R[0]  (ceres EigenQuaternionParameterization)
//     [3] plane z    (1: zpw)                     para_plane_Z[0]
//   잔차(원본 코드 검증, gam=Identity):
//     row0-1 = (Rio^T · Ri^T · Rpw^T · e3)[x,y]     (법선 정렬 오차의 x,y = pitch,roll)
//     row2   = zpw + (Rpw · (Pi + Ri·tio))[z]       (바퀴 고도 vs 평면 높이)
//   가중치 sqrt_info = diag(pitch_n_inv, roll_n_inv, zpw_n_inv).
//   ※ 해석적 야코비안 대신 AutoDiff — 잔차만으로 미분 자동(무버그). 평면회전은 EigenQuaternionParameterization.
class PlaneFactor
{
  public:
    PlaneFactor(double pitch_n_inv, double roll_n_inv, double zpw_n_inv)
        : pitch_n_inv_(pitch_n_inv), roll_n_inv_(roll_n_inv), zpw_n_inv_(zpw_n_inv) {}

    template <typename T>
    bool operator()(const T *const pose, const T *const ex_wheel,
                    const T *const plane_R, const T *const plane_Z, T *residual) const
    {
        // 파라미터 저장 순서: quaternion은 [x,y,z,w] → Eigen 생성자는 (w,x,y,z)
        const Eigen::Matrix<T, 3, 1> Pi(pose[0], pose[1], pose[2]);
        const Eigen::Quaternion<T>   Qi(pose[6], pose[3], pose[4], pose[5]);
        const Eigen::Matrix<T, 3, 1> tio(ex_wheel[0], ex_wheel[1], ex_wheel[2]);
        const Eigen::Quaternion<T>   qio(ex_wheel[6], ex_wheel[3], ex_wheel[4], ex_wheel[5]);
        const Eigen::Quaternion<T>   qpw(plane_R[3], plane_R[0], plane_R[1], plane_R[2]);
        const T                      zpw = plane_Z[0];

        const Eigen::Matrix<T, 3, 1> e3(T(0), T(0), T(1));
        // 평면 법선(e3)을 바퀴 프레임으로 이동: n = Rio^T · Ri^T · Rpw^T · e3
        //   바퀴 z축이 평면 법선과 정렬되면 n=(0,0,1) → n.x,n.y = 0 (roll/pitch 오차).
        const Eigen::Matrix<T, 3, 1> n =
            qio.conjugate() * (Qi.conjugate() * (qpw.conjugate() * e3));
        residual[0] = T(pitch_n_inv_) * n[0];
        residual[1] = T(roll_n_inv_) * n[1];

        // 바퀴 위치를 평면 프레임으로: pw.z 가 -zpw 여야 (평면 위).
        const Eigen::Matrix<T, 3, 1> pw = qpw * (Pi + Qi * tio);
        residual[2] = T(zpw_n_inv_) * (zpw + pw[2]);
        return true;
    }

    // ceres CostFunction 생성 (AutoDiff<잔차3, pose7, wheel-extr7, plane-rot4, plane-z1>)
    static ceres::CostFunction *Create(double pitch_n_inv, double roll_n_inv, double zpw_n_inv)
    {
        return new ceres::AutoDiffCostFunction<PlaneFactor, 3, 7, 7, 4, 1>(
            new PlaneFactor(pitch_n_inv, roll_n_inv, zpw_n_inv));
    }

  private:
    double pitch_n_inv_, roll_n_inv_, zpw_n_inv_;
};
