#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// [SW1-1837] Body-frame NHC (비홀로노믹 제약, planar-motion Level 2)
//   지면 차동구동 로봇의 참 kinematics: 바퀴 프레임 기준 횡방향(vy)·수직(vz) 속도 ≈ 0.
//   (바퀴는 구르는 방향으로만 움직인다 — 옆으로 미끄러지지도, 바닥을 뚫거나 뜨지도 않는다.)
//
//   ★월드 vz 제약(VerticalVelocityFactor, Level 1)과의 핵심 차이 = "같은 속도를 어느 축으로 재느냐":
//     · 평지(차체 수평): 월드 vz = 바디 vz = 0 → 두 제약 동일.
//     · 경사(pitch θ)를 오를 때: 월드 vz = |v|·sinθ ≠ 0 (실제로 고도가 오르는 중)
//                                바디 vz = 0        (차체 기준으론 그냥 전진일 뿐)
//     → 월드 vz=0 강제는 경사에서 '틀린 제약'(오차 주입, 게이팅 필요)이지만,
//       바디 vz=0은 경사에서도 여전히 참 → 지형 게이팅 없이 상시 적용 가능(지형 강건).
//
//   잔차 (2행):
//     v_body = qio^-1 · Qi^-1 · V_world   (월드 속도를 바퀴 프레임으로 회전)
//     residual[0] = w_y · v_body.y   (횡방향 — 무슬립)
//     residual[1] = w_z · v_body.z   (수직 — 바닥 뚫음/뜸 없음)
//
//   파라미터 블록: [0] pose(7: P,q — q만 사용), [1] speed_bias(9: V,Ba,Bg — V만 사용).
//   qio(휠 extrinsic 회전)는 '상수'로 생성자에 받는다 — 파라미터 블록으로 넣으면 use_wheel:0일 때
//   자유 회전 gauge가 생겨 편향 방향으로 흘러갈 위험(plane 자유법선의 tilt 각인과 동일 문제, 07-01 실증).
//
//   알려진 단순화: 레버암 항(ω × t_io) 무시 — 제자리회전 시 몇 cm/s 수준이며 소프트 σ가 흡수.
//   범프/문턱/슬립 구간(순간적으로 바디 vy·vz≠0)의 factor-skip 게이팅은 후속 단계.
//   가중치 σ≈1/w [m/s]: 잡을 드리프트 vz가 mm/s 수준이라 w는 수백이어야 물린다(w20 무효 실증).
class BodyNhcFactor
{
  public:
    BodyNhcFactor(const Eigen::Quaterniond &qio, double w_y, double w_z)
        : qio_(qio), w_y_(w_y), w_z_(w_z) {}

    template <typename T>
    bool operator()(const T *const pose, const T *const speed_bias, T *residual) const
    {
        // 파라미터 저장 순서: quaternion은 [x,y,z,w] → Eigen 생성자는 (w,x,y,z)
        const Eigen::Quaternion<T>   Qi(pose[6], pose[3], pose[4], pose[5]);
        const Eigen::Matrix<T, 3, 1> V(speed_bias[0], speed_bias[1], speed_bias[2]);
        const Eigen::Quaternion<T>   qio = qio_.cast<T>();

        // 월드 속도 → IMU 프레임 → 바퀴 프레임
        const Eigen::Matrix<T, 3, 1> v_body = qio.conjugate() * (Qi.conjugate() * V);
        residual[0] = T(w_y_) * v_body[1];
        residual[1] = T(w_z_) * v_body[2];
        return true;
    }

    // ceres CostFunction 생성 (AutoDiff<잔차2, pose7, speed_bias9>)
    static ceres::CostFunction *Create(const Eigen::Quaterniond &qio, double w_y, double w_z)
    {
        return new ceres::AutoDiffCostFunction<BodyNhcFactor, 2, 7, 9>(
            new BodyNhcFactor(qio, w_y, w_z));
    }

  private:
    Eigen::Quaterniond qio_;  // 휠 extrinsic 회전(IMU→바퀴), 상수
    double             w_y_, w_z_;
};
