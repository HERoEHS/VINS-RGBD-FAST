#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// [SW1-1837] Body-frame NHC (비홀로노믹 제약, planar-motion Level 2) — 레버암 보상 포함(07-02 v2)
//   지면 차동구동 로봇의 참 kinematics: "바퀴 프레임 원점"의 횡(vy)·수직(vz) 속도 ≈ 0.
//   (바퀴는 구르는 방향으로만 움직인다 — 옆으로 미끄러지지도, 바닥을 뚫거나 뜨지도 않는다.)
//
//   ★월드 vz 제약(VerticalVelocityFactor, Level 1)과의 핵심 차이 = "같은 속도를 어느 축으로 재느냐":
//     · 평지(차체 수평): 월드 vz = 바디 vz = 0 → 두 제약 동일.
//     · 경사(pitch θ)를 오를 때: 월드 vz = |v|·sinθ ≠ 0 vs 바디 vz = 0 (여전히 참)
//     → 바디 제약은 지형 게이팅 없이 상시 적용 가능(지형 강건).
//   ⚠️ 뒷면(07-02 평지 A/B 실증): 바디 z축은 자세 bias와 함께 기울므로 pitch-bias가 만든
//     가짜 수직성분은 통과시킴 → 평지 z 억제력은 월드 vz보다 약함(z범위 313 vs 178mm).
//
//   ★레버암 보상(v2에서 추가된 이유, 07-02 평지 A/B 실증):
//     상태 V는 'IMU 위치'의 속도다. IMU가 회전축에서 t_io(≈10.6cm)만큼 떨어져 있으면 제자리회전
//     (ω≈3rad/s) 중 IMU엔 실제 횡속도 ω×|t_io|≈0.32m/s가 생긴다. 이를 보상 없이 0으로 강제하면
//     (σ=3mm/s 대비 100σ 모순) 오차가 xy로 주입됨 — v1 실측 xy 0.126→0.226(+80%) 악화의 원인.
//     보상: 바퀴 원점 속도 = V_imu + R_i·((ω_meas − Bg) × t_io) 를 제약 대상으로 사용.
//
//   잔차 (2행):
//     v_wheel = qio⁻¹ · ( Qi⁻¹·V + (ω_meas − Bg) × t_io )   (바퀴 '원점'의 바퀴 프레임 속도)
//     residual[0] = w_y · v_wheel.y   (횡방향 — 무슬립)
//     residual[1] = w_z · v_wheel.z   (수직 — 바닥 뚫음/뜸 없음)
//
//   파라미터 블록: [0] pose(7: q만 사용), [1] speed_bias(9: V[0..2]·Bg[6..8] 사용).
//   상수(생성자): qio·t_io(휠 extrinsic — 파라미터 블록로 넣으면 use_wheel:0일 때 자유 gauge 위험),
//                 ω_meas(해당 프레임의 raw gyro 측정. bias는 상태 Bg로 잔차 안에서 보정).
//   범프/문턱/슬립 구간(순간 vy·vz≠0)의 factor-skip 게이팅은 후속 단계.
//   가중치 σ≈1/w [m/s]: 잡을 드리프트 vz가 mm/s 수준이라 w는 수백이어야 물린다(w20 무효 실증).
class BodyNhcFactor
{
  public:
    BodyNhcFactor(const Eigen::Quaterniond &qio, const Eigen::Vector3d &tio,
                  const Eigen::Vector3d &gyr_meas, double w_y, double w_z)
        : qio_(qio), tio_(tio), gyr_meas_(gyr_meas), w_y_(w_y), w_z_(w_z) {}

    template <typename T>
    bool operator()(const T *const pose, const T *const speed_bias, T *residual) const
    {
        // 파라미터 저장 순서: quaternion은 [x,y,z,w] → Eigen 생성자는 (w,x,y,z)
        const Eigen::Quaternion<T>   Qi(pose[6], pose[3], pose[4], pose[5]);
        const Eigen::Matrix<T, 3, 1> V(speed_bias[0], speed_bias[1], speed_bias[2]);
        const Eigen::Matrix<T, 3, 1> Bg(speed_bias[6], speed_bias[7], speed_bias[8]);

        const Eigen::Quaternion<T>   qio = qio_.cast<T>();
        const Eigen::Matrix<T, 3, 1> tio = tio_.cast<T>();
        const Eigen::Matrix<T, 3, 1> omega = gyr_meas_.cast<T>() - Bg;  // 참 각속도(IMU 프레임)

        // 바퀴 원점 속도(IMU 프레임) = IMU 속도(IMU 프레임) + 레버암 항 ω×t_io
        const Eigen::Matrix<T, 3, 1> v_imu_body = Qi.conjugate() * V;
        const Eigen::Matrix<T, 3, 1> v_wheel    = qio.conjugate() * (v_imu_body + omega.cross(tio));
        residual[0] = T(w_y_) * v_wheel[1];
        residual[1] = T(w_z_) * v_wheel[2];
        return true;
    }

    // ceres CostFunction 생성 (AutoDiff<잔차2, pose7, speed_bias9>)
    static ceres::CostFunction *Create(const Eigen::Quaterniond &qio, const Eigen::Vector3d &tio,
                                       const Eigen::Vector3d &gyr_meas, double w_y, double w_z)
    {
        return new ceres::AutoDiffCostFunction<BodyNhcFactor, 2, 7, 9>(
            new BodyNhcFactor(qio, tio, gyr_meas, w_y, w_z));
    }

  private:
    Eigen::Quaterniond qio_;       // 휠 extrinsic 회전(IMU→바퀴), 상수
    Eigen::Vector3d    tio_;       // 휠 원점 위치(IMU 프레임), 상수 — 레버암
    Eigen::Vector3d    gyr_meas_;  // 해당 프레임 raw gyro 측정 (bias는 상태 Bg로 보정)
    double             w_y_, w_z_;
};
