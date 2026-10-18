#pragma once
// [SW1-1959] 로봇별 카메라–IMU 외부값 — <로봇 설정 폴더>/vins/extrinsic.yaml 이 vio_edie.yaml 의 같은 키를 덮어쓴다.
// vio_edie.yaml 은 이미지(/opt) 공통 설정이라 업데이트 때마다 바뀐다 → 로봇마다 다른 값은 업데이트에도 남는 폴더에 둔다.

#include <string>

#include <eigen3/Eigen/Dense>

namespace vins_calib
{

struct ImuCamExtrinsic
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // extrinsicRotation: R_imu_cam
  Eigen::Vector3d t = Eigen::Vector3d::Zero();      // extrinsicTranslation: t_imu_cam [m]
  bool has_td = false;                              // td 키가 있을 때만 덮는다
  double td = 0.0;
  // 로그에만 쓴다(판정에 안 씀).
  std::string robot_id;
  std::string source;
  std::string calibrated_at;
};

// OpenCV FileStorage yaml(vio_edie.yaml 과 같은 꼴)에서 extrinsicRotation(3x3)·extrinsicTranslation(3x1)·td(선택)를 읽는다.
// 둘 중 하나라도 없거나 크기가 틀리거나, 값이 유한하지 않거나, 회전이 직교 행렬이 아니면(det≈1, RᵀR≈I, 허용 1e-3)
// false + err 에 이유(영어 — 로봇 터미널에서 한글이 깨짐). 그때는 파일 전체를 쓰지 않는다.
bool ReadImuCamExtrinsic(const std::string& path, ImuCamExtrinsic* out, std::string* err);

}  // namespace vins_calib
