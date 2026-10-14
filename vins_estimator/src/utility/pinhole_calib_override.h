#pragma once
// [SW1-1962] 카메라 내부 값 단일 출처 — VINS 가 edie_vision 캘리브 파일을 직접 읽는다.
// 카메라 영상(image_gray)은 edie_vision 이 이미 편(정류) 영상이라 VINS 카메라 모델 = 정류 P(왜곡 0).
// 그래서 ROS camera_info 형식 캘리브의 projection_matrix 에서 fx·fy·cx·cy 만 가져온다.

#include <string>

#include "camodocal/camera_models/Camera.h"

namespace vins_calib
{

struct RectPinhole
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  // 캘리브 파일에 image_width/height 가 있으면 그 값(없으면 0 = 모름). 설정 해상도와 다르면 쓰지 않는다.
  int width = 0;
  int height = 0;
};

// edie_vision pinhole_*_calib.yaml 의 projection_matrix(3x4, 정류 P)에서 fx·fy·cx·cy 를 읽는다.
// 파일이 없거나 형식이 틀리거나 fx·fy 가 양수가 아니면 false + err 에 이유(영어 — 로봇 터미널에서 한글이 깨짐).
bool ReadRectifiedPinhole(const std::string& path, RectPinhole* out, std::string* err);

// base(vio_edie.yaml 로 만든 카메라)의 이름·해상도는 그대로 두고, 내부 값만 k 로 바꾼 핀홀(왜곡 0)을 만든다.
// base 가 핀홀이 아니거나 캘리브 해상도가 base 와 다르면 nullptr + err — 그때는 덮어쓰지 않는다.
camodocal::CameraPtr MakeRectifiedPinhole(const camodocal::CameraPtr& base, const RectPinhole& k,
                                          std::string* err = nullptr);

}  // namespace vins_calib
