#include "pinhole_calib_override.h"

#include <boost/make_shared.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "camodocal/camera_models/PinholeCamera.h"

namespace vins_calib
{

bool ReadRectifiedPinhole(const std::string& path, RectPinhole* out, std::string* err)
{
  try
  {
    // ROS camera_info 형식은 "%YAML:1.0" 머리가 없어 cv::FileStorage 로는 못 연다 → yaml-cpp.
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node data = root["projection_matrix"]["data"];
    if (!data || !data.IsSequence() || data.size() != 12)
    {
      *err = path + ": projection_matrix.data is not a 3x4 (12 values) matrix";
      return false;
    }
    // edie_vision CheckCalibrationData 와 같은 검사 — 빠진 키·틀린 형은 yaml-cpp 예외(아래 catch).
    const auto K = root["camera_matrix"]["data"].as<std::vector<double>>();
    const auto D = root["distortion_coefficients"]["data"].as<std::vector<double>>();
    root["distortion_coefficients"]["cols"].as<int>();
    const auto R = root["rectification_matrix"]["data"].as<std::vector<double>>();
    if (K.size() != 9)
    {
      *err = path + ": camera_matrix has " + std::to_string(K.size()) + " values (need 9)";
      return false;
    }
    if (D.empty())
    {
      *err = path + ": distortion_coefficients is empty";
      return false;
    }
    if (R.size() != 9)
    {
      *err = path + ": rectification_matrix has " + std::to_string(R.size()) + " values (need 9)";
      return false;
    }
    // !(x > 0) 로 NaN 도 걸러낸다.
    if (!(K[0] > 0.0) || !(K[4] > 0.0) || !std::isfinite(K[0]) || !std::isfinite(K[4]))
    {
      *err = path + ": camera_matrix fx/fy must be > 0";
      return false;
    }
    // P = [fx 0 cx Tx; 0 fy cy 0; 0 0 1 0]
    RectPinhole k;
    k.fx = data[0].as<double>();
    k.cx = data[2].as<double>();
    k.fy = data[5].as<double>();
    k.cy = data[6].as<double>();
    if (!(k.fx > 0.0) || !(k.fy > 0.0) || !std::isfinite(k.fx) || !std::isfinite(k.fy))
    {
      *err = path + ": fx/fy must be positive";
      return false;
    }
    if (root["image_width"] && root["image_height"])
    {
      k.width = root["image_width"].as<int>();
      k.height = root["image_height"].as<int>();
    }
    *out = k;
    return true;
  }
  catch (const std::exception& e)
  {
    *err = path + ": " + e.what();
    return false;
  }
}

camodocal::CameraPtr MakeRectifiedPinhole(const camodocal::CameraPtr& base, const RectPinhole& k,
                                          std::string* err)
{
  std::string why;
  if (!base || base->modelType() != camodocal::Camera::PINHOLE)
  {
    why = "config camera is not PINHOLE";
  }
  else if (k.width > 0 && (k.width != base->imageWidth() || k.height != base->imageHeight()))
  {
    // 다른 해상도로 잡은 캘리브면 픽셀 값이 맞지 않는다 → 쓰지 않는다.
    why = "calib resolution " + std::to_string(k.width) + "x" + std::to_string(k.height) +
          " != config " + std::to_string(base->imageWidth()) + "x" + std::to_string(base->imageHeight());
  }
  if (!why.empty())
  {
    if (err)
    {
      *err = why;
    }
    return nullptr;
  }
  // 정류된 영상이라 왜곡(k1·k2·p1·p2)은 0.
  return boost::make_shared<camodocal::PinholeCamera>(
      base->cameraName(), base->imageWidth(), base->imageHeight(),
      0.0, 0.0, 0.0, 0.0, k.fx, k.fy, k.cx, k.cy);
}

}  // namespace vins_calib
