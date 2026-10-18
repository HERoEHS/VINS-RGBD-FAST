#include "extrinsic_override.h"

#include <cmath>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace vins_calib
{

namespace
{

// 직교성 허용치 — vio_edie.yaml 처럼 소수 8~10 자리로 적은 회전도 넉넉히 통과하고, 잘못 옮겨 적은 값은 걸린다.
constexpr double kOrthoTol = 1e-3;

bool ReadMatrix(const cv::FileStorage& fs, const char* key, int rows, int cols, cv::Mat* out,
                std::string* err)
{
  const cv::FileNode node = fs[key];
  if (node.empty())
  {
    *err = std::string(key) + " missing";
    return false;
  }
  cv::Mat m;
  node >> m;
  if (m.rows != rows || m.cols != cols || m.channels() != 1)
  {
    *err = std::string(key) + " must be " + std::to_string(rows) + "x" + std::to_string(cols) + " (got " +
           std::to_string(m.rows) + "x" + std::to_string(m.cols) + ")";
    return false;
  }
  m.convertTo(*out, CV_64F);
  if (!cv::checkRange(*out))
  {
    *err = std::string(key) + " has non-finite values";
    return false;
  }
  return true;
}

std::string ReadString(const cv::FileStorage& fs, const char* key)
{
  const cv::FileNode node = fs[key];
  return node.isString() ? static_cast<std::string>(node) : std::string();
}

}  // namespace

bool ReadImuCamExtrinsic(const std::string& path, ImuCamExtrinsic* out, std::string* err)
{
  std::string why;
  try
  {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      *err = path + ": cannot open";
      return false;
    }
    cv::Mat R, t;
    if (!ReadMatrix(fs, "extrinsicRotation", 3, 3, &R, &why) ||
        !ReadMatrix(fs, "extrinsicTranslation", 3, 1, &t, &why))
    {
      *err = path + ": " + why;
      return false;
    }
    ImuCamExtrinsic ex;
    cv::cv2eigen(R, ex.R);
    cv::cv2eigen(t, ex.t);
    const double det = ex.R.determinant();
    const double ortho = (ex.R.transpose() * ex.R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
    if (std::abs(det - 1.0) > kOrthoTol || ortho > kOrthoTol)
    {
      *err = path + ": extrinsicRotation is not a rotation (det=" + std::to_string(det) +
             ", max|R^T R - I|=" + std::to_string(ortho) + ")";
      return false;
    }
    const cv::FileNode td = fs["td"];
    if (!td.empty())
    {
      if (!td.isReal() && !td.isInt())
      {
        *err = path + ": td must be a number";
        return false;
      }
      ex.td = static_cast<double>(td);
      if (!std::isfinite(ex.td))
      {
        *err = path + ": td is not finite";
        return false;
      }
      ex.has_td = true;
    }
    ex.robot_id = ReadString(fs, "robot_id");
    ex.source = ReadString(fs, "source");
    ex.calibrated_at = ReadString(fs, "calibrated_at");
    *out = ex;
    return true;
  }
  catch (const std::exception& e)  // cv::Exception — yaml 문법이 깨진 파일
  {
    *err = path + ": " + e.what();
    return false;
  }
}

}  // namespace vins_calib
