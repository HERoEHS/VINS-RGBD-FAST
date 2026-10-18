// [SW1-1959] 로봇별 카메라–IMU 외부값(<로봇 설정 폴더>/vins/extrinsic.yaml) 읽기·검사.
// ① 정상 파일(td 있음/없음, 로그용 문자열) ② vio_edie.yaml 자체도 같은 꼴로 읽힌다
// ③ 키 빠짐·크기 틀림·회전 아님·숫자 아님·파일 없음·문법 깨짐 → 통째로 거부.
#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include <Eigen/Dense>  // cv2eigen 템플릿은 Eigen 을 먼저 include 해야 보인다
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "utility/extrinsic_override.h"

namespace
{

const std::string kVinsRoot = VINS_SOURCE_ROOT;  // CMake 가 주입 (VINS-RGBD-FAST 루트)
const std::string kVioYaml = kVinsRoot + "/config/elp_stereo_edie/vio_edie.yaml";

std::string WriteTemp(const std::string& name, const std::string& body)
{
  const std::string path = ::testing::TempDir() + name;
  std::ofstream(path) << body;
  return path;
}

std::string Matrix(const char* key, int rows, int cols, const std::string& data)
{
  return std::string(key) + ": !!opencv-matrix\n   rows: " + std::to_string(rows) + "\n   cols: " +
         std::to_string(cols) + "\n   dt: d\n   data: [" + data + "]\n";
}

// 실제 .127 계열 값 모양(90° 가까운 회전) — 직교 행렬이다.
const std::string kRot = "0.0, 0.5735764, 0.8191520, -1.0, 0.0, 0.0, 0.0, -0.8191520, 0.5735764";
const std::string kTrans = "0.19, 0.02, -0.10";

std::string Body(const std::string& extra = "")
{
  return "%YAML:1.0\nrobot_id: \"edie-127b\"\nsource: \"urdf\"\ncalibrated_at: \"2026-10-18\"\n" +
         Matrix("extrinsicRotation", 3, 3, kRot) + Matrix("extrinsicTranslation", 3, 1, kTrans) + extra;
}

}  // namespace

TEST(ExtrinsicOverride, ReadsValidFileWithTd)
{
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  ASSERT_TRUE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_ok.yaml", Body("td: 0.0126\n")), &ex, &err)) << err;
  EXPECT_DOUBLE_EQ(ex.R(0, 1), 0.5735764);
  EXPECT_DOUBLE_EQ(ex.R(1, 0), -1.0);
  EXPECT_DOUBLE_EQ(ex.t.x(), 0.19);
  EXPECT_DOUBLE_EQ(ex.t.z(), -0.10);
  EXPECT_TRUE(ex.has_td);
  EXPECT_DOUBLE_EQ(ex.td, 0.0126);
  EXPECT_EQ(ex.robot_id, "edie-127b");
  EXPECT_EQ(ex.source, "urdf");
  EXPECT_EQ(ex.calibrated_at, "2026-10-18");
}

TEST(ExtrinsicOverride, TdIsOptional)
{
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  ASSERT_TRUE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_no_td.yaml", Body()), &ex, &err)) << err;
  EXPECT_FALSE(ex.has_td);  // 없으면 vio_edie.yaml 의 td 그대로 둔다
}

TEST(ExtrinsicOverride, LogStringsAreOptional)
{
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  const std::string body =
      "%YAML:1.0\n" + Matrix("extrinsicRotation", 3, 3, kRot) + Matrix("extrinsicTranslation", 3, 1, kTrans);
  ASSERT_TRUE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_bare.yaml", body), &ex, &err)) << err;
  EXPECT_TRUE(ex.robot_id.empty());
}

TEST(ExtrinsicOverride, VioEdieYamlHasTheSameShape)
{
  // 형식이 vio_edie.yaml 과 같다 — 그 파일의 값을 그대로 옮겨 적은 파일이면 같은 값이 나와야 한다.
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  ASSERT_TRUE(vins_calib::ReadImuCamExtrinsic(kVioYaml, &ex, &err)) << err;
  cv::FileStorage fs(kVioYaml, cv::FileStorage::READ);
  cv::Mat R, t;
  fs["extrinsicRotation"] >> R;
  fs["extrinsicTranslation"] >> t;
  Eigen::Matrix3d eR;
  Eigen::Vector3d et;
  cv::cv2eigen(R, eR);
  cv::cv2eigen(t, et);
  EXPECT_TRUE(ex.R.isApprox(eR));
  EXPECT_TRUE(ex.t.isApprox(et));
  ASSERT_TRUE(ex.has_td);
  EXPECT_DOUBLE_EQ(ex.td, static_cast<double>(fs["td"]));
}

TEST(ExtrinsicOverride, MissingKeysRejectWholeFile)
{
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(
      WriteTemp("ex_no_t.yaml", "%YAML:1.0\n" + Matrix("extrinsicRotation", 3, 3, kRot)), &ex, &err));
  EXPECT_NE(err.find("extrinsicTranslation"), std::string::npos) << err;
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(
      WriteTemp("ex_no_r.yaml", "%YAML:1.0\n" + Matrix("extrinsicTranslation", 3, 1, kTrans)), &ex, &err));
  EXPECT_NE(err.find("extrinsicRotation"), std::string::npos) << err;
}

TEST(ExtrinsicOverride, WrongSizeIsRejected)
{
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  const std::string bad_t = "%YAML:1.0\n" + Matrix("extrinsicRotation", 3, 3, kRot) +
                            Matrix("extrinsicTranslation", 1, 3, kTrans);
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_t13.yaml", bad_t), &ex, &err));
  EXPECT_NE(err.find("3x1"), std::string::npos) << err;
  const std::string bad_r = "%YAML:1.0\n" + Matrix("extrinsicRotation", 2, 2, "1, 0, 0, 1") +
                            Matrix("extrinsicTranslation", 3, 1, kTrans);
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_r22.yaml", bad_r), &ex, &err));
  EXPECT_NE(err.find("3x3"), std::string::npos) << err;
}

TEST(ExtrinsicOverride, NonOrthonormalRotationIsRejected)
{
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  // 열 하나를 2 배 — 크기가 1 이 아님
  const std::string scaled = "%YAML:1.0\n" + Matrix("extrinsicRotation", 3, 3, "2, 0, 0, 0, 1, 0, 0, 0, 1") +
                             Matrix("extrinsicTranslation", 3, 1, kTrans);
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_scaled.yaml", scaled), &ex, &err));
  EXPECT_NE(err.find("not a rotation"), std::string::npos) << err;
  // 거울(det = -1) — 직교지만 회전이 아님(축 하나 부호를 잘못 옮겨 적은 경우)
  const std::string mirror = "%YAML:1.0\n" + Matrix("extrinsicRotation", 3, 3, "-1, 0, 0, 0, 1, 0, 0, 0, 1") +
                             Matrix("extrinsicTranslation", 3, 1, kTrans);
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_mirror.yaml", mirror), &ex, &err));
  // 소수 4 자리 반올림 정도(1e-4)는 통과
  const std::string rounded = "%YAML:1.0\n" +
                              Matrix("extrinsicRotation", 3, 3, "0.0, 0.5736, 0.8192, -1.0, 0.0, 0.0, 0.0, -0.8192, 0.5736") +
                              Matrix("extrinsicTranslation", 3, 1, kTrans);
  EXPECT_TRUE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_rounded.yaml", rounded), &ex, &err)) << err;
}

TEST(ExtrinsicOverride, NonFiniteAndBadTdAreRejected)
{
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  const std::string nan_t = "%YAML:1.0\n" + Matrix("extrinsicRotation", 3, 3, kRot) +
                            Matrix("extrinsicTranslation", 3, 1, "0.1, .nan, 0.0");
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_nan.yaml", nan_t), &ex, &err));
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(WriteTemp("ex_td_str.yaml", Body("td: \"abc\"\n")), &ex, &err));
  EXPECT_NE(err.find("td"), std::string::npos) << err;
}

TEST(ExtrinsicOverride, MissingOrBrokenFileIsRejected)
{
  vins_calib::ImuCamExtrinsic ex;
  std::string err;
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(::testing::TempDir() + "no_such_extrinsic.yaml", &ex, &err));
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(vins_calib::ReadImuCamExtrinsic(
      WriteTemp("ex_broken.yaml", "%YAML:1.0\nextrinsicRotation: [1, 2\n"), &ex, &err));
  EXPECT_FALSE(err.empty());
}
