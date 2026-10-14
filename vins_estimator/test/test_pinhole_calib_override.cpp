// [SW1-1962] VINS 가 edie_vision 캘리브(정류 P)를 직접 읽어 카메라 모델을 만든다.
// ① 캘리브 읽기(정상·실패) ② 덮어쓴 카메라의 모양(이름·해상도 유지, 왜곡 0)
// ③ 같은 값이면 vio_edie.yaml 로 만든 카메라와 투영이 똑같다(성능 영향 없음 확인).
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <iomanip>
#include <sstream>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "utility/pinhole_calib_override.h"

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

const char* kCalibBody =
    "camera_model: \"pinhole\"\n"
    "camera_matrix:\n  rows: 3\n  cols: 3\n"
    "  data: [438.3, 0.0, 310.3, 0.0, 436.8, 262.3, 0.0, 0.0, 1.0]\n"
    "projection_matrix:\n  rows: 3\n  cols: 4\n"
    "  data: [401.5, 0.0, 330.25, 0.0, 0.0, 402.5, 260.75, 0.0, 0.0, 0.0, 1.0, 0.0]\n";

}  // namespace

TEST(PinholeCalibOverride, ReadsProjectionNotCameraMatrix)
{
  vins_calib::RectPinhole k;
  std::string err;
  ASSERT_TRUE(vins_calib::ReadRectifiedPinhole(WriteTemp("calib_ok.yaml", kCalibBody), &k, &err)) << err;
  // 정류 영상이라 camera_matrix(K, 438.3) 가 아니라 projection_matrix(P) 값이어야 한다.
  EXPECT_DOUBLE_EQ(k.fx, 401.5);
  EXPECT_DOUBLE_EQ(k.fy, 402.5);
  EXPECT_DOUBLE_EQ(k.cx, 330.25);
  EXPECT_DOUBLE_EQ(k.cy, 260.75);
}

TEST(PinholeCalibOverride, RejectsBadFiles)
{
  vins_calib::RectPinhole k;
  std::string err;
  EXPECT_FALSE(vins_calib::ReadRectifiedPinhole(::testing::TempDir() + "no_such.yaml", &k, &err));
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(vins_calib::ReadRectifiedPinhole(
      WriteTemp("calib_short.yaml", "projection_matrix:\n  data: [1.0, 2.0]\n"), &k, &err));
  EXPECT_FALSE(vins_calib::ReadRectifiedPinhole(
      WriteTemp("calib_zero.yaml", "projection_matrix:\n  data: [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0]\n"),
      &k, &err));
  EXPECT_FALSE(vins_calib::ReadRectifiedPinhole(WriteTemp("calib_none.yaml", "foo: 1\n"), &k, &err));
}

TEST(PinholeCalibOverride, KeepsNameAndSizeZeroDistortion)
{
  const camodocal::CameraPtr base = boost::make_shared<camodocal::PinholeCamera>(
      "camera", 640, 480, -0.3, 0.1, 0.001, 0.002, 438.3, 436.8, 310.3, 262.3);
  const camodocal::CameraPtr cam = vins_calib::MakeRectifiedPinhole(base, {401.5, 402.5, 330.25, 260.75});
  ASSERT_TRUE(cam);
  EXPECT_EQ(cam->cameraName(), "camera");
  EXPECT_EQ(cam->imageWidth(), 640);
  EXPECT_EQ(cam->imageHeight(), 480);
  std::vector<double> p;
  cam->writeParameters(p);  // 핀홀: k1 k2 p1 p2 fx fy cx cy
  ASSERT_EQ(p.size(), 8u);
  EXPECT_EQ(p[0], 0.0);
  EXPECT_EQ(p[1], 0.0);
  EXPECT_EQ(p[2], 0.0);
  EXPECT_EQ(p[3], 0.0);
  EXPECT_DOUBLE_EQ(p[4], 401.5);
  EXPECT_DOUBLE_EQ(p[7], 260.75);
}

TEST(PinholeCalibOverride, NonPinholeBaseIsNotReplaced)
{
  std::string err;
  EXPECT_FALSE(vins_calib::MakeRectifiedPinhole(nullptr, {400, 400, 320, 240}, &err));
  // 실제 핀홀이 아닌 카메라(MEI) — 덮어쓰지 않는다(critic R4).
  const camodocal::CameraPtr mei = boost::make_shared<camodocal::CataCamera>(
      "camera", 640, 480, 1.0, 0.0, 0.0, 0.0, 0.0, 400.0, 400.0, 320.0, 240.0);
  EXPECT_FALSE(vins_calib::MakeRectifiedPinhole(mei, {400, 400, 320, 240}, &err));
  EXPECT_NE(err.find("PINHOLE"), std::string::npos);
}

TEST(PinholeCalibOverride, ResolutionMismatchIsNotApplied)
{
  // 캘리브에 적힌 해상도가 설정(640x480)과 다르면 쓰지 않는다(critic R3). 적혀 있지 않으면(0) 검사 생략.
  const camodocal::CameraPtr base = boost::make_shared<camodocal::PinholeCamera>(
      "camera", 640, 480, 0, 0, 0, 0, 400, 400, 320, 240);
  vins_calib::RectPinhole k;
  std::string err;
  ASSERT_TRUE(vins_calib::ReadRectifiedPinhole(
      WriteTemp("calib_720.yaml", std::string("image_width: 1280\nimage_height: 720\n") + kCalibBody), &k, &err));
  EXPECT_EQ(k.width, 1280);
  EXPECT_FALSE(vins_calib::MakeRectifiedPinhole(base, k, &err));
  EXPECT_NE(err.find("1280x720"), std::string::npos);
  ASSERT_TRUE(vins_calib::ReadRectifiedPinhole(
      WriteTemp("calib_480.yaml", std::string("image_width: 640\nimage_height: 480\n") + kCalibBody), &k, &err));
  EXPECT_TRUE(vins_calib::MakeRectifiedPinhole(base, k, &err));
}

TEST(PinholeCalibOverride, SameValuesGiveSameProjectionAsConfig)
{
  // vio_edie.yaml 로 만든 카메라(지금까지 방식)와, 같은 fx·fy·cx·cy 를 캘리브에서 읽어 만든 카메라(C안)가
  // 화면 곳곳의 점을 똑같이 3D 방향으로 바꾸는지 — 값이 같으면 VINS 결과도 같다.
  const camodocal::CameraPtr base = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(kVioYaml);
  ASSERT_TRUE(base);
  std::vector<double> p;
  base->writeParameters(p);
  ASSERT_EQ(p.size(), 8u);
  std::ostringstream body;
  body << std::setprecision(17) << "projection_matrix:\n  data: [" << p[4] << ", 0, " << p[6]
       << ", 0, 0, " << p[5] << ", " << p[7] << ", 0, 0, 0, 1, 0]\n";
  const std::string calib = WriteTemp("calib_same.yaml", body.str());
  vins_calib::RectPinhole k;
  std::string err;
  ASSERT_TRUE(vins_calib::ReadRectifiedPinhole(calib, &k, &err)) << err;
  const camodocal::CameraPtr cam = vins_calib::MakeRectifiedPinhole(base, k);
  ASSERT_TRUE(cam);
  for (double u : {0.0, 100.0, 339.0, 500.0, 639.0})
  {
    for (double v : {0.0, 240.0, 479.0})
    {
      Eigen::Vector3d a, b;
      base->liftProjective(Eigen::Vector2d(u, v), a);
      cam->liftProjective(Eigen::Vector2d(u, v), b);
      EXPECT_NEAR((a - b).norm(), 0.0, 1e-5) << "u=" << u << " v=" << v;
    }
  }
}
