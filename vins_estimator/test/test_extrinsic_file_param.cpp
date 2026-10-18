// [SW1-1959] extrinsic_file·camera_calib_from_robot 파라미터 연결부 — readParameters 가 vio_edie.yaml 을 다 읽은 뒤
// 로봇 외부값 파일로 RIC·TIC(·TD)를 덮는지, 비었거나 틀린 파일이면 vio_edie.yaml 값 그대로인지.
// 로봇 폴더 파일이 틀렸으면 죽지 않고 기본값으로 가되 로그는 ERROR "robot ... invalid" 여야 한다(로그 가로채 확인).
#include <gtest/gtest.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include "scoped_temp_dir.h"
#include "../src/feature_tracker/feature_tracker.h"
#include "../src/utility/parameters.h"

namespace
{

const std::string kVinsRoot = VINS_SOURCE_ROOT;  // CMake 가 주입 (VINS-RGBD-FAST 루트)
const std::string kVioYaml = kVinsRoot + "/config/elp_stereo_edie/vio_edie.yaml";

std::string WriteTemp(const std::string& name, const std::string& body)
{
  const std::string path = vins_test::TestTempDir() + name;
  std::ofstream(path) << body;
  return path;
}

// vio_edie.yaml 과 확실히 다른 값(축 순서만 바꾼 회전).
const char* kRobotExtrinsic =
    "%YAML:1.0\n"
    "robot_id: \"edie-test\"\nsource: \"urdf\"\ncalibrated_at: \"2026-10-18\"\n"
    "extrinsicRotation: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
    "   data: [0, 0, 1, -1, 0, 0, 0, -1, 0]\n"
    "extrinsicTranslation: !!opencv-matrix\n   rows: 3\n   cols: 1\n   dt: d\n"
    "   data: [0.1, 0.2, 0.3]\n";

// ---- 로그 가로채기: (심각도, 글) 목록 ----
std::mutex g_log_mutex;
std::vector<std::pair<int, std::string>> g_logs;

void CaptureLog(const rcutils_log_location_t*, int severity, const char*, rcutils_time_point_value_t,
                const char* format, va_list* args)
{
  char buf[4096];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  std::lock_guard<std::mutex> lock(g_log_mutex);
  g_logs.emplace_back(severity, buf);
}

// needle 을 담은 로그의 심각도(없으면 -1).
int SeverityOf(const std::string& needle)
{
  std::lock_guard<std::mutex> lock(g_log_mutex);
  for (const auto& l : g_logs)
  {
    if (l.second.find(needle) != std::string::npos)
      return l.first;
  }
  return -1;
}

// readParameters 는 전역에 쌓는다(RIC.push_back) — 시험마다 비우고 다시 읽는다.
void ReadWith(const std::string& extrinsic_file, const std::string& camera_calib_file = "",
              bool camera_calib_from_robot = false, const std::string& config_file = kVioYaml)
{
  RIC.clear();
  TIC.clear();
  {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_logs.clear();
  }
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("config_file", config_file),
    rclcpp::Parameter("vins_folder", kVinsRoot + "/"),
    rclcpp::Parameter("extrinsic_file", extrinsic_file),
    rclcpp::Parameter("camera_calib_file", camera_calib_file),
    rclcpp::Parameter("camera_calib_from_robot", camera_calib_from_robot)});
  auto node = std::make_shared<rclcpp::Node>("vins_extrinsic_file_test", opts);
  readParameters(node.get());
}

struct Config
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double td;
};

Config FromVioYaml()
{
  cv::FileStorage fs(kVioYaml, cv::FileStorage::READ);
  cv::Mat R, t;
  fs["extrinsicRotation"] >> R;
  fs["extrinsicTranslation"] >> t;
  Config c;
  cv::cv2eigen(R, c.R);
  cv::cv2eigen(t, c.t);
  c.td = fs["td"];
  return c;
}

}  // namespace

TEST(ExtrinsicFileParam, EmptyKeepsConfigValues)
{
  const Config c = FromVioYaml();
  ReadWith("");
  ASSERT_EQ(RIC.size(), 1u);
  EXPECT_TRUE(RIC[0].isApprox(c.R, 1e-6));
  EXPECT_TRUE(TIC[0].isApprox(c.t));
  EXPECT_DOUBLE_EQ(TD, c.td);
}

TEST(ExtrinsicFileParam, RobotFileOverridesRicTicAndTd)
{
  ReadWith(WriteTemp("robot_extrinsic_td.yaml", std::string(kRobotExtrinsic) + "td: 0.02\n"));
  ASSERT_EQ(RIC.size(), 1u);
  Eigen::Matrix3d expect;
  expect << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  EXPECT_TRUE(RIC[0].isApprox(expect, 1e-9));
  EXPECT_TRUE(Ric.isApprox(expect, 1e-9));
  EXPECT_TRUE(TIC[0].isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_DOUBLE_EQ(TD, 0.02);
}

TEST(ExtrinsicFileParam, RobotFileWithoutTdKeepsConfigTd)
{
  const Config c = FromVioYaml();
  ReadWith(WriteTemp("robot_extrinsic_no_td.yaml", kRobotExtrinsic));
  EXPECT_TRUE(TIC[0].isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_DOUBLE_EQ(TD, c.td);
}

TEST(ExtrinsicFileParam, BadOrMissingFileKeepsConfigValues)
{
  const Config c = FromVioYaml();
  ReadWith(vins_test::TestTempDir() + "no_such_robot_extrinsic.yaml");
  EXPECT_TRUE(RIC[0].isApprox(c.R, 1e-6));
  EXPECT_TRUE(TIC[0].isApprox(c.t));
  // 회전은 맞는데 이동이 1x3 — 반쪽만 섞지 않고 통째로 버린다.
  ReadWith(WriteTemp("robot_extrinsic_bad.yaml",
                     "%YAML:1.0\n"
                     "extrinsicRotation: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
                     "   data: [0, 0, 1, -1, 0, 0, 0, -1, 0]\n"
                     "extrinsicTranslation: !!opencv-matrix\n   rows: 1\n   cols: 3\n   dt: d\n"
                     "   data: [0.1, 0.2, 0.3]\n"
                     "td: 0.5\n"));
  EXPECT_TRUE(RIC[0].isApprox(c.R, 1e-6));
  EXPECT_TRUE(TIC[0].isApprox(c.t));
  EXPECT_DOUBLE_EQ(TD, c.td);
}

TEST(ExtrinsicFileParam, InvalidRobotExtrinsicIsErrorNamingFile)
{
  // 로봇 폴더 외부값이 틀렸으면 죽지 않고 vio_edie 값으로 — 로그는 WARN 이 아니라 ERROR, 그 파일 경로와 이유를 담는다.
  const Config c = FromVioYaml();
  const std::string bad = WriteTemp("robot_extrinsic_mirror.yaml",
                                    "%YAML:1.0\n"
                                    "extrinsicRotation: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
                                    "   data: [-1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
                                    "extrinsicTranslation: !!opencv-matrix\n   rows: 3\n   cols: 1\n   dt: d\n"
                                    "   data: [0.1, 0.2, 0.3]\n");
  ASSERT_NO_THROW(ReadWith(bad));
  EXPECT_TRUE(RIC[0].isApprox(c.R, 1e-6));
  EXPECT_TRUE(TIC[0].isApprox(c.t));
  EXPECT_EQ(SeverityOf("robot extrinsic invalid (" + bad + ": "), RCUTILS_LOG_SEVERITY_ERROR);
  EXPECT_NE(SeverityOf("- using config_file values"), -1);
}

TEST(ExtrinsicFileParam, InvalidRobotCameraCalibIsErrorNamingFile)
{
  const std::string bad = WriteTemp("robot_pinhole_left_calib.yaml", "projection_matrix:\n  data: [1.0, 2.0]\n");
  ASSERT_NO_THROW(ReadWith("", bad, true));
  EXPECT_FALSE(CAMERA_CALIB_OK);  // 카메라는 vio_edie.yaml 값
  EXPECT_EQ(SeverityOf("robot calib invalid (" + bad + ": "), RCUTILS_LOG_SEVERITY_ERROR);
}

TEST(ExtrinsicFileParam, InvalidImageDefaultCameraCalibStaysWarn)
{
  // 이미지 기본 파일(로봇 폴더 아님)이 틀린 것은 지금처럼 WARN — 로봇 캘리브가 망가졌다고 말하지 않는다.
  const std::string bad = WriteTemp("image_pinhole_left_calib.yaml", "projection_matrix:\n  data: [1.0, 2.0]\n");
  ReadWith("", bad, false);
  EXPECT_EQ(SeverityOf("camera calib not applied (" + bad), RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(SeverityOf("robot calib invalid"), -1);
}

TEST(ExtrinsicFileParam, RobotCalibRejectedByTrackerIsErrorNamingFile)
{
  // 읽기는 됐지만 해상도가 설정(640x480)과 달라 카메라 모델로 못 쓰는 로봇 파일 — feature_tracker 에서 ERROR.
  const std::string calib = WriteTemp(
      "robot_calib_720.yaml",
      "image_width: 1280\nimage_height: 720\n"
      "camera_matrix:\n  data: [438.3, 0, 310.3, 0, 436.8, 262.3, 0, 0, 1]\n"
      "distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: [0, 0, 0, 0, 0]\n"
      "rectification_matrix:\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
      "projection_matrix:\n  data: [401.5, 0, 330.25, 0, 0, 402.5, 260.75, 0, 0, 0, 1, 0]\n");
  ReadWith("", calib, true);
  ASSERT_TRUE(CAMERA_CALIB_OK);
  FeatureTracker tracker;
  tracker.readIntrinsicParameter(kVioYaml);
  EXPECT_EQ(SeverityOf("robot calib invalid (" + calib + ": calib resolution 1280x720"),
            RCUTILS_LOG_SEVERITY_ERROR);
  ASSERT_TRUE(tracker.m_camera);
  EXPECT_EQ(tracker.m_camera->imageWidth(), 640);  // vio_edie.yaml 카메라 그대로
  CAMERA_CALIB_OK = false;
}

TEST(ExtrinsicFileParam, EstimateExtrinsic2StartsFromRobotValue)
{
  // estimate_extrinsic: 2 는 config 에서 RIC=I·TIC=0 으로 시작한다. 로봇 파일이 있으면 그 값으로 시작하고,
  // 회전은 VINS 가 움직임으로 다시 구한다는 WARN 을 남긴다. 모드(2)는 바꾸지 않는다.
  std::ifstream in(kVioYaml);
  std::string cfg((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const std::string key = "estimate_extrinsic: 1";
  const size_t at = cfg.find(key);
  ASSERT_NE(at, std::string::npos) << "vio_edie.yaml 에 '" << key << "' 가 없다 — 시험을 고쳐야 한다";
  cfg.replace(at, key.size(), "estimate_extrinsic: 2");
  const std::string cfg2 = WriteTemp("vio_edie_est2.yaml", cfg);

  ReadWith("", "", false, cfg2);  // 로봇 파일 없음 → 원래 동작(단위 회전·0 이동)
  ASSERT_EQ(ESTIMATE_EXTRINSIC, 2);
  EXPECT_TRUE(RIC[0].isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(TIC[0].isZero());

  ReadWith(WriteTemp("robot_extrinsic_est2.yaml", kRobotExtrinsic), "", false, cfg2);
  Eigen::Matrix3d expect;
  expect << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  EXPECT_EQ(ESTIMATE_EXTRINSIC, 2);
  EXPECT_TRUE(RIC[0].isApprox(expect, 1e-9));
  EXPECT_TRUE(TIC[0].isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_EQ(SeverityOf("estimate_extrinsic=2: rotation will be re-calibrated"), RCUTILS_LOG_SEVERITY_WARN);
}

TEST(ExtrinsicFileParam, RobotExtrinsicWithoutYamlHeaderIsErrorAndConfigValues)
{
  const Config c = FromVioYaml();
  std::string body(kRobotExtrinsic);
  body = body.substr(std::string("%YAML:1.0\n").size());
  const std::string path = WriteTemp("robot_extrinsic_no_header.yaml", body);
  ASSERT_NO_THROW(ReadWith(path));
  EXPECT_TRUE(RIC[0].isApprox(c.R, 1e-6));
  EXPECT_TRUE(TIC[0].isApprox(c.t));
  EXPECT_EQ(SeverityOf("robot extrinsic invalid (" + path + ": "), RCUTILS_LOG_SEVERITY_ERROR);
}

int main(int argc, char ** argv)
{
  // 기록 폴더를 시험용 임시 폴더로 — 홈(~/.edie/logs/vins)·/tmp 에 파일을 남기지 않게(끝나면 폴더째 지움).
  setenv("EDIE_VINS_OUTPUT_DIR", vins_test::TestTempDir().c_str(), 1);
  rclcpp::init(argc, argv);
  rcutils_logging_set_output_handler(CaptureLog);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
