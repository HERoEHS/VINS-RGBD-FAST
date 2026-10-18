// [SW1-1959] extrinsic_file 파라미터 연결부 — readParameters 가 vio_edie.yaml 을 다 읽은 뒤
// 로봇 외부값 파일로 RIC·TIC(·TD)를 덮는지, 비었거나 틀린 파일이면 vio_edie.yaml 값 그대로인지.
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "../src/utility/parameters.h"

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

// vio_edie.yaml 과 확실히 다른 값(축 순서만 바꾼 회전).
const char* kRobotExtrinsic =
    "%YAML:1.0\n"
    "robot_id: \"edie-test\"\nsource: \"urdf\"\ncalibrated_at: \"2026-10-18\"\n"
    "extrinsicRotation: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
    "   data: [0, 0, 1, -1, 0, 0, 0, -1, 0]\n"
    "extrinsicTranslation: !!opencv-matrix\n   rows: 3\n   cols: 1\n   dt: d\n"
    "   data: [0.1, 0.2, 0.3]\n";

// readParameters 는 전역에 쌓는다(RIC.push_back) — 시험마다 비우고 다시 읽는다.
void ReadWith(const std::string& extrinsic_file)
{
  RIC.clear();
  TIC.clear();
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("config_file", kVioYaml),
    rclcpp::Parameter("vins_folder", kVinsRoot + "/"),
    rclcpp::Parameter("extrinsic_file", extrinsic_file)});
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
  ReadWith(::testing::TempDir() + "no_such_robot_extrinsic.yaml");
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

int main(int argc, char ** argv)
{
  // 기록 폴더를 임시 폴더로 — 시험이 홈(~/.edie/logs/vins)에 파일을 남기지 않게.
  setenv("EDIE_VINS_OUTPUT_DIR", ::testing::TempDir().c_str(), 1);
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
