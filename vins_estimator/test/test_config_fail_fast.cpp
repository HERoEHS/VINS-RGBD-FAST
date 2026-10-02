// [SW1-1948] 설정 파일을 못 열면 readParameters 가 즉시 예외를 던지는지 — 회귀 테스트.
//   배경: 2026-10-02 설치본만 있는 로봇(.127)에서 launch 기본 경로가 없는 파일을 가리켰는데, readParameters 가
//   경고만 찍고 계속 가서 모든 값이 0 으로 읽히다 extrinsicRotation 을 빈 행렬로 cv2eigen 에 넣는 순간
//   OpenCV assert 로 abort(exit -6) 했다. 지금은 "설정 파일을 열 수 없음" 메시지와 함께 std::runtime_error.
#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "../src/utility/parameters.h"

namespace
{

// 테스트용 노드 — config_file/vins_folder 파라미터를 생성자 override 로 꽂는다(readParam 의 declare 가 이 값을 받는다).
std::shared_ptr<rclcpp::Node> makeNode(const std::string & config_file)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("config_file", config_file),
    rclcpp::Parameter("vins_folder", "/nonexistent-vins-root/")});
  return std::make_shared<rclcpp::Node>("vins_param_fail_fast_test", opts);
}

}  // namespace

TEST(ConfigFailFast, ThrowsWhenConfigFileMissing)
{
  auto node = makeNode("/nonexistent-vins-root/config/elp_stereo_edie/vio_edie.yaml");
  EXPECT_THROW(readParameters(node.get()), std::runtime_error);
}

TEST(ConfigFailFast, MessageNamesTheMissingPath)
{
  const std::string path = "/nonexistent-vins-root/config/elp_stereo_edie/vio_edie.yaml";
  auto node = makeNode(path);
  try {
    readParameters(node.get());
    FAIL() << "예외가 나야 한다";
  } catch (const std::runtime_error & e) {
    // 운영자가 로그만 보고 어느 경로가 틀렸는지 알 수 있어야 한다.
    EXPECT_NE(std::string(e.what()).find(path), std::string::npos) << e.what();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
