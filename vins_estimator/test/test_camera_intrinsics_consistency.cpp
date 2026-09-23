// [SW1-1828] vio_edie.yaml 카메라 모델 ↔ 실제 입력 영상 일치 회귀 테스트
//
// 배경: image_gray 는 edie_vision SbsUndistorter 가 이미 펴서(정류) 발행한다.
//   그런데 06-18 부터 vio_edie.yaml 에는 '정류 전 원본' K+D(fx 438, k1 −0.37)가 들어 있어
//   VINS 가 이미 편 영상을 한 번 더 폈다(이중 보정, 모서리 방향 오차 7~10°).
//   이 테스트는 같은 실수가 되풀이되지 않도록 두 가지를 확인한다.
//   ① 불변식: 정류 영상을 받으므로 왜곡 계수는 0, fx == fy (정류 P 의 성질)
//   ② 교차 검사: edie_vision 캘리브 파일이 보이면 use_undistort 에 맞는 쪽(P 또는 K+D)과 값이 같은지
//      (edie_vision 이 없는 작업공간에서는 건너뜀)
#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <cstdlib>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace
{

const std::string kVinsRoot = VINS_SOURCE_ROOT;  // CMake 가 주입 (VINS-RGBD-FAST 루트)
const std::string kVioYaml = kVinsRoot + "/config/elp_stereo_edie/vio_edie.yaml";
// edie9 저장소 배치: edie9/edie_localization/VINS-RGBD-FAST ↔ edie9/edie_vision/edie_vision
// task 작업공간처럼 edie_vision 이 옆에 없을 때는 EDIE_VISION_CFG_DIR 로 경로를 지정할 수 있다.
const std::string kVisionCfg = [] {
  const char* env = std::getenv("EDIE_VISION_CFG_DIR");
  return env ? std::string(env) + "/"
             : kVinsRoot + "/../../edie_vision/edie_vision/config/gstreamer/";
}();

struct Intrinsics
{
  double fx, fy, cx, cy, k1, k2, p1, p2;
};

Intrinsics ReadVio()
{
  cv::FileStorage fs(kVioYaml, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("vio_edie.yaml 열기 실패: " + kVioYaml);
  }
  cv::FileNode d = fs["distortion_parameters"];
  cv::FileNode p = fs["projection_parameters"];
  return {p["fx"], p["fy"], p["cx"], p["cy"], d["k1"], d["k2"], d["p1"], d["p2"]};
}

std::string Slurp(const std::string& path)
{
  std::ifstream f(path);
  if (!f) {
    return {};
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// ROS camera_info 형식 yaml 에서 "<key>:" 블록의 첫 번째 (주석 아닌) data 배열을 읽는다.
// 파일에 해상도별 주석 줄("# data: [...]")이 섞여 있어 주석은 건너뛴다.
std::vector<double> ReadDataBlock(const std::string& text, const std::string& key)
{
  const auto pos = text.find(key + ":");
  if (pos == std::string::npos) {
    return {};
  }
  std::istringstream in(text.substr(pos));
  std::string line;
  static const std::regex kData(R"(^\s*data:\s*\[([^\]]*)\])");
  std::smatch m;
  while (std::getline(in, line)) {
    if (std::regex_search(line, m, kData)) {
      std::vector<double> v;
      std::stringstream vs(m[1].str());
      std::string tok;
      while (std::getline(vs, tok, ',')) {
        v.push_back(std::stod(tok));
      }
      return v;
    }
  }
  return {};
}

// camera_v4l2.yaml 의 left: 블록 use_undistort 값. 못 찾으면 -1.
int ReadLeftUseUndistort(const std::string& text)
{
  const auto left = text.find("\nleft:");
  if (left == std::string::npos) {
    return -1;
  }
  static const std::regex kUse(R"(use_undistort:\s*(true|false))");
  std::smatch m;
  const std::string tail = text.substr(left);
  if (!std::regex_search(tail, m, kUse)) {
    return -1;
  }
  return m[1].str() == "true" ? 1 : 0;
}

}  // namespace

// ① 불변식: VINS 입력(image_gray)은 정류 영상 → 왜곡 0, 정방 화소(fx == fy)
//   예외: edie_vision 설정이 보이고 use_undistort:false(원본 발행, Kalibr 녹화용)일 때만 건너뛴다.
TEST(CameraIntrinsicsConsistency, RectifiedInputHasNoDistortion)
{
  const std::string v4l2 = Slurp(kVisionCfg + "camera_v4l2.yaml");
  if (!v4l2.empty() && ReadLeftUseUndistort(v4l2) == 0) {
    GTEST_SKIP() << "use_undistort:false — 원본 영상 모드라 K+D 가 정답(교차 검사에서 확인)";
  }
  const Intrinsics v = ReadVio();
  EXPECT_DOUBLE_EQ(v.k1, 0.0);
  EXPECT_DOUBLE_EQ(v.k2, 0.0);
  EXPECT_DOUBLE_EQ(v.p1, 0.0);
  EXPECT_DOUBLE_EQ(v.p2, 0.0);
  EXPECT_NEAR(v.fx, v.fy, 1e-6) << "정류 P 는 fx == fy 다. 다르면 정류 전 K 를 넣었을 가능성이 크다";
}

// ② 교차 검사: edie_vision 캘리브와 값 일치
TEST(CameraIntrinsicsConsistency, MatchesEdieVisionCalibration)
{
  const std::string calib = Slurp(kVisionCfg + "pinhole_left_calib.yaml");
  const std::string v4l2 = Slurp(kVisionCfg + "camera_v4l2.yaml");
  if (calib.empty() || v4l2.empty()) {
    GTEST_SKIP() << "edie_vision 설정 파일 없음(단독 작업공간) — 교차 검사 생략: " << kVisionCfg;
  }
  const int use_undistort = ReadLeftUseUndistort(v4l2);
  ASSERT_NE(use_undistort, -1) << "camera_v4l2.yaml left.use_undistort 를 찾지 못함";

  const Intrinsics v = ReadVio();
  constexpr double kTol = 1e-3;
  if (use_undistort == 1) {
    // 정류 영상 → projection_matrix P (3x4): [fx 0 cx Tx; 0 fy cy 0; 0 0 1 0]
    const auto P = ReadDataBlock(calib, "projection_matrix");
    ASSERT_EQ(P.size(), 12u);
    EXPECT_NEAR(v.fx, P[0], kTol);
    EXPECT_NEAR(v.fy, P[5], kTol);
    EXPECT_NEAR(v.cx, P[2], kTol);
    EXPECT_NEAR(v.cy, P[6], kTol);
  } else {
    // 원본 영상 → camera_matrix K + distortion D
    const auto K = ReadDataBlock(calib, "camera_matrix");
    const auto D = ReadDataBlock(calib, "distortion_coefficients");
    ASSERT_EQ(K.size(), 9u);
    ASSERT_GE(D.size(), 4u);
    EXPECT_NEAR(v.fx, K[0], kTol);
    EXPECT_NEAR(v.fy, K[4], kTol);
    EXPECT_NEAR(v.cx, K[2], kTol);
    EXPECT_NEAR(v.cy, K[5], kTol);
    EXPECT_NEAR(v.k1, D[0], kTol);
    EXPECT_NEAR(v.k2, D[1], kTol);
    EXPECT_NEAR(v.p1, D[2], kTol);
    EXPECT_NEAR(v.p2, D[3], kTol);
  }
}
