#pragma once
// [SW1-1959] 교차 검사 시험이 edie_vision 설정 폴더(config/gstreamer)를 찾는 순서 — 못 찾으면 그 시험만 SKIP.
// 예전엔 edie9 배치(edie9/edie_localization/VINS-RGBD-FAST ↔ edie9/edie_vision) 한 곳만 봐서, task 작업공간처럼
// src/VINS-RGBD-FAST 가 따로 있는 배치에서는 아무 말 없이 건너뛰었다(critic A2).
//   ① 환경변수 EDIE_VISION_CFG_DIR
//   ② 소스 트리: <VINS 루트>/../../edie_vision/edie_vision(edie9 배치), <VINS 루트>/../edie_vision/edie_vision(src 형제)
//   ③ ament share/edie_vision (설치된 edie_vision — 소스가 옆에 없을 때)

#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace vins_test
{

// 끝에 '/' 가 붙은 폴더. 하나도 못 찾으면 ② 첫 후보(SKIP 메시지에 쓰임).
inline std::string EdieVisionCfgDir(const std::string& vins_root)
{
  if (const char* env = std::getenv("EDIE_VISION_CFG_DIR"))
    return std::string(env) + "/";
  const std::string rel = "/config/gstreamer/";
  std::vector<std::string> candidates = {
      vins_root + "/../../edie_vision/edie_vision" + rel,
      vins_root + "/../edie_vision/edie_vision" + rel,
  };
  try
  {
    candidates.push_back(ament_index_cpp::get_package_share_directory("edie_vision") + rel);
  }
  catch (const std::exception&)  // PackageNotFoundError — edie_vision 이 설치돼 있지 않음
  {
  }
  for (const auto& dir : candidates)
  {
    if (std::ifstream(dir + "pinhole_left_calib.yaml").good())
      return dir;
  }
  return candidates.front();
}

}  // namespace vins_test
