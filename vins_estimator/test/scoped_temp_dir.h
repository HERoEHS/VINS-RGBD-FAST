#pragma once
// [SW1-1959] 시험 파일을 /tmp 바로 아래가 아니라 시험 실행마다 새로 만든 폴더에 쓰고, 끝나면 폴더째 지운다.
// 예전엔 ::testing::TempDir() + 이름 으로 써서 /tmp 에 robot_calib_720.yaml 같은 파일이 계속 남았다(critic B2 R6).
// C++14(이 패키지 표준)라 std::filesystem 대신 mkdtemp·nftw 를 쓴다.

#include <ftw.h>
#include <stdlib.h>

#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace vins_test
{

class ScopedTempDir
{
public:
  ScopedTempDir()
  {
    std::string tmpl = ::testing::TempDir() + "vins_test_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (!mkdtemp(buf.data()))
      throw std::runtime_error("mkdtemp failed: " + tmpl);
    path_ = std::string(buf.data()) + "/";
  }
  ~ScopedTempDir()
  {
    // 안쪽부터(FTW_DEPTH) 파일·폴더를 지운다. 링크는 따라가지 않는다(FTW_PHYS).
    nftw(path_.c_str(), [](const char* p, const struct stat*, int, struct FTW*) { return std::remove(p); }, 16,
         FTW_DEPTH | FTW_PHYS);
  }
  ScopedTempDir(const ScopedTempDir&) = delete;
  ScopedTempDir& operator=(const ScopedTempDir&) = delete;

  // 끝에 '/' 가 붙은 폴더 경로
  const std::string& path() const { return path_; }

private:
  std::string path_;
};

// 시험 프로그램 하나에 폴더 하나 — 프로그램이 끝날 때(정적 객체 소멸) 지워진다.
inline const std::string& TestTempDir()
{
  static ScopedTempDir dir;
  return dir.path();
}

}  // namespace vins_test
