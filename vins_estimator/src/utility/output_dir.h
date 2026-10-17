#pragma once
// [SW1-1959] VINS 기록 폴더(외부값 extrinsic_parameter.txt·궤적 CSV) 정하기 — 순수 STL+POSIX 헤더(시험에서 단독 사용).
//
// 왜: 예전엔 vio_edie.yaml 의 output_path 가 "/tmp/vins_edie_output" 로 박혀 있었다. edieOS(Yocto, root) 에는 그 폴더가
//     없어서 외부값 파일 쓰기가 발행 프레임마다 실패하고 OpenCV ERROR 가 초당 여러 번 찍혔다(2026-10-18 .127 실측).
//     개발 PC·Ubuntu 로봇에선 누군가 폴더를 만들어 둬서 안 보였다.
// 규칙(imu-ros2-wrapper offset_record.hpp·edie_vision state_path.hpp 와 같은 데이터 폴더 규칙, 2026-10-17 사용자 결정):
//   ① 환경변수 EDIE_VINS_OUTPUT_DIR 가 있으면 그대로         (명시)
//   ② yaml output_path 가 비어 있지 않으면 그대로             (명시 — 예전 설정·bag 재생 도구 호환)
//   ③ root 로 돌고 /var/lib/edie 를 만들거나 쓸 수 있으면 /var/lib/edie/data/logs/vins   (edieOS)
//   ④ 그 밖엔 $HOME/.edie/logs/vins                            (Ubuntu edie 계정)
//   ⑤ HOME 도 없으면 ""(기록 안 함)
//   홈 "이름" 이 아니라 uid 로 가르는 이유: 서비스로 켜지면 HOME 이 비어 있을 수 있고, root 인데 /home/root 에 쓰면
//   로봇 데이터가 홈에 흩어진다.
// C++14 패키지라 <filesystem> 대신 stat·mkdir·access 를 쓴다.

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <string>

namespace vins_output_dir
{

constexpr const char * kDirEnv = "EDIE_VINS_OUTPUT_DIR";
constexpr const char * kSystemRoot = "/var/lib/edie";
constexpr const char * kSystemSubdir = "data/logs/vins";
constexpr const char * kHomeSubdir = ".edie/logs/vins";
// 외부값 파일은 추정이 돌 때마다(발행 프레임마다) 다시 쓰였다. 값은 천천히만 바뀌므로 이 간격으로만 쓴다(eMMC 마모 줄이기).
constexpr double kExtrinsicWritePeriodS = 10.0;

struct Resolved
{
  std::string dir;           // 기록 폴더(비어 있으면 기록 안 함)
  bool explicit_dir = false; // ①·② 처럼 사람이 직접 정했는가 — 궤적 CSV 는 이때만 쓴다(아래 설명)
};

inline bool IsWritableDir(const std::string & path)
{
  struct stat st {};
  return !path.empty() && ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode) &&
         ::access(path.c_str(), W_OK) == 0;
}

// mkdir -p. 이미 있으면 그대로. 끝에 폴더가 있으면 true(쓰기 가능 여부는 IsWritableDir 로 따로 본다).
inline bool MakeDirs(const std::string & path)
{
  if (path.empty()) {
    return false;
  }
  std::string cur;
  std::size_t pos = 0;
  while (pos <= path.size()) {
    const std::size_t next = path.find('/', pos);
    const std::size_t end = (next == std::string::npos) ? path.size() : next;
    cur = path.substr(0, end);
    if (!cur.empty()) {
      if (::mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
      }
    }
    if (next == std::string::npos) {
      break;
    }
    pos = next + 1;
  }
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

// 시험용으로 입력을 다 받는 판. system_root 는 root 일 때만 만든다(부모까지).
inline Resolved ResolveFrom(
  const char * env, const std::string & yaml_path, bool run_as_root, const char * home,
  const std::string & system_root)
{
  Resolved r;
  if (env != nullptr && env[0] != '\0') {
    r.dir = env;
    r.explicit_dir = true;
    return r;
  }
  if (!yaml_path.empty()) {
    r.dir = yaml_path;
    r.explicit_dir = true;
    return r;
  }
  if (run_as_root) {
    MakeDirs(system_root);
    // 폴더이고 쓸 수 있을 때만(만들기 실패·읽기 전용이면 아래 홈으로)
    if (IsWritableDir(system_root)) {
      r.dir = system_root + "/" + kSystemSubdir;
      return r;
    }
  }
  if (home != nullptr && home[0] != '\0') {
    r.dir = std::string(home) + "/" + kHomeSubdir;
  }
  return r;
}

// 실제 환경(환경변수·uid·HOME·/var/lib/edie)으로 위 규칙을 적용한다.
inline Resolved Resolve(const std::string & yaml_path)
{
  return ResolveFrom(std::getenv(kDirEnv), yaml_path, ::geteuid() == 0, std::getenv("HOME"), kSystemRoot);
}

// 폴더를 만들고 쓸 수 있으면 true. 못 쓰면 호출자는 경로를 비워 기록을 끈다(프레임마다 오류를 찍지 않게).
inline bool Prepare(const std::string & dir)
{
  if (dir.empty()) {
    return false;
  }
  MakeDirs(dir);
  return IsWritableDir(dir);
}

// "마지막으로 쓴 뒤 period 초가 지났는가" — 처음 한 번은 바로 쓴다.
class WriteThrottle
{
public:
  explicit WriteThrottle(double period_s) : period_s_(period_s) {}
  bool Due(double now_s)
  {
    if (has_last_ && now_s - last_s_ < period_s_) {
      return false;
    }
    has_last_ = true;
    last_s_ = now_s;
    return true;
  }

private:
  double period_s_;
  double last_s_ = 0.0;
  bool has_last_ = false;
};

}  // namespace vins_output_dir
