# SophusPin.cmake — Sophus 판을 1.24.6 으로 못 박는다 (SW1-1959, 2026-10-15)
#
# 왜 필요한가:
#   Sophus 는 헤더 전용이라 어떤 판을 집어 와도 빌드가 "되는" 것처럼 보인다. 판이 다르면 SE3/SO3 API 가
#   조금씩 달라 조용히 다른 코드가 되거나(2020 판 vs 1.24), Yocto 의 meta-ros sophus(1.22)와 섞여 깨진다.
#   제품(Yocto)은 레시피 SRCREV d0b7315a(= 태그 1.24.6)로 못 박으므로, CMake 도 그 판만 받는다.
#
# 찾는 순서 — Yocto 가 1순위:
#   ① 설치본 SophusConfig.cmake (sysroot·/usr/local·CMAKE_PREFIX_PATH·-DSophus_DIR).
#      find_package 에 판(1.24.6 EXACT)을 주므로 검색 경로의 모든 후보를 보고 맞는 것만 고른다
#      (ros-humble-sophus 1.22 가 앞에 있어도 뒤의 1.24.6 을 가리지 못한다).
#   ② 소스 트리 — 교차 빌드(CMAKE_CROSSCOMPILING)가 아닐 때만. -DSOPHUS_INCLUDE_DIR=<경로>, 기본 ~/Sophus.
#      판은 트리의 SOPHUS_VERSION 파일로 확인한다(1.24 부터 있음, 없으면 "unknown" → 실패).
#   ③ 둘 다 없거나 판이 다르면 FATAL_ERROR + 설치 안내(본 후보의 판·경로를 같이 적는다).
#      -DSOPHUS_ALLOW_VERSION_MISMATCH=ON 이면 판 불일치를 경고로 낮춘다(개발 중 실험용, 제품 빌드 금지).
#
# 결과 변수(호출한 CMakeLists 에서 사용):
#   VINS_SOPHUS_INCLUDE_DIRS  include 경로
#   VINS_SOPHUS_VERSION       찾은 판 문자열
#   VINS_SOPHUS_SOURCE        "installed" 또는 "source-tree"

set(VINS_SOPHUS_REQUIRED_VERSION "1.24.6")
set(VINS_SOPHUS_REQUIRED_COMMIT "d0b7315a")

option(SOPHUS_ALLOW_VERSION_MISMATCH
  "요구 판(${VINS_SOPHUS_REQUIRED_VERSION})과 다른 Sophus 로도 빌드 — 경고만 내고 진행(개발용)" OFF)
set(SOPHUS_INCLUDE_DIR "$ENV{HOME}/Sophus" CACHE PATH
  "Sophus 소스 트리 경로 — 설치본(SophusConfig.cmake)이 없고 교차 빌드가 아닐 때만 쓴다")

set(VINS_SOPHUS_INCLUDE_DIRS "")
set(VINS_SOPHUS_VERSION "")
set(VINS_SOPHUS_SOURCE "")

# 설치 안내 문구(Ubuntu·Yocto 공통) — 여러 곳에서 쓴다
string(CONCAT _sophus_howto
  "  → Ubuntu: git clone -b ${VINS_SOPHUS_REQUIRED_VERSION} https://github.com/strasdat/Sophus ~/Sophus\n"
  "    (다른 위치면 -DSOPHUS_INCLUDE_DIR=<경로>. 설치본으로 두려면 cmake --install; Sophus 가 cmake ≥ 3.24 를 "
  "요구하므로 cmake 3.22 에선 CMakeLists.txt 의 cmake_minimum_required 를 3.22 로 한 줄 낮춰 설치한다)\n"
  "  → Yocto: sophus 레시피(SRC_URI strasdat/Sophus, SRCREV ${VINS_SOPHUS_REQUIRED_COMMIT} = 태그 "
  "${VINS_SOPHUS_REQUIRED_VERSION}, inherit cmake, DEPENDS libeigen, 위 최소 판 패치 포함)를 넣고 VINS 레시피 "
  "DEPENDS 에 sophus 를 추가. 설치 결과 share/sophus/cmake/SophusConfig*.cmake 가 sysroot 에 있어야 하며 "
  "ros-humble-sophus(1.22)가 같이 들어오지 않는지 bitbake -g 로 확인.")

# ── ① 설치본 — 요구 판 EXACT 로 검색 ───────────────────────────────────────────
set(_sophus_inst_ok FALSE)        # 요구 판 설치본을 쓸 수 있음
set(_sophus_inst_seen_version "") # 보긴 했지만 못 쓰는 설치본(판 다름·의존 실패)의 판 — 안내용
set(_sophus_inst_seen_dir "")
set(_sophus_inst_unusable_reason "")
set(_sophus_inst_include "")
find_package(Sophus ${VINS_SOPHUS_REQUIRED_VERSION} EXACT CONFIG QUIET)
if(Sophus_FOUND AND TARGET Sophus::Sophus)
  get_target_property(_sophus_inst_include Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT _sophus_inst_include)
    # 타깃에 include 속성이 없으면 Config 위치(<prefix>/share/sophus/cmake)에서 <prefix>/include 로 추정
    get_filename_component(_sophus_inst_include "${Sophus_DIR}/../../../include" ABSOLUTE)
  endif()
  set(_sophus_inst_ok TRUE)
  set(_sophus_inst_seen_version "${Sophus_VERSION}")
  set(_sophus_inst_seen_dir "${Sophus_DIR}")
else()
  if(Sophus_CONSIDERED_VERSIONS)
    # 검색 경로에서 본 후보(판이 달라 건너뜀) — 첫 번째를 안내에 쓴다
    list(GET Sophus_CONSIDERED_VERSIONS 0 _sophus_inst_seen_version)
    list(GET Sophus_CONSIDERED_CONFIGS 0 _sophus_inst_seen_dir)
    get_filename_component(_sophus_inst_seen_dir "${_sophus_inst_seen_dir}" DIRECTORY)
  endif()
  if(Sophus_NOT_FOUND_MESSAGE)
    # 판은 맞는데 Config 안의 의존(find_dependency(Eigen3 3.4.0))이 실패한 경우 등.
    # 실패한 Config 의 위치는 Sophus_DIR 이다 — 앞에 판이 다른 후보가 있어도 그쪽을 가리키지 않게 덮어쓴다.
    set(_sophus_inst_unusable_reason "${Sophus_NOT_FOUND_MESSAGE}")
    if(Sophus_DIR)
      set(_sophus_inst_seen_dir "${Sophus_DIR}")
      set(_sophus_inst_seen_version "${VINS_SOPHUS_REQUIRED_VERSION}")
    endif()
  endif()
endif()

# ── ② 소스 트리(교차 빌드 제외) ─────────────────────────────────────────────────
set(_sophus_src_version "")
if(NOT CMAKE_CROSSCOMPILING AND EXISTS "${SOPHUS_INCLUDE_DIR}/sophus/se3.hpp")
  if(EXISTS "${SOPHUS_INCLUDE_DIR}/SOPHUS_VERSION")
    file(READ "${SOPHUS_INCLUDE_DIR}/SOPHUS_VERSION" _sophus_src_version)
    string(STRIP "${_sophus_src_version}" _sophus_src_version)
  else()
    set(_sophus_src_version "unknown")    # 1.22 이하 소스엔 SOPHUS_VERSION 파일이 없다
  endif()
endif()

# ── 고르기 ─────────────────────────────────────────────────────────────────────
set(_sophus_mismatch FALSE)
if(_sophus_inst_ok)
  set(VINS_SOPHUS_INCLUDE_DIRS "${_sophus_inst_include}")
  set(VINS_SOPHUS_VERSION "${_sophus_inst_seen_version}")
  set(VINS_SOPHUS_SOURCE "installed")
  set(_sophus_where "${_sophus_inst_seen_dir}")
elseif(_sophus_src_version STREQUAL VINS_SOPHUS_REQUIRED_VERSION)
  if(_sophus_inst_seen_version)
    if(_sophus_inst_unusable_reason)
      set(_sophus_skip_why "의존 실패(${_sophus_inst_unusable_reason})")
    else()
      set(_sophus_skip_why "요구 판이 아님")
    endif()
    message(STATUS "Sophus 설치본 ${_sophus_inst_seen_version} (${_sophus_inst_seen_dir}) 은 쓰지 않음 — ${_sophus_skip_why}")
  endif()
  set(VINS_SOPHUS_INCLUDE_DIRS "${SOPHUS_INCLUDE_DIR}")
  set(VINS_SOPHUS_VERSION "${_sophus_src_version}")
  set(VINS_SOPHUS_SOURCE "source-tree")
  set(_sophus_where "${SOPHUS_INCLUDE_DIR}")
elseif(_sophus_inst_unusable_reason)
  # 판은 맞는데 설치본을 못 쓰는 경우(의존 실패) — 허용 옵션과 무관하게 멈춘다(이유를 그대로 보여 준다)
  message(FATAL_ERROR
    "Sophus 설치본(${_sophus_inst_seen_dir})을 쓸 수 없음: ${_sophus_inst_unusable_reason}\n${_sophus_howto}")
elseif(_sophus_inst_seen_version)
  # 설치본은 있는데 판이 다름(맞는 소스 트리도 없음)
  set(_sophus_mismatch TRUE)
  set(VINS_SOPHUS_VERSION "${_sophus_inst_seen_version}")
  set(VINS_SOPHUS_SOURCE "installed")
  set(_sophus_where "${_sophus_inst_seen_dir}")
  if(SOPHUS_ALLOW_VERSION_MISMATCH)
    # 허용 모드에서만 그 설치본을 다시 읽어 include 경로를 얻는다(판 조건 없이, 그 폴더만)
    find_package(Sophus CONFIG QUIET PATHS "${_sophus_inst_seen_dir}" NO_DEFAULT_PATH)
    if(TARGET Sophus::Sophus)
      get_target_property(VINS_SOPHUS_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
    endif()
    if(NOT VINS_SOPHUS_INCLUDE_DIRS)
      get_filename_component(VINS_SOPHUS_INCLUDE_DIRS "${_sophus_inst_seen_dir}/../../../include" ABSOLUTE)
    endif()
  endif()
elseif(_sophus_src_version)
  # 소스 트리는 있는데 판이 다르거나 모름
  set(_sophus_mismatch TRUE)
  set(VINS_SOPHUS_INCLUDE_DIRS "${SOPHUS_INCLUDE_DIR}")
  set(VINS_SOPHUS_VERSION "${_sophus_src_version}")
  set(VINS_SOPHUS_SOURCE "source-tree")
  set(_sophus_where "${SOPHUS_INCLUDE_DIR}")
else()
  if(CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR
      "Sophus ${VINS_SOPHUS_REQUIRED_VERSION} 을 sysroot 에서 못 찾음"
      "(교차 빌드에선 호스트 홈의 소스 트리를 쓰지 않는다).\n${_sophus_howto}")
  else()
    message(FATAL_ERROR
      "Sophus ${VINS_SOPHUS_REQUIRED_VERSION} 을 못 찾음 — 설치본(SophusConfig.cmake)도, 소스 트리"
      "('${SOPHUS_INCLUDE_DIR}')도 없다.\n${_sophus_howto}")
  endif()
endif()

# ── 판 확인 ────────────────────────────────────────────────────────────────────
if(_sophus_mismatch)
  string(CONCAT _sophus_msg
    "Sophus 판 불일치: 찾은 판 '${VINS_SOPHUS_VERSION}' (${VINS_SOPHUS_SOURCE}: ${_sophus_where}), "
    "요구 판 ${VINS_SOPHUS_REQUIRED_VERSION} (태그 = ${VINS_SOPHUS_REQUIRED_COMMIT}).\n"
    "  → 설치본: 그 판을 지우거나 -DSophus_DIR 로 ${VINS_SOPHUS_REQUIRED_VERSION} 설치본을 가리킨다.\n"
    "  → 소스 트리: (cd '${SOPHUS_INCLUDE_DIR}' && git fetch --tags && git checkout ${VINS_SOPHUS_REQUIRED_VERSION})\n"
    "${_sophus_howto}\n"
    "  (개발 중 실험만: -DSOPHUS_ALLOW_VERSION_MISMATCH=ON 으로 경고로 낮출 수 있다 — 제품 빌드 금지)")
  if(SOPHUS_ALLOW_VERSION_MISMATCH)
    message(WARNING "${_sophus_msg}")
  else()
    message(FATAL_ERROR "${_sophus_msg}")
  endif()
endif()

message(STATUS "Sophus ${VINS_SOPHUS_VERSION} <- ${_sophus_where} (${VINS_SOPHUS_SOURCE}, 요구 ${VINS_SOPHUS_REQUIRED_VERSION})")
