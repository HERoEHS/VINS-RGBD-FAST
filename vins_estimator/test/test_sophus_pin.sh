#!/usr/bin/env bash
# SophusPin.cmake 동작 검증 (SW1-1959)
#   가짜 설치본(SophusConfig*.cmake)·가짜 소스 트리(SOPHUS_VERSION)를 만들어 configure 를 돌리고,
#   판 검사·찾는 순서·교차 빌드 분기·안내 문구를 확인한다. 실제 /usr/local·~/Sophus 는 보지 않도록
#   CMake 의 시스템/환경 검색 경로를 끈다. 컴파일러는 쓰지 않는다(project LANGUAGES NONE).
# 사용: test_sophus_pin.sh [cmake 실행파일] [SophusPin.cmake 경로]
set -u

CMAKE="${1:-cmake}"
MODULE="${2:-$(cd "$(dirname "$0")/.." && pwd)/cmake/SophusPin.cmake}"
[ -f "$MODULE" ] || { echo "모듈 없음: $MODULE"; exit 2; }

T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
REQ="1.24.6"
pass=0; fail=0
# 환경 PATH 검색을 끄면 CMake 가 make 도 못 찾으므로 직접 넘긴다(실행은 안 한다 — configure 만).
MAKE_PROGRAM="$(command -v make || command -v gmake || echo /bin/true)"
# 환경변수로 새어 들 수 있는 힌트는 지운다
unset Sophus_ROOT SOPHUS_ROOT Sophus_DIR

mk_install() {  # $1 접두 경로  $2 판  [$3 변형: depfail(의존 실패) | noinc(include 속성 없음)]
  mkdir -p "$1/share/sophus/cmake" "$1/include/sophus"
  case "${3:-}" in
    depfail)  # 실제 Config 의 find_dependency(Eigen3 3.4.0) 실패를 흉내 낸다
      cat > "$1/share/sophus/cmake/SophusConfig.cmake" <<EOF
set(Sophus_FOUND FALSE)
set(Sophus_NOT_FOUND_MESSAGE "가짜 의존 실패: Eigen3 3.4.0 없음")
EOF
      ;;
    noinc)
      cat > "$1/share/sophus/cmake/SophusConfig.cmake" <<EOF
add_library(Sophus::Sophus INTERFACE IMPORTED)
EOF
      ;;
    *)
      cat > "$1/share/sophus/cmake/SophusConfig.cmake" <<EOF
add_library(Sophus::Sophus INTERFACE IMPORTED)
set_target_properties(Sophus::Sophus PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "$1/include")
EOF
      ;;
  esac
  cat > "$1/share/sophus/cmake/SophusConfigVersion.cmake" <<EOF
set(PACKAGE_VERSION "$2")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
if("\${PACKAGE_FIND_VERSION}" STREQUAL "\${PACKAGE_VERSION}")
  set(PACKAGE_VERSION_EXACT TRUE)
endif()
EOF
}

mk_source() {  # $1 경로  $2 판 또는 none(SOPHUS_VERSION 파일 없음)
  mkdir -p "$1/sophus"
  : > "$1/sophus/se3.hpp"
  [ "$2" = none ] || echo "$2" > "$1/SOPHUS_VERSION"
}

mkdir -p "$T/proj"
cat > "$T/proj/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.16)
project(sophus_pin_probe LANGUAGES NONE)
include("$MODULE")
message(STATUS "RESULT src=\${VINS_SOPHUS_SOURCE} ver=\${VINS_SOPHUS_VERSION} dirs=\${VINS_SOPHUS_INCLUDE_DIRS}")
EOF

# run <이름> <기대 종료: ok|fail> <로그에 있어야 할 문구> [cmake 인자...]
run() {
  local name="$1" expect="$2" needle="$3"; shift 3
  local b="$T/b_$name"; rm -rf "$b"
  "$CMAKE" -S "$T/proj" -B "$b" \
    -DCMAKE_MAKE_PROGRAM="$MAKE_PROGRAM" \
    -DCMAKE_FIND_USE_CMAKE_SYSTEM_PATH=OFF \
    -DCMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH=OFF \
    -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=OFF \
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF \
    -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF \
    -DCMAKE_FIND_USE_PACKAGE_ROOT_PATH=OFF \
    "$@" > "$T/$name.log" 2>&1
  local rc=$?
  local ok=1
  if [ "$expect" = ok ] && [ $rc -ne 0 ]; then ok=0; fi
  if [ "$expect" = fail ] && [ $rc -eq 0 ]; then ok=0; fi
  if [ -n "$needle" ] && ! grep -q -- "$needle" "$T/$name.log"; then ok=0; fi
  if [ $ok -eq 1 ]; then
    pass=$((pass+1)); echo "PASS $name"
  else
    fail=$((fail+1)); echo "FAIL $name (rc=$rc, 기대=$expect, 문구='$needle')"
    sed 's/^/    /' "$T/$name.log" | tail -15
  fi
}

NOWHERE="$T/nowhere"   # 존재하지 않는 소스 트리 경로(기본 ~/Sophus 를 가리지 않게 항상 넘긴다)

# 1. 설치본 1.24.6 → 통과, installed
mk_install "$T/inst_ok" "$REQ"
run inst_ok ok "RESULT src=installed ver=$REQ" \
  -DCMAKE_PREFIX_PATH="$T/inst_ok" -DSOPHUS_INCLUDE_DIR="$NOWHERE"

# 2. 설치본 1.22.0 만 → 판 불일치로 실패
mk_install "$T/inst_old" "1.22.0"
run inst_old_fail fail "판 불일치" \
  -DCMAKE_PREFIX_PATH="$T/inst_old" -DSOPHUS_INCLUDE_DIR="$NOWHERE"

# 3. 설치본 1.22.0 + 허용 옵션 → 경고만 내고 통과
run inst_old_allow ok "판 불일치" \
  -DCMAKE_PREFIX_PATH="$T/inst_old" -DSOPHUS_INCLUDE_DIR="$NOWHERE" -DSOPHUS_ALLOW_VERSION_MISMATCH=ON

# 4. 설치본 1.22.0 + 소스 트리 1.24.6 (교차 아님) → 소스 트리로 통과, 설치본 건너뜀 안내
mk_source "$T/src_ok" "$REQ"
run inst_old_src_ok ok "RESULT src=source-tree ver=$REQ" \
  -DCMAKE_PREFIX_PATH="$T/inst_old" -DSOPHUS_INCLUDE_DIR="$T/src_ok"
grep -q "쓰지 않음" "$T/inst_old_src_ok.log" || { fail=$((fail+1)); echo "FAIL inst_old_src_ok: 설치본을 쓰지 않는다는 안내 없음"; }

# 5. 설치본 없음 + 소스 트리 1.24.6 → 통과, source-tree (.219 같은 Ubuntu 로봇)
run src_only_ok ok "RESULT src=source-tree ver=$REQ" \
  -DCMAKE_PREFIX_PATH="$NOWHERE" -DSOPHUS_INCLUDE_DIR="$T/src_ok"

# 6. 소스 트리에 SOPHUS_VERSION 파일 없음(1.22 이하) → unknown 으로 실패
mk_source "$T/src_nover" none
run src_nover_fail fail "unknown" \
  -DCMAKE_PREFIX_PATH="$NOWHERE" -DSOPHUS_INCLUDE_DIR="$T/src_nover"

# 7. 소스 트리 1.20.0 → 판 불일치로 실패
mk_source "$T/src_old" "1.20.0"
run src_old_fail fail "판 불일치" \
  -DCMAKE_PREFIX_PATH="$NOWHERE" -DSOPHUS_INCLUDE_DIR="$T/src_old"

# 8. 교차 빌드 + 소스 트리만 → 소스 트리를 쓰지 않고 실패, Yocto 레시피 안내
run cross_src_fail fail "교차 빌드" \
  -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
  -DCMAKE_PREFIX_PATH="$NOWHERE" -DSOPHUS_INCLUDE_DIR="$T/src_ok"

# 9. 교차 빌드 + sysroot 설치본 1.24.6 → 통과 (Yocto 정상 경로)
run cross_inst_ok ok "RESULT src=installed ver=$REQ" \
  -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
  -DCMAKE_PREFIX_PATH="$T/inst_ok" -DSOPHUS_INCLUDE_DIR="$T/src_ok"

# 10. 아무것도 없음 → 실패 + 설치 안내(clone 명령)
run nothing_fail fail "git clone" \
  -DCMAKE_PREFIX_PATH="$NOWHERE" -DSOPHUS_INCLUDE_DIR="$NOWHERE"

# 11. 검색 순서에서 1.22.0 이 앞, 1.24.6 이 뒤 → 앞 것에 가리지 않고 1.24.6 설치본을 고른다
run inst_old_then_ok ok "RESULT src=installed ver=$REQ dirs=$T/inst_ok/include" \
  -DCMAKE_PREFIX_PATH="$T/inst_old;$T/inst_ok" -DSOPHUS_INCLUDE_DIR="$NOWHERE"

# 12. 판은 맞는데 Config 안 의존이 실패 → 그 이유를 보여 주며 실패(허용 옵션과 무관)
mk_install "$T/inst_depfail" "$REQ" depfail
run inst_depfail_fail fail "가짜 의존 실패" \
  -DCMAKE_PREFIX_PATH="$T/inst_depfail" -DSOPHUS_INCLUDE_DIR="$NOWHERE" -DSOPHUS_ALLOW_VERSION_MISMATCH=ON

# 13. 타깃에 include 속성이 없는 설치본 → Config 위치에서 <prefix>/include 로 추정
mk_install "$T/inst_noinc" "$REQ" noinc
run inst_noinc_ok ok "RESULT src=installed ver=$REQ dirs=$T/inst_noinc/include" \
  -DCMAKE_PREFIX_PATH="$T/inst_noinc" -DSOPHUS_INCLUDE_DIR="$NOWHERE"

# 14. 판 불일치 허용 모드의 설치본 → include 경로를 얻고, 메시지에 리스트 구분자 ';' 가 없어야 한다
run inst_old_allow_dirs ok "RESULT src=installed ver=1.22.0 dirs=$T/inst_old/include" \
  -DCMAKE_PREFIX_PATH="$T/inst_old" -DSOPHUS_INCLUDE_DIR="$NOWHERE" -DSOPHUS_ALLOW_VERSION_MISMATCH=ON
if grep -A3 "판 불일치" "$T/inst_old_allow_dirs.log" | grep -q ";"; then
  fail=$((fail+1)); echo "FAIL inst_old_allow_dirs: 메시지에 ';' 가 섞임"
fi

# 15. 앞에 판이 다른 설치본, 뒤에 의존 실패 설치본 → 실패 안내가 뒤쪽(실패한) 폴더를 가리켜야 한다
run old_then_depfail fail "inst_depfail" \
  -DCMAKE_PREFIX_PATH="$T/inst_old;$T/inst_depfail" -DSOPHUS_INCLUDE_DIR="$NOWHERE"
if grep "쓸 수 없음" "$T/old_then_depfail.log" | grep -q "inst_old/"; then
  fail=$((fail+1)); echo "FAIL old_then_depfail: 안내가 앞쪽(판 다른) 폴더를 가리킴"
fi

echo "sophus_pin: $pass passed, $fail failed"
[ "$fail" -eq 0 ]
