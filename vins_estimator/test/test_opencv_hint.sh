#!/usr/bin/env bash
# OpenCvSystemHint.cmake 동작 검증 (SW1-1959)
#   가짜 "시스템 lib 뿌리"를 만들어 configure 만 돌리고(컴파일러 불필요), 네 경우를 확인한다:
#   네이티브 → 힌트 적용 / 교차 빌드 → 힌트 건너뜀 / -DOpenCV_DIR 지정 → 그대로 / 뿌리에 OpenCV 없음 → 힌트 없음.
#   그리고 세 패키지의 모듈 사본이 같은지 확인한다(한 곳만 고치는 실수 방지).
# 사용: test_opencv_hint.sh [cmake 실행파일] [VINS 저장소 루트]
set -u

CMAKE="${1:-cmake}"
ROOT="${2:-$(cd "$(dirname "$0")/../.." && pwd)}"
MODULE="$ROOT/vins_estimator/cmake/OpenCvSystemHint.cmake"
[ -f "$MODULE" ] || { echo "모듈 없음: $MODULE"; exit 2; }

T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
pass=0; fail=0
MAKE_PROGRAM="$(command -v make || command -v gmake || echo /bin/true)"

# 0. 세 사본 동일성
for p in camera_model pose_graph; do
  if cmp -s "$MODULE" "$ROOT/$p/cmake/OpenCvSystemHint.cmake"; then
    pass=$((pass+1)); echo "PASS copy_identical_$p"
  else
    fail=$((fail+1)); echo "FAIL copy_identical_$p: $p/cmake/OpenCvSystemHint.cmake 가 vins_estimator 사본과 다름"
  fi
done

# 0-b. 세 CMakeLists 에서 include(…OpenCvSystemHint.cmake) 가 첫 find_package 보다 앞에 있어야 한다
#      (뒤로 밀리면 cv_bridge/camera_model config 가 먼저 OpenCV_DIR 을 박아 힌트가 무력화된다 — SW1-1837 의 핵심)
for p in vins_estimator camera_model pose_graph; do
  f="$ROOT/$p/CMakeLists.txt"
  inc_line=$(grep -n 'OpenCvSystemHint.cmake)' "$f" | grep -v '^[0-9]*:#' | head -1 | cut -d: -f1)
  fp_line=$(grep -n '^find_package(' "$f" | head -1 | cut -d: -f1)
  if [ -n "$inc_line" ] && [ -n "$fp_line" ] && [ "$inc_line" -lt "$fp_line" ]; then
    pass=$((pass+1)); echo "PASS include_before_find_package_$p"
  else
    fail=$((fail+1)); echo "FAIL include_before_find_package_$p (include=$inc_line, first find_package=$fp_line)"
  fi
done

# 가짜 시스템 lib 뿌리: <root>/x86_64-linux-gnu/cmake/opencv4
FAKE="$T/sysroot_lib"; mkdir -p "$FAKE/x86_64-linux-gnu/cmake/opencv4"
EMPTY="$T/empty_lib"; mkdir -p "$EMPTY"

mkdir -p "$T/proj"
cat > "$T/proj/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.16)
project(opencv_hint_probe LANGUAGES NONE)
include("$MODULE")
message(STATUS "RESULT OpenCV_DIR=[\${OpenCV_DIR}]")
EOF

# run <이름> <로그에 있어야 할 문구> [cmake 인자...]  — configure 는 항상 성공해야 한다
run() {
  local name="$1" needle="$2"; shift 2
  local b="$T/b_$name"; rm -rf "$b"
  "$CMAKE" -S "$T/proj" -B "$b" -DCMAKE_MAKE_PROGRAM="$MAKE_PROGRAM" "$@" > "$T/$name.log" 2>&1
  local rc=$?
  if [ $rc -eq 0 ] && grep -qF -- "$needle" "$T/$name.log"; then
    pass=$((pass+1)); echo "PASS $name"
  else
    fail=$((fail+1)); echo "FAIL $name (rc=$rc, 문구='$needle')"; sed 's/^/    /' "$T/$name.log" | tail -8
  fi
}

# 1. 네이티브 + 뿌리에 OpenCV 있음 → 그 폴더가 OpenCV_DIR
run native_hint "RESULT OpenCV_DIR=[$FAKE/x86_64-linux-gnu/cmake/opencv4]" \
  -DVINS_OPENCV_SYS_LIB_ROOT="$FAKE"

# 2. 교차 빌드(CMAKE_SYSTEM_NAME 지정) + 같은 뿌리 → 힌트 건너뜀, OpenCV_DIR 비어 있음
run cross_skip "RESULT OpenCV_DIR=[]" \
  -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64 -DVINS_OPENCV_SYS_LIB_ROOT="$FAKE"
grep -q "교차 빌드" "$T/cross_skip.log" || { fail=$((fail+1)); echo "FAIL cross_skip: 교차 빌드 안내 없음"; }

# 3. -DOpenCV_DIR 외부 지정 → 네이티브·교차 모두 그대로
run explicit_native "RESULT OpenCV_DIR=[$T/explicit]" \
  -DOpenCV_DIR="$T/explicit" -DVINS_OPENCV_SYS_LIB_ROOT="$FAKE"
run explicit_cross "RESULT OpenCV_DIR=[$T/explicit]" \
  -DCMAKE_SYSTEM_NAME=Linux -DOpenCV_DIR="$T/explicit" -DVINS_OPENCV_SYS_LIB_ROOT="$FAKE"

# 4. 네이티브 + 뿌리에 OpenCV 없음 → 힌트 없음(OpenCV_DIR 비어 있음), 에러 아님
run native_nohint "RESULT OpenCV_DIR=[]" \
  -DVINS_OPENCV_SYS_LIB_ROOT="$EMPTY"

echo "opencv_hint: $pass passed, $fail failed"
[ "$fail" -eq 0 ]
