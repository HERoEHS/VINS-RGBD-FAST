#!/usr/bin/env bash
# =============================================================================
# OpenLORIS 휠 factor E2E 1회 실행 (SW1-1829)
#   estimator(KLT, GPU/YOLO 불필요) + imu_merge + bag play + 출력 record 를
#   오케스트레이션한다. config만 바꿔 baseline(use_wheel:0) vs wheel(use_wheel:1)
#   두 번 돌린 뒤 scripts/eval 로 비교.
#
# 사용:
#   bash run_openloris_e2e.sh <config.yaml> <out_dir> [bag_dir] [rate]
# 예:
#   bash run_openloris_e2e.sh \
#     <repo>/config/openloris/openloris_vio.yaml       /tmp/vio_ol_base  ~/Downloads/openloris/office1-1_ros2
#   bash run_openloris_e2e.sh \
#     <repo>/config/openloris/openloris_vio_wheel.yaml /tmp/vio_ol_wheel ~/Downloads/openloris/office1-1_ros2
#
# 주의: 멀티프로세스(estimator/merge/record/play)를 백그라운드로 띄우고
#       bag 재생이 끝나면 모두 정리한다. ros2 환경이 source 되어 있어야 함.
# =============================================================================
set -u

CONFIG="${1:?config.yaml 경로 필요}"
OUT="${2:?출력 bag 디렉토리 필요}"
BAG="${3:-$HOME/Downloads/openloris/office1-1_ros2}"
RATE="${4:-1.0}"

# vins_folder = repo 루트 (끝에 '/' 필수: parameters.cpp가 문자열 이어붙임)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VINS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)/"
MERGE_PY="$SCRIPT_DIR/imu_merge.py"

echo "[E2E] config   = $CONFIG"
echo "[E2E] out      = $OUT"
echo "[E2E] bag      = $BAG"
echo "[E2E] vins_root= $VINS_ROOT"

[ -f "$CONFIG" ] || { echo "✗ config 없음"; exit 1; }
[ -d "$BAG" ]    || { echo "✗ bag 디렉토리 없음"; exit 1; }
rm -rf "$OUT"

# 각 자식을 setsid로 새 프로세스 그룹(PGID=PID) 리더로 띄운다.
# → cleanup에서 `kill -- -PGID`로 ros2 run 래퍼 + orphan C++ 노드까지 한 번에 종료.
#   (이전 버그: kill $!는 래퍼만 죽이고 estimator 노드가 orphan으로 살아남아 토픽 오염)
MERGE_PID=""; EST_PID=""; REC_PID=""
cleanup() {
  echo "[E2E] 정리 중..."
  # record는 db3 정상 flush 위해 먼저 SIGINT
  [ -n "$REC_PID" ] && kill -INT -- "-$REC_PID" 2>/dev/null
  sleep 2
  for p in "$REC_PID" "$EST_PID" "$MERGE_PID"; do
    [ -n "$p" ] && kill -TERM -- "-$p" 2>/dev/null
  done
  sleep 1
  for p in "$REC_PID" "$EST_PID" "$MERGE_PID"; do
    [ -n "$p" ] && kill -9 -- "-$p" 2>/dev/null
  done
  # 안전망: 혹시 남은 동일 실행파일 orphan 제거
  pkill -9 -f 'vins_estimator_node' 2>/dev/null
  pkill -9 -f "$MERGE_PY" 2>/dev/null
}
trap cleanup EXIT INT TERM

# 1) IMU 병합 (accel+gyro → /d400/imu0)
setsid python3 "$MERGE_PY" &  MERGE_PID=$!
sleep 1

# 2) estimator (KLT 내장). config_file/vins_folder는 ros 파라미터로 주입.
setsid ros2 run vins_estimator vins_estimator_node --ros-args \
  -p config_file:="$CONFIG" -p vins_folder:="$VINS_ROOT" &  EST_PID=$!
sleep 3

# 3) 출력 record: VIO 추정 + extrinsic + 정답(/gt) + 휠(/odom)
setsid ros2 bag record -o "$OUT" \
  /vins_estimator/odometry /vins_estimator/extrinsic /gt /odom &  REC_PID=$!
sleep 2

# 4) bag 재생 (blocking) — 끝나면 다음 줄로
echo "[E2E] bag 재생 시작 (rate=$RATE)..."
ros2 bag play "$BAG" --rate "$RATE"
echo "[E2E] bag 재생 완료. 5초 flush 후 종료."
sleep 5
# cleanup()는 trap EXIT에서 자동 실행
echo "[E2E] 완료 → $OUT"
