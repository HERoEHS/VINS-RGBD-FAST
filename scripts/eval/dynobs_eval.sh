#!/bin/bash
# [SW1-1837] 정지+동적 장애물 실험 — 재생 3run + 무운동 기준 평가 원버튼
# 사용: bash scripts/eval/dynobs_eval.sh <bag이름>  (~/ros2_ws/bag/<이름> 폴더)
# 산출: ~/ros2_ws/bag/<이름>_r{1..3}.tum + 판정 리포트(stdout), est 로그는 /tmp/dynobs_<이름>/
# 프로토콜·판정 기준: scratchpad/dynobs_protocol.md
set -u
NAME=${1:?사용법: dynobs_eval.sh <bag이름>}
BAG=~/ros2_ws/bag/$NAME
[ -d "$BAG" ] || { echo "bag 없음: $BAG"; exit 1; }
export ROS_DOMAIN_ID=77
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG=$SRC/config/elp_stereo_edie/vio_edie.yaml
LOG=/tmp/dynobs_$NAME
mkdir -p "$LOG"
export VINS_BG_LOG=1
# 재생 중 hf TF만 끄고 종료 시 원복 (검증 run 공통 관례)
cp "$CFG" "$LOG/cfg.bak"
trap 'cp "$LOG/cfg.bak" "$CFG"' EXIT
sed -i "s/^publish_hf_body_tf: ./publish_hf_body_tf: 0/" "$CFG"

run_one() {
  local OUT=~/ros2_ws/bag/${NAME}_$1.tum
  rm -f "$OUT"
  ros2 launch vins_estimator edie_vslam.launch.py use_sim_time:=true log_level:=info > "$LOG/est_$1.log" 2>&1 &
  local EST=$!
  sleep 5
  python3 "$SRC/scripts/vins_tum_recorder.py" --topic /vins_estimator/camera_pose --msg-type pose \
    --output "$OUT" --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
  local REC=$!
  sleep 3
  ros2 bag play "$BAG" --clock 400 > /dev/null 2>&1
  sleep 3
  kill -INT $REC 2>/dev/null; sleep 3
  kill -INT $EST 2>/dev/null; sleep 2
  kill -9 $REC $EST 2>/dev/null
  for P in $(pgrep -f "vins_estimator_nod[e]"); do kill -9 "$P" 2>/dev/null; done
  sleep 4
  echo "[$1] $(wc -l < "$OUT" 2>/dev/null || echo 0) poses"
}
for R in r1 r2 r3; do echo "===== $NAME $R ====="; run_one $R; done

echo ""
python3 "$SRC/scripts/eval/dynobs_stationary_eval.py" \
  ~/ros2_ws/bag/${NAME}_r1.tum ~/ros2_ws/bag/${NAME}_r2.tum ~/ros2_ws/bag/${NAME}_r3.tum
echo "DYNOBS_DONE"
