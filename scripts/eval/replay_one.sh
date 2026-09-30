#!/bin/bash
# VINS 1런 재생: 깨끗한 환경(env -i)에서 task 설치본 바이너리 + 지정 설정으로 bag 재생 → TUM 기록
# 사용: TASK_WS=<작업공간> [RECORD_TF=1] [REPLAY_DOMAIN=<id>] [REPLAY_TOPICS="<토픽 ...>"] replay_one.sh <label> <config_yaml> <bag_dir> <out_dir>
#   REPLAY_DOMAIN: 전용 ROS 도메인(기본 77). 여러 세션이 동시에 재생할 수 있으면 세션마다 다른 값을 쓴다 —
#   같은 도메인이면 토픽이 섞여 서로의 검증을 오염시킨다(09-29 세션 간 격리 규칙).
#   REPLAY_TOPICS: 재생할 토픽만(공백 구분). 비우면 bag 전체. 라이브 /vins_estimator/odometry 가 녹화된 실기 bag
#   (t3_field_0930·lift_yaw_0930 등)은 전체 재생하면 기록기가 라이브·재생 두 흐름을 같은 시각에 섞어 받는다(10-01) —
#   입력 토픽(영상·깊이·IMU·휠·joint_states·다리 명령·/tf_static)만 틀 것.
#   VINS 소스 위치가 다르면 VINS_ROOT=<.../VINS-RGBD-FAST/> 로 덮어쓴다(메인 ws: src/edie9/edie_localization/VINS-RGBD-FAST/).
#   평가: gt_xy_eval.py <bag_dir> <out_dir>/*.tum (08-10 REPORT 핀 규약, 휠 대조군 RMS 0.475 로 평가기 검증됨)
# 규약(VINS 메모리): 최종 바이너리, env -i, 바이너리 md5 기록, 정리는 PID kill 만(pkill -f 금지)
#
# ⚠️ 09-23 결함 수정: 비대화형 셸의 백그라운드(&) 프로세스는 SIGINT 가 '무시'로 상속된다.
#   ① 셸 함수로 감싸 & 하면 $! 가 서브셸 PID 라 신호가 실제 노드에 안 닿았다 → env 를 직접 & 한다
#      (env → bash -c "...; exec 노드" 가 모두 exec 라 $! == 노드 PID).
#   ② python 은 시작 시 SIGINT 가 무시면 계속 무시한다 → 래퍼로 기본 SIGINT(KeyboardInterrupt)를 되살려
#      기록기의 finish()(남은 버퍼 flush·close)가 돌게 한다.
set -u
LABEL=$1; CFG=$2; BAG=$3; OUT=$4
TASK_WS=${TASK_WS:?task 작업공간 경로를 TASK_WS 로 지정}
VINS_ROOT=${VINS_ROOT:-$TASK_WS/src/edie_localization/VINS-RGBD-FAST/}
BIN=$TASK_WS/install/vins_estimator/lib/vins_estimator/vins_estimator_node
REC=$VINS_ROOT/scripts/vins_tum_recorder.py
mkdir -p "$OUT"
LOG=$OUT/$LABEL.log; TUM=$OUT/$LABEL.tum; META=$OUT/$LABEL.meta

# 다른 ROS 노드와 섞이지 않게 전용 도메인 + localhost 전용
DOMAIN=${REPLAY_DOMAIN:-77}
ENVSETUP="source /opt/ros/humble/setup.bash; source /home/higony/ros2_ws/install/setup.bash; source $TASK_WS/install/setup.bash; export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1"
CLEAN=(env -i HOME=$HOME PATH=/usr/bin:/bin USER=$USER bash -c)
PYWRAP="import signal,runpy,sys; signal.signal(signal.SIGINT, signal.default_int_handler); sys.argv=sys.argv[1:]; runpy.run_path(sys.argv[0], run_name='__main__')"

# 오염 방지: 전용 도메인에 이미 노드가 있으면(이전 런 잔존 등) 시작하지 않는다
# (09-23 사고: 대기 상태로 남은 이전 추정기가 새 런과 같은 도메인에서 함께 발행)
LEFT=$("${CLEAN[@]}" "$ENVSETUP; timeout 10 ros2 node list --no-daemon 2>/dev/null")
if [ -n "$LEFT" ]; then
  echo "ABORT $LABEL: 도메인 $DOMAIN 에 잔존 노드: $LEFT" | tee "$META"; exit 3
fi

{
  echo "label=$LABEL"; echo "domain=$DOMAIN"; echo "config=$CFG"; echo "config_md5=$(md5sum < "$CFG" | cut -d' ' -f1)"
  echo "binary=$BIN"; echo "binary_md5=$(md5sum < "$BIN" | cut -d' ' -f1)"
  echo "vins_head=$(git -C "$VINS_ROOT" rev-parse HEAD) dirty=$(git -C "$VINS_ROOT" status --porcelain | wc -l)"
  echo "bag=$BAG"; echo "topics=${REPLAY_TOPICS:-all}"; echo "start=$(date -Is)"
} > "$META"

"${CLEAN[@]}" "$ENVSETUP; exec $BIN --ros-args -r __node:=vins_estimator -p config_file:=$CFG -p vins_folder:=$VINS_ROOT -p use_sim_time:=true" > "$LOG" 2>&1 &
EST=$!
sleep 6
"${CLEAN[@]}" "$ENVSETUP; exec python3 -c \"$PYWRAP\" $REC --topic /vins_estimator/odometry --msg-type odom --output $TUM --ros-args -p use_sim_time:=true" > "$OUT/$LABEL.rec.log" 2>&1 &
RECP=$!
TFR=""
if [ "${RECORD_TF:-0}" = 1 ]; then
  # 계획서 KPI 경로(vins/base_footprint TF)를 직접 검증하려고 VINS 출력 TF 도 녹화한다
  "${CLEAN[@]}" "$ENVSETUP; exec python3 -c \"$PYWRAP\" /opt/ros/humble/bin/ros2 bag record -o $OUT/$LABEL.tfbag /tf /tf_static /vins_estimator/odometry" > "$OUT/$LABEL.tfrec.log" 2>&1 &
  TFR=$!
fi
sleep 3
TOPICS_ARG=""
[ -n "${REPLAY_TOPICS:-}" ] && TOPICS_ARG="--topics ${REPLAY_TOPICS}"
"${CLEAN[@]}" "$ENVSETUP; ros2 bag play $BAG --clock 400 $TOPICS_ARG" > "$OUT/$LABEL.play.log" 2>&1
sleep 4

# 기록기 → 추정기 순으로 정상 종료 요청, 시간 제한 뒤 강제 종료
stop_pid() {  # $1=PID $2=이름
  kill -INT "$1" 2>/dev/null
  for _ in $(seq 1 30); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done
  echo "warn: $2 가 15초 안에 끝나지 않아 SIGKILL" >> "$META"; kill -9 "$1" 2>/dev/null
}
stop_pid $RECP recorder
[ -n "$TFR" ] && stop_pid $TFR tfbag
stop_pid $EST estimator
wait $EST 2>/dev/null; RC=$?
{
  echo "end=$(date -Is)"; echo "estimator_exit=$RC"
  echo "reboots=$(grep -c 'system reboot' "$LOG")"
  echo "crash_markers=$(grep -c -E 'Segmentation fault|SIGSEGV|Assertion|terminate called|core dumped' "$LOG")"
  echo "tum_lines=$(wc -l < "$TUM" 2>/dev/null || echo 0)"
  echo "recorder_final=$(tail -1 "$OUT/$LABEL.rec.log" | cut -c1-120)"
} >> "$META"
cat "$META"
