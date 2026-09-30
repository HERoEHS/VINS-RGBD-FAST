# VIO 평가/진단 스크립트 (SW1-1828 Phase 1 PoC)

녹화한 VIO 출력 bag과 입력 bag을 **오프라인**으로 분석하는 헬퍼.
(라이브 노드 아님 → 재현 빠름. 모두 `rosbag2_py`로 db3 직접 읽음.)

## 사전
```bash
source /opt/ros/humble/setup.bash
```

## 스크립트

| 파일 | 입력 | 출력 |
|------|------|------|
| `vio_offline.py <out_dir>` | `/vins_estimator/odometry`,`/extrinsic` 녹화 bag | \|pos\| max·시작-끝 갭·궤적 범위·extrinsic 수렴 |
| `vio_timeseries.py <out_dir>` | 위와 동일 | 시간별 \|pos\| (발산이 점진/급격인지) |
| `imu_motion.py [bag] [imu_topic]` | 입력 bag의 IMU | 15초 윈도우별 회전(gyro)·병진여기(accel std) |
| `depth_check.py [bag] [depth_topic] [n]` | 입력 bag의 depth(32FC1 m) | 거리 구간별 유효 픽셀 비율 |
| `bias_separation.py <bag> <tum...> [--yaml]` | 입력 bag + VINS camera_pose TUM(들) | **정지 자세 오차를 p(캘리브 bias 아티팩트, QC)와 진짜 자세 드리프트로 분리** (SW1-1837) |
| `dynobs_eval.sh <bag이름>` | 정지+동적 장애물 bag (프로토콜: `scratchpad/dynobs_protocol.md`) | 재생 3run + 무운동 기준 판정 원버튼 (SW1-1837) |
| `gt_xy_eval.py <bag> [vins.tum ...]` | GT 포함 bag(v13~v15) + VINS `/vins_estimator/odometry` TUM(들) | **핀 규약 xy 오차**(08-10 REPORT 재현): GT=`map→apriltag_gt_camera`∘정적 `gt/base_footprint`, 휠 대조군, 창별 RMS/med/p95/max. 휠만 돌려 RMS 0.475 m가 나오면 평가기 정상 (SW1-1828) |
| `replay_one.sh <label> <cfg> <bag> <out>` | `TASK_WS`(설치본 위치), 선택 `RECORD_TF=1`·`REPLAY_DOMAIN`(기본 77 — 다른 세션과 동시 재생이면 세션마다 다른 값)·`REPLAY_TOPICS`(재생할 토픽만 — 라이브 VINS 출력이 녹화된 실기 bag 은 입력 토픽만 틀어야 기록이 섞이지 않음, 10-01) | 1런 재생: `env -i` + 전용 도메인(잔존 노드 있으면 중단) + 바이너리·설정 md5 기록 → TUM·로그·meta(reboot/crash 수). 백그라운드 SIGINT 무시 함정을 피해 실제 PID로 정상 종료 (SW1-1828) |
| `tf_kpi_check.py <gt_bag> <out> <label...>` | `RECORD_TF=1`로 녹화한 `<label>.tfbag` | `map→body`∘정적 `vins/base_footprint` TF 경로와 odometry+레버 경로 비교, GT RMS, 시작 오차 대 R·LEVER. ⚠️원시 KPI에는 표시 앵커 레버 편향 약 9.3 cm가 공통으로 얹혀 있음(SW1-1828 critic) |
| `dynobs_stationary_eval.py <tum...>` | 정지 녹화의 VINS camera_pose TUM(들) | 2s 버킷 \|Δyaw\|·\|Δxy\|·\|Δz\| 타임라인 + 피크/종점 + 합격/취약 판정 (참값=무운동, 노이즈 바닥 0.003m/0.04° 실측) |

### `bias_separation.py` — 지표의 성격 전환 (SW1-1837)

부팅 캘리브가 단일 자세라 tilt/bias를 못 가르고 가속도계 turn-on bias를 레벨링 회전에 구워넣는다
(세션 가변). 그 결과 raw-acc 기준 '정지 자세 오차'는 **진짜 VINS 자세 드리프트 + bias 아티팩트가
뒤섞인 오염 지표**였고, 이 때문에 중력 재정렬을 수 주간 헛짚었다(→ 봉인).

이 도구는 정지들을 heading별로 모아 `m − e_z ≈ ε(t) + [Rz(ψ)−Rz(ψ_init)]·p` 를 회귀해 분리한다:
- **p** = 몸체 고정 bias 벡터(heading 따라 돎) → **QC 값**. wrapper 레벨링 각(`imu_offsets.yaml`
  `level_angle_deg`)과 `--yaml`로 교차검증(일치=인과 확증).
- **drift 변동(std/p2p)** = bias 제거 후 남는 VINS 자세 드리프트 — 절대값은 init 프레임 기울기라
  gauge(분리 불가), **시간 변동만 관측 가능량**.

성립 조건: 정지가 여러 heading + heading-시간 비상관(`bag_acceptance_check.py` C6~C8). 미충족 시
`corr(t,\|ψ\|)` 경고. p 추정은 드리프트-프리 클러스터(차분) 우선, 잔차 큰 run은 낮은 신뢰.
```bash
python3 scripts/eval/bias_separation.py ~/ros2_ws/bag/<bag> gv_r1.tum gv_r2.tum gv_r3.tum \
  --yaml <icm20948_ros2>/config/imu_offsets.yaml
```

### STILL-DRIFT 가드 재생 검증 절차 (SW1-1922)

가드 관련 변경은 로그 토큰을 **정확히** 세어 판정한다(안내문·유사 토큰 오집계 방지).

| 토큰 | 뜻 |
|---|---|
| `[STILL-DRIFT]` | 가드 발동(조기 재초기화 요청). v15·v14 에서는 0 이어야 한다 |
| `[STILL-DRIFT-SKIP-OVER]` | **스로틀 없음.** 건너뛰지 않았으면 셌을 solve(변위 > 상한인데 몸체 운동으로 건너뜀) — 이 줄 수가 "수정이 막은 계수" 라 인과 증거다 |
| `[STILL-DRIFT-SKIP] … 사유=gyro` | 휠은 정지지만 창 안 자이로 노름 평균 ≥ `still_drift_gyro_busy_rad_s` → 판정 건너뜀·카운터 리셋. **1 s 스로틀 표본**이라 개수는 solve 수가 아니다 |
| `[STILL-DRIFT-SKIP] … 사유=leg` | 가드 창이 다리 게이팅 구간(강제 종료 끝 +`still_drift_leg_gate_extend_sec`)과 겹치고 창 안 0.1 s 블록에 운동 증거가 있음. 같은 스로틀을 gyro 와 공유한다 |
| `system reboot` | 재초기화 횟수(`replay_one.sh` meta 의 reboots) |

```bash
# 로그에 줄바꿈 없는 printf 가 섞여 한 줄에 레코드 둘이 붙을 수 있다 → grep -c(줄 수) 대신 -o | wc -l
grep -o '\[STILL-DRIFT\]' <label>.log | wc -l            # 발동
grep -o '\[STILL-DRIFT-SKIP-OVER\]' <label>.log | wc -l  # 막은 계수(창별)
grep -o '\[STILL-DRIFT-SKIP\]' <label>.log | wc -l       # 건너뜀 표본(1 s 스로틀)
```

결함 재현 조건(v15 + 옛 카메라 설정 `vio_A_head.yaml`, 수정 전 +219.605 s 발동 3/3)에서 발동 0 인지,
현행 설정으로 v15·v14 KPI(`gt_xy_eval.py`)가 종전 대비 악화 없는지, v16_play 에서 대조군(수정 전 바이너리)과
같은 런 수로 >1 m 폭주가 늘지 않는지를 본다. 09-25 캠페인 스크립트와 결과:
`~/ros2_ws/bag/analysis/sw1_1922_still_drift_20260925/campaign.sh`, `summary.tsv`.

### 재초기화 시드 다리 정렬 확인 (SW1-1936)

재초기화가 난 런에서는 시드 3줄을 함께 본다(이름 로거 `vins_reboot_seed` — `/rosout` 이 아니라 콘솔·`~/.ros/log` 에만 남는다).

| 토큰 | 뜻 |
|---|---|
| `[REBOOT-SEED] … 시드 캡처(앵커\|정화pose)` | 고른 시드 재료와 그 자세(발행 프레임) |
| `[REBOOT-SEED-BRIDGE] … 방식=시드자세시각 다리시작 t=… 캡처까지 다리회전=…deg` | 다리 시작 시각(= 시드 자세 시각)과 그때부터 캡처까지 운동 구간 회전. 앵커를 잡은 뒤 들어 돌렸다면 여기 그 회전이 보여야 한다. `방식=캡처시각(옛)` 이면 `reboot_seed_bridge_align: 0` |
| `[REBOOT-SEED] T_seed 확정 … (다리 …m / …deg, 다리시작 t=…)` | 확정된 시드. yaw = 재료 yaw + 다리 회전 |

합격 판정은 발행 yaw 로 한다: 재초기화 직전·직후 발행 yaw 차(계단)가 사라지고, 재초기화 뒤 발행 yaw − bag 자이로 방위가 몇 도 이내.

## VIO 출력 녹화 (분석 대상 만들기)
```bash
ros2 bag record -o /tmp/vio_out /vins_estimator/odometry /vins_estimator/extrinsic
# (다른 터미널에서) estimator + 입력 bag 재생
```

## 예시 (klt_gate)
```bash
python3 scripts/eval/vio_offline.py    /tmp/vio_out
python3 scripts/eval/vio_timeseries.py /tmp/vio_out
python3 scripts/eval/imu_motion.py     /home/higony/ros2_ws/bag/klt_gate
python3 scripts/eval/depth_check.py    /home/higony/ros2_ws/bag/klt_gate
```

## Phase 1 PoC 요약 (이 스크립트들로 도출)
- EDIE klt_gate: VIO \|pos\| 558~838m 발산. 회전 본격화(30s) 직후부터 가속 발산 → scale degeneracy.
- 근거리(<1.5m) depth 1.9%뿐, 장면 ~2m(58mm baseline 노이즈).
- OpenLORIS 레퍼런스(`../openloris/imu_merge.py` 사용): 3.48m bounded → 빌드 무결.
- 결론: vision+IMU 단독 한계 → 휠/평면제약 필요. (Linear SW1-1828)
