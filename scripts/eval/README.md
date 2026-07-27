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
