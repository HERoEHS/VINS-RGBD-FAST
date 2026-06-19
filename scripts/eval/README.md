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
