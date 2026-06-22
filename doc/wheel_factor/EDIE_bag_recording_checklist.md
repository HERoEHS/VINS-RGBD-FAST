# EDIE 휠 factor 검증용 bag 녹화 체크리스트 (SW1-1829)

> 목적: **기본(공칭) 자세 단일 주행**에서 휠 factor 발산 여부를 EDIE 실데이터로 검증.
> (설계·로직은 OpenLORIS로 검증 완료 → 남은 건 EDIE 데이터 1개.)

## 필수 토픽
| 토픽 | 타입 | 용도 |
|------|------|------|
| `/edie/diff_drive_controller/odom` | nav_msgs/Odometry | **휠 factor 입력(twist)** — 현재 어떤 bag에도 없음 |
| `/edie/sensors/camera/left/image_gray` | sensor_msgs/Image | VIO 영상 (vio_edie.yaml image_topic) |
| `/edie/sensors/camera/left/depth` | sensor_msgs/Image | RGBD depth |
| `/edie/sensor/offset_imu` (또는 실제 IMU 토픽) | sensor_msgs/Imu | VIO IMU |
| `/joint_states` | sensor_msgs/JointState | 다리각 기록(Step3 대비, 기본자세 확인용) |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | 프레임/디버그 |

## 녹화 전 확인
- [ ] 다리를 **기본(공칭) 자세로 고정**하고 그 자세로만 주행 (인출입 금지)
- [ ] `image_gray`와 `depth` 타임스탬프 ±3ms 이내 (estimator 동기 조건; gate1_klt README 참조)
- [ ] diff_drive_controller가 odom을 실제로 publish 하는지: `ros2 topic hz /edie/diff_drive_controller/odom`
- [ ] IMU 실제 토픽명 확인 후 vio_edie.yaml `imu_topic`과 일치

## 주행 시나리오 (권장)
- [ ] 충분한 병진+회전 (휠/VIO 모두 여기). 정지 구간 일부 포함(static_init 확인)
- [ ] **루프 클로저**(출발점 복귀)로 시작-끝 갭 측정 가능하게
- [ ] 30~60초, 너무 빠른 회전 자제(KLT 추적 생존)

## 녹화 명령 (예시)
```bash
ros2 bag record -o ~/ros2_ws/bag/edie_wheel_base \
  /edie/diff_drive_controller/odom \
  /edie/sensors/camera/left/image_gray \
  /edie/sensors/camera/left/depth \
  <실제_IMU_토픽> /joint_states /tf /tf_static
```

## 녹화 후 검증 (A/B)
```bash
# vio_edie.yaml use_wheel:0 (baseline) / 1 (wheel) 토글하며 2회
# (러너/평가 스크립트는 scripts/openloris·scripts/eval 패턴 재사용)
python3 scripts/eval/vio_offline.py <out>     # |pos| 발산 여부 (>20m 의심)
python3 scripts/eval/vio_timeseries.py <out>  # 발산이 점진/급격인지
```
- 합격 기준: **use_wheel:1에서 발산 없음**(|pos| 방 규모 유지) + baseline 대비 비퇴행.
- body_T_wheel 기본자세 값은 vio_edie.yaml에 이미 기입(URDF FK: R=I, t=(0.1056,0,-0.0941)).
