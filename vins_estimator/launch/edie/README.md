# EDIE VIO — 게이트① (KLT 추적 생존율) launch

VIO Phase 1 (Linear SW1-1828)의 GO/NO-GO 게이트①을 위한 실행 파일.
55° 상향 틸트 scene에서 KLT 특징 추적이 충분히 살아남는지 정성 확인한다.

## 사전 준비

1. 빌드: `colcon build --packages-select vins_estimator camera_model pose_graph`
2. 환경: `source install/setup.bash`
3. **녹화 bag**: 아래 토픽이 들어 있어야 한다 (config `vio_edie.yaml` 기준).

| 토픽 | 타입 | 비고 |
|------|------|------|
| `/edie/sensors/camera/left/image_gray` | sensor_msgs/Image (mono8) | 필수 |
| `/edie/sensors/camera/left/depth` | sensor_msgs/Image | 필수. image와 **±3ms 이내** 동기 |
| `/edie/sensor/offset_imu` | sensor_msgs/Imu | 필수, ~200Hz+ |

> ⚠️ depth가 없으면 estimator의 추적 스레드가 color+depth 페어를 기다리며
> 멈춰 KLT가 한 프레임도 처리되지 않는다(`estimator_nodelet.cpp` L183).

## 실행

```bash
# 노드 + 뷰어만 (bag은 다른 터미널에서 play)
ros2 launch vins_estimator gate1_klt.launch.py

# bag 자동 재생까지
ros2 launch vins_estimator gate1_klt.launch.py bag:=/path/to/rosbag2_dir rate:=1.0
```

추적 결과는 `rqt_image_view`(`/vins_estimator/feature_img`)로 표시된다.

## 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `config_file` | `.../config/elp_stereo_edie/vio_edie.yaml` | estimator config 절대경로 |
| `vins_folder` | VINS-RGBD-FAST 루트(끝 `/`) | 코드가 `config/` 를 이어붙임 |
| `bag` | (빈값) | 주면 `ros2 bag play` 자동 실행 |
| `rate` | `1.0` | bag 재생 배속 |
| `rqt` | `true` | rqt_image_view 자동 실행 |

## 게이트① 판정 기준

- 코너 수가 `max_cnt`(150) 근접
- 프레임 간 KLT 추적 생존율 양호, 화면 분포 고름(한 곳 몰림 X)
- 밋밋한 흰 벽/천장·반복 패턴 구간에서 추적 유지 여부 확인
- NO-GO 시 VIW-Fusion/VINS-RGBD 방향 자체 재검토
