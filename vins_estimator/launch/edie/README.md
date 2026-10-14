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
| `config_file` | `/opt/edie/current/share/vins_estimator/config/elp_stereo_edie/vio_edie.yaml` (그 파일이 있을 때), 없으면 `<share>/vins_estimator/config/elp_stereo_edie/vio_edie.yaml` | estimator config 절대경로. 파일마다 **/opt/edie 우선 → 패키지 share**(설치본) — 소스 트리가 없는 로봇·상품 Yocto 에서도 뜬다 (SW1-1948, SW1-1959) |
| `vins_folder` | **기본** `config_file` 의 루트(`.../vins_estimator/`, 끝 `/`) — `config_file:=` 로 다른 파일을 넘겨도 따라가지 않으니 필요하면 같이 넘긴다 | 코드가 `config/` 를 이어붙임(fisheye mask, `fisheye: 0` 이면 안 씀) |
| `use_vision_calib` | `true` | **edie_vslam.launch.py 만.** 시작할 때 `config_file` 의 카메라 `fx·fy·cx·cy` 를 edie_vision `pinhole_left_calib.yaml` 정류 P(`/opt/edie` 우선 → share)로 덮은 사본(`/tmp/edie_vins-<uid>/vins_*_vio_edie.yaml`, 실행마다 새 파일)을 넘긴다. 로봇별 캘리브를 바꾸면 재시작만으로 반영. 캘리브 파일은 vision 과 같은 규칙(`camera_v4l2.yaml` 옆의 `left.calibration_file`)으로 찾는다. `left.use_undistort` 가 true 가 아니거나 캘리브를 못 읽으면 원본 값 그대로 + WARN. `false` 면 끔. ⚠️ **bag 재생**은 녹화한 로봇이 아니라 이 PC 의 캘리브가 들어가므로, 다른 로봇 bag 이면 `use_vision_calib:=false` 또는 그 로봇 캘리브를 쓴다 (SW1-1962) |
| `bag` | (빈값) | 주면 `ros2 bag play` 자동 실행 |
| `rate` | `1.0` | bag 재생 배속 |
| `rqt` | `true` | rqt_image_view 자동 실행 |

> 설정 원본은 저장소 루트 `config/` 이고 CMake 가 통째로 `share/vins_estimator/config` 에 설치한다.
> `/opt/edie/current/share/vins_estimator/` 는 edie_common `install_edie_share.sh`(기본 대상에 vins_estimator 포함)가 설치본을 복사해 채운다 — 규칙은 edie_vision·edie_localization launch 의 `_share_file` 과 같다(SW1-1959). 카메라 내부 값은 `edie_vslam.launch.py` 가 시작할 때 edie_vision 캘리브로 덮는다(`use_vision_calib`, SW1-1962) — vio_edie.yaml 의 `projection_parameters` 는 `use_vision_calib:=false`·`gate1_klt.launch.py`(bag 재생)일 때만 쓰인다.
> PC 는 `colcon build --symlink-install` 이라 소스 yaml 을 고치면 재빌드 없이 반영된다(복사 설치본은 재빌드 필요 — 그 기계에 `/opt/edie/current/share/vins_estimator` 사본이 있으면 그 사본이 먼저 쓰이므로 `install_edie_share.sh` 도 다시 돌린다).
> ⚠️ 이 변경을 pull 한 symlink 워크스페이스는 **vins_estimator 를 한 번 재빌드해야** share/config 가 채워진다 — 재빌드 전에는
> launch 기본값이 빈 폴더를 가리켜 "설정 파일을 열 수 없음" 으로 즉시 exit 1 한다.
> 설정 파일을 못 열면 estimator 는 FATAL 로그 뒤 exit 1 로 바로 끝난다(예전엔 값 0 으로 가다 abort).

> 같은 캘리브 읽기 함수(`_share_file`·`_rect_intrinsics`)가 bringup `bringup_robot.launch.py`·mind `edie_mind.launch.py`·VINS `edie_vslam.launch.py` 세 곳에 있다(저장소가 달라 공유 불가) — 규칙을 바꿀 때 **세 사본을 같이 고친다**.

## 게이트① 판정 기준

- 코너 수가 `max_cnt`(150) 근접
- 프레임 간 KLT 추적 생존율 양호, 화면 분포 고름(한 곳 몰림 X)
- 밋밋한 흰 벽/천장·반복 패턴 구간에서 추적 유지 여부 확인
- NO-GO 시 VIW-Fusion/VINS-RGBD 방향 자체 재검토
