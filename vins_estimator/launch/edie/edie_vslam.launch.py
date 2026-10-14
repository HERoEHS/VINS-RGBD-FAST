#!/usr/bin/env python3
# [SW1-1837] edie 라이브 VIO 실행 launch — 실 로봇에서 vins_estimator만 헤드리스로 띄운다.
#   gate1_klt.launch.py와 달리 rqt/bag 자동실행 없음 → SSH 헤드리스 라이브 테스트 친화.
#   config_file/vins_folder 기본값은 파일마다 /opt/edie 우선, 없으면 패키지 share(설치본) — 소스 체크아웃이
#   없는 로봇에서도 그대로 뜬다(SW1-1948, SW1-1959).
#   시각화가 필요하면 별도 쉘에서 rviz2 직접 실행 (bag 재생 시: rviz2 --ros-args -p use_sim_time:=true).
#
# 사용 예:
#   ros2 launch vins_estimator edie_vslam.launch.py   # 헤드리스 라이브 (기본 = /opt 또는 share 설정)
#   ros2 launch vins_estimator edie_vslam.launch.py config_file:=<다른 yaml>
#   ※ 게이팅 on/off는 config(vio_edie.yaml)의 use_wheel_vel_gate로 토글 후 재실행.
#   [SW1-1962] 카메라 fx·fy·cx·cy 는 시작할 때 edie_vision 캘리브(정류 P)로 덮어쓴다 — 로봇별 캘리브를
#   바꾸면 재시작만으로 반영된다. 끄려면 use_vision_calib:=false (vio_edie.yaml 값 그대로).
import hashlib
import os
import tempfile

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
import yaml
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 설정 파일 경로: 파일마다 1순위 /opt/edie/current/share/<pkg>/<rel>(그 파일이 있을 때만),
# 없으면 개발 워크스페이스 ament share, 둘 다 없으면 1순위 후보(SW1-1959).
# edie_vision config_path.hpp·edie_hazard_filter config_path.py·edie_localization launch 의
# _share_file 과 같은 규칙 — 같이 바꾼다(이 저장소의 두 launch 사본은
# test_installed_config.py 가 동일성까지 확인).
_OPT_SHARE_ROOT = '/opt/edie/current/share'


def _share_file(package, rel):
    opt_path = os.path.join(_OPT_SHARE_ROOT, package, rel)
    if os.path.exists(opt_path):
        return opt_path
    try:
        dev_path = os.path.join(get_package_share_directory(package), rel)
        if os.path.exists(dev_path):
            return dev_path
    # ament 에 패키지가 없음·색인 경로 없음(상품 이미지 등)
    except (PackageNotFoundError, ValueError, OSError):
        pass
    return opt_path


# vio_edie.yaml 의 패키지 share 안 상대 경로. vins_folder 는 이 파일을 고른 쪽의 루트(끝 '/' 포함).
_CONFIG_REL = os.path.join('config', 'elp_stereo_edie', 'vio_edie.yaml')


def _vins_root_of(config_path):
    # <루트>/<_CONFIG_REL> → <루트>/ (코드가 vins_folder + "config/..." 로 이어붙임).
    # _CONFIG_REL 깊이만큼 올라가므로 상대 경로가 바뀌어도 루트가 어긋나지 않는다.
    root = config_path
    for _ in _CONFIG_REL.split(os.sep):
        root = os.path.dirname(root)
    return os.path.join(root, '')


# ---- [SW1-1962] edie_vision 캘리브 → vio_edie.yaml 카메라 값 ----
# 카메라 영상(image_gray)은 edie_vision 이 이미 편(정류) 영상이라 VINS 카메라 모델 = 정류 P(왜곡 0).
# 캘리브 파일은 edie_vision 이 영상을 펼 때와 똑같이 찾는다: camera_v4l2.yaml 을 /opt 우선으로 찾고,
# 그 옆의 left.calibration_file(gstreamer_utils.hpp LoadCameraConfig). 그래야 vision 이 편 값과 어긋나지 않는다.
_VISION_CAMERA_REL = os.path.join('config', 'gstreamer', 'camera_v4l2.yaml')
# 덮어쓴 사본을 두는 곳 — 재부팅하면 지워져도 되는 임시 파일(eMMC 대신 /tmp).
# 계정별 폴더: root 로 한 번 띄운 폴더 때문에 다른 계정이 못 쓰는 일을 막는다.
# 파일 이름 = 고친 내용의 지문(sha256 앞 8자): 같은 설정이면 늘 같은 파일 1개를 다시 써서 쌓이지 않고,
# 설정이 다르면 이름이 달라 동시에 띄운 두 launch 가 서로의 사본을 덮어쓰지 않는다.
_PATCHED_ROOT = '/tmp'
_PROJ_KEYS = ('fx', 'fy', 'cx', 'cy')


def _rect_intrinsics():
    """edie_vision 정류 P 의 (fx, fy, cx, cy) 와 출처 경로. 못 쓰면 (None, 이유)."""
    camera_yaml = _share_file('edie_vision', _VISION_CAMERA_REL)
    calib_yaml = camera_yaml
    try:
        with open(camera_yaml) as f:
            left = yaml.safe_load(f)['left']
        use_undistort = left['use_undistort']
        calib_yaml = os.path.join(os.path.dirname(camera_yaml), left['calibration_file'])
        if use_undistort is not True:
            # 정류 안 한 원본 영상이면 P 가 아니라 K+D 를 써야 해서 여기서 덮어쓰지 않는다.
            return None, f'{camera_yaml}: left.use_undistort={use_undistort} (정류 영상 아님)'
        with open(calib_yaml) as f:
            p = yaml.safe_load(f)['projection_matrix']['data']
        return (float(p[0]), float(p[5]), float(p[2]), float(p[6])), calib_yaml
    except (OSError, KeyError, IndexError, TypeError, ValueError, yaml.YAMLError) as e:
        return None, f'{calib_yaml}: {e}'


def _with_vision_intrinsics(config_text, k):
    """vio_edie.yaml 본문에서 projection_parameters 블록의 fx·fy·cx·cy 4줄만 바꾼 본문.

    블록이 없거나 네 줄이 다 없으면 ValueError — 엉뚱한 줄을 바꾸지 않게 한다.
    """
    lines = config_text.splitlines(keepends=True)
    try:
        start = next(i for i, ln in enumerate(lines) if ln.rstrip() == 'projection_parameters:')
    except StopIteration:
        raise ValueError('projection_parameters 블록 없음')
    vals = dict(zip(_PROJ_KEYS, k))
    done = set()
    for i in range(start + 1, len(lines)):
        ln = lines[i]
        if ln.strip() and not ln[0].isspace():
            break  # 들여쓰기 없는 줄 = 블록 끝
        key = ln.strip().split(':', 1)[0]
        if key in vals:
            indent = ln[:len(ln) - len(ln.lstrip())]
            lines[i] = f'{indent}{key}: {vals[key]!r}\n'
            done.add(key)
    if done != set(_PROJ_KEYS):
        raise ValueError(f'projection_parameters 에 {sorted(set(_PROJ_KEYS) - done)} 없음')
    return ''.join(lines)


def _write_patched_copy(config_path, text):
    """고친 본문을 <계정별 폴더>/<원본이름>.<내용 지문>.yaml 로 저장하고 그 경로를 돌려준다.

    같은 내용이면 이미 있는 파일을 그대로 쓴다. 새로 쓸 때는 임시 파일에 다 쓴 뒤 os.replace 로
    한 번에 바꿔 넣어, 동시에 읽는 쪽이 반쯤 쓰인 파일을 보지 않게 한다.
    """
    out_dir = os.path.join(_PATCHED_ROOT, f'edie_vins-{os.getuid()}')
    os.makedirs(out_dir, mode=0o700, exist_ok=True)
    # 다른 계정이 먼저 만든 폴더면 쓰지 않는다(이름이 예측 가능해 바꿔치기 막기) → 원본 + WARN.
    if os.lstat(out_dir).st_uid != os.getuid():
        raise OSError(f'{out_dir} 주인이 이 계정이 아님')
    stem, ext = os.path.splitext(os.path.basename(config_path))
    digest = hashlib.sha256(text.encode('utf-8')).hexdigest()[:8]
    out = os.path.join(out_dir, f'{stem}.{digest}{ext}')
    try:
        with open(out) as f:
            if f.read() == text:
                return out
    except OSError:
        pass  # 아직 없음 → 새로 쓴다
    fd, tmp = tempfile.mkstemp(prefix=f'.{stem}.', suffix='.tmp', dir=out_dir)
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(text)
        os.replace(tmp, out)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise
    return out


def _vision_calibrated_config(config_path):
    """config_path 의 카메라 값을 edie_vision 캘리브로 덮은 사본 경로. 못 하면 원본 경로 + WARN."""
    log = launch_logging.get_logger('edie_vslam')
    k, src = _rect_intrinsics()
    if k is None:
        log.warning(f'edie_vision 캘리브를 못 씀 — vio_edie.yaml 카메라 값 그대로 사용: {src}')
        return config_path
    try:
        with open(config_path) as f:
            text = _with_vision_intrinsics(f.read(), k)
        out = _write_patched_copy(config_path, text)
    except (OSError, ValueError) as e:
        log.warning(f'카메라 값 덮어쓰기 실패 — {config_path} 그대로 사용: {e}')
        return config_path
    log.info('카메라 값(edie_vision 캘리브) fx=%.3f fy=%.3f cx=%.3f cy=%.3f <- %s → %s'
             % (*k, src, out))
    return out


def _make_estimator(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    if LaunchConfiguration('use_vision_calib').perform(context).lower() == 'true':
        config_file = _vision_calibrated_config(config_file)
    return [Node(
        package='vins_estimator',
        executable='vins_estimator_node',
        name='vins_estimator',
        output='screen',
        # emulate_tty: stdout을 TTY로 위장 → printf/std::cout 가 풀버퍼링 안 되고 실시간 출력
        #   (RCLCPP_INFO/WARN 는 output='screen'만으로도 보이지만, estimator.cpp의 printf/cout는 이게 있어야 보임)
        emulate_tty=True,
        # --log-level 로 RCLCPP 로그 레벨 조절 (기본 info; 'debug' 주면 RCLCPP_DEBUG 까지 표시)
        arguments=['--ros-args', '--log-level',
                   ['vins_estimator:=', LaunchConfiguration('log_level')]],
        parameters=[{
            # 카메라 모델도 이 파일에서 읽는다(CAM_NAMES = config_file). vins_folder 는 원본 루트 그대로.
            'config_file': config_file,
            'vins_folder': LaunchConfiguration('vins_folder'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )]


def generate_launch_description():
    # [SW1-1948] 기본 경로는 패키지 share(설치본) 기준. 예전 기본값은 ~/ros2_ws/src/edie9/... 소스 트리라
    # 설치본만 있는 로봇(/opt/edie, root 계정)에서 파일을 못 열어 기동 즉시 abort 했다.
    # share/vins_estimator/config 는 CMake 가 저장소 루트 config/ 를 통째로 설치한 것이고, PC 는 --symlink-install 이라
    # 소스 yaml 을 고치면 그대로 반영된다. 끝에 '/' 필수 — 코드가 vins_folder + "config/..." 로 이어붙인다.
    # [SW1-1959] 그 위에 파일마다 /opt/edie 우선 — /opt/edie/current/share/vins_estimator/ 에 그 파일이 있으면 그것.
    config_default = _share_file('vins_estimator', _CONFIG_REL)
    vins_root_default = _vins_root_of(config_default)

    args = [
        DeclareLaunchArgument(
            'config_file', default_value=config_default,
            description='vio_edie.yaml 절대 경로 (estimator가 cv::FileStorage로 읽음)'),
        DeclareLaunchArgument(
            'vins_folder', default_value=vins_root_default,
            description="config/ 를 품은 루트 경로 (끝에 '/' 포함). 기본 = 기본 config_file 쪽 루트 "
                        "(/opt/edie 에 그 파일이 있으면 /opt, 없으면 패키지 share)"),
        DeclareLaunchArgument(
            'log_level', default_value='info', #'debug',
            description='vins_estimator RCLCPP 로그 레벨 (debug/info/warn/error)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description="bag 재생 시 true (ros2 bag play --clock 과 짝). "
                        "라이브 실행은 false 유지. 필수 — 안 주면 TF lookup이 wall time 으로 가 buffer miss."),
        DeclareLaunchArgument(
            'use_vision_calib', default_value='true',
            description='true: 카메라 fx·fy·cx·cy 를 edie_vision pinhole_left_calib.yaml(정류 P, /opt 우선)로 '
                        '덮어쓴 사본을 쓴다. false: config_file 값 그대로 (SW1-1962)'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_make_estimator)])
