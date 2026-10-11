#!/usr/bin/env python3
# [SW1-1837] edie 라이브 VIO 실행 launch — 실 로봇에서 vins_estimator만 헤드리스로 띄운다.
#   gate1_klt.launch.py와 달리 rqt/bag 자동실행 없음 → SSH 헤드리스 라이브 테스트 친화.
#   config_file/vins_folder 기본값은 패키지 share(설치본) 기준 — 소스 체크아웃이 없는 로봇에서도 그대로 뜬다(SW1-1948).
#   시각화가 필요하면 별도 쉘에서 rviz2 직접 실행 (bag 재생 시: rviz2 --ros-args -p use_sim_time:=true).
#
# 사용 예:
#   ros2 launch vins_estimator edie_vslam.launch.py                # 헤드리스 라이브 (기본 = 패키지 share 의 vio_edie.yaml)
#   ros2 launch vins_estimator edie_vslam.launch.py config_file:=<다른 yaml>
#   ※ 게이팅 on/off는 config(vio_edie.yaml)의 use_wheel_vel_gate로 토글 후 재실행.
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    # <루트>/config/elp_stereo_edie/vio_edie.yaml → <루트>/ (코드가 vins_folder + "config/..." 로 이어붙임)
    return os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(config_path))), '')


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
            description="config/ 를 품은 루트 경로 (끝에 '/' 포함). 기본 = 패키지 share"),
        DeclareLaunchArgument(
            'log_level', default_value='info', #'debug',
            description='vins_estimator RCLCPP 로그 레벨 (debug/info/warn/error)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description="bag 재생 시 true (ros2 bag play --clock 과 짝). "
                        "라이브 실행은 false 유지. 필수 — 안 주면 TF lookup이 wall time 으로 가 buffer miss."),
    ]

    estimator = Node(
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
            'config_file': LaunchConfiguration('config_file'),
            'vins_folder': LaunchConfiguration('vins_folder'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(args + [estimator])
