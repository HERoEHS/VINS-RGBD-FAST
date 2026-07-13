#!/usr/bin/env python3
# [SW1-1837] edie 라이브 VIO 실행 launch — 실 로봇에서 vins_estimator만 헤드리스로 띄운다.
#   gate1_klt.launch.py와 달리 rqt/bag 자동실행 없음 → SSH 헤드리스 라이브 테스트 친화.
#   config_file/vins_folder는 ~(홈) 기준 기본값이라 사용자·로봇 어디서나 동작(이식성).
#   시각화가 필요하면 별도 쉘에서 rviz2 직접 실행 (bag 재생 시: rviz2 --ros-args -p use_sim_time:=true).
#
# 사용 예:
#   ros2 launch vins_estimator edie_live.launch.py                 # 헤드리스 라이브
#   ros2 launch vins_estimator edie_live.launch.py config_file:=<다른 yaml>
#   ※ 게이팅 on/off는 config(vio_edie.yaml)의 use_wheel_vel_gate로 토글 후 재실행.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # VINS-RGBD-FAST 소스 루트(끝에 '/' 필수 — 코드가 vins_folder + "config/..." 로 이어붙임).
    vins_root_default = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'edie9', 'edie_localization', 'VINS-RGBD-FAST', '')
    config_default = os.path.join(
        vins_root_default, 'config', 'elp_stereo_edie', 'vio_edie.yaml')

    args = [
        DeclareLaunchArgument(
            'config_file', default_value=config_default,
            description='vio_edie.yaml 절대 경로 (estimator가 cv::FileStorage로 읽음)'),
        DeclareLaunchArgument(
            'vins_folder', default_value=vins_root_default,
            description="VINS-RGBD-FAST 루트 경로 (끝에 '/' 포함)"),
        DeclareLaunchArgument(
            'log_level', default_value='debug',
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
