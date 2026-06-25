#!/usr/bin/env python3
# [SW1-1837] edie 라이브 VIO 실행 launch — 실 로봇에서 vins_estimator만 헤드리스로 띄운다.
#   gate1_klt.launch.py와 달리 rqt/bag 자동실행 없음 → SSH 헤드리스 라이브 테스트 친화.
#   config_file/vins_folder는 ~(홈) 기준 기본값이라 사용자·로봇 어디서나 동작(이식성).
#   RViz는 옵션(rviz:=true, 디스플레이 필요).
#
# 사용 예:
#   ros2 launch vins_estimator edie_live.launch.py                 # 헤드리스 라이브
#   ros2 launch vins_estimator edie_live.launch.py rviz:=true      # 시각화 포함
#   ros2 launch vins_estimator edie_live.launch.py config_file:=<다른 yaml>
#   ※ 게이팅 on/off는 config(vio_edie.yaml)의 use_wheel_vel_gate로 토글 후 재실행.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            'rviz', default_value='false',
            description='RViz 실행 여부 (디스플레이 필요, SSH 헤드리스면 false)'),
    ]

    estimator = Node(
        package='vins_estimator',
        executable='vins_estimator_node',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'vins_folder': LaunchConfiguration('vins_folder'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='vins_rviz',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )

    return LaunchDescription(args + [estimator, rviz])
