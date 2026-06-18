#!/usr/bin/env python3
# =============================================================================
# VIO Phase 1 (SW1-1828) 게이트① — KLT 추적 생존율 측정용 launch
#
#   - VINS-RGBD-FAST의 estimator(`vins_estimator_node`)를 EDIE config로 구동한다.
#   - estimator는 feature_tracker(KLT)를 내장하며, config의 show_track:1 이면
#     추적 결과 이미지를 `/vins_estimator/feature_img` 로 발행한다.
#   - 그 토픽을 rqt_image_view로 띄워 코너 수/생존율/분포를 정성 확인한다.
#
# 사용 예:
#   # 1) 노드 + 뷰어만 띄우고, bag은 다른 터미널에서 직접 play
#   ros2 launch vins_estimator gate1_klt.launch.py
#   # 2) bag 경로를 주면 자동 재생
#   ros2 launch vins_estimator gate1_klt.launch.py bag:=/경로/to/rosbag2_dir rate:=1.0
#
# 주의: estimator는 color+depth 타임스탬프가 ±3ms 이내여야 프레임을 처리한다.
#       즉 bag에 image_gray 와 depth 가 (거의) 같은 스탬프로 들어 있어야 한다.
#       depth가 없으면 KLT 추적 스레드가 영원히 대기하여 한 프레임도 처리되지 않는다.
# =============================================================================

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # --- 경로 기본값 ---------------------------------------------------------
    # VINS-RGBD-FAST 소스 루트. 끝에 '/' 필수:
    # 코드(parameters.cpp)가 vins_folder + "config/..." 형태로 문자열을 이어붙이기 때문.
    vins_root_default = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'edie9', 'edie_localization', 'VINS-RGBD-FAST', '')
    config_default = os.path.join(
        vins_root_default, 'config', 'elp_stereo_edie', 'vio_edie.yaml')

    # --- launch 인자 ---------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'config_file', default_value=config_default,
            description='vio_edie.yaml 절대 경로 (estimator가 cv::FileStorage로 읽음)'),
        DeclareLaunchArgument(
            'vins_folder', default_value=vins_root_default,
            description="VINS-RGBD-FAST 루트 경로 (끝에 '/' 포함)"),
        DeclareLaunchArgument(
            'bag', default_value='',
            description='재생할 ros2 bag 디렉토리. 비우면 자동 재생 안 함(직접 play).'),
        DeclareLaunchArgument(
            'rate', default_value='1.0', description='bag 재생 배속'),
        DeclareLaunchArgument(
            'rqt', default_value='true', description='rqt_image_view 자동 실행 여부'),
    ]

    config_file = LaunchConfiguration('config_file')
    vins_folder = LaunchConfiguration('vins_folder')
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')

    # --- estimator (KLT feature_tracker 내장) --------------------------------
    estimator = Node(
        package='vins_estimator',
        executable='vins_estimator_node',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'vins_folder': vins_folder,
        }],
    )

    # --- 추적 이미지 뷰어 ----------------------------------------------------
    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='klt_track_view',
        arguments=['/vins_estimator/feature_img'],
        condition=IfCondition(LaunchConfiguration('rqt')),
        output='log',
    )

    # --- (선택) bag 자동 재생: bag 인자가 비어있지 않을 때만 ------------------
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--rate', rate],
        output='screen',
        condition=IfCondition(PythonExpression(["'", bag, "' != ''"])),
    )

    return LaunchDescription(args + [estimator, rqt, bag_play])
