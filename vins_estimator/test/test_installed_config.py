# [SW1-1948] 설치본(share)에 VINS 설정이 실제로 들어가고, launch 기본값이 그 파일을 가리키는지 확인하는 회귀 테스트.
#   배경: CMake 가 존재하지 않는 vins_estimator/config/ 를 OPTIONAL 로 설치해 share/config 가 비어 있었고,
#   launch 기본값은 ~/ros2_ws/src/edie9/... 소스 트리를 가정해 설치본만 있는 로봇에서 기동 즉시 abort 했다.
#   colcon test 는 install 뒤에 돌므로 ament_index 로 share 를 직접 본다.
import importlib.util
import os

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

SHARE = get_package_share_directory('vins_estimator')
LAUNCH_FILES = ['edie_vslam.launch.py', 'gate1_klt.launch.py']


def test_config_installed_in_share():
    # 저장소 루트 config/ 가 통째로 share/vins_estimator/config 에 들어가야 한다.
    assert os.path.isfile(os.path.join(SHARE, 'config', 'elp_stereo_edie', 'vio_edie.yaml'))
    # fisheye:1 일 때 코드가 vins_folder + "config/fisheye_mask.jpg" 를 읽는다 — 같이 설치돼야 한다.
    assert os.path.isfile(os.path.join(SHARE, 'config', 'fisheye_mask.jpg'))


PATH_ARGS = ('config_file', 'vins_folder')


def _launch_defaults(launch_file):
    """launch 파일을 import 해 경로 인자 두 개의 기본값(문자열)을 모은다. 노드 실행은 하지 않는다."""
    path = os.path.join(SHARE, 'launch', 'edie', launch_file)
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    defaults = {}
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name in PATH_ARGS:
            defaults[entity.name] = perform_substitutions(context, entity.default_value)
    assert set(defaults) == set(PATH_ARGS), f'{launch_file}: 경로 인자 누락 {defaults.keys()}'
    return defaults


@pytest.mark.parametrize('launch_file', LAUNCH_FILES)
def test_launch_defaults_point_to_installed_files(launch_file):
    d = _launch_defaults(launch_file)
    # 기본값은 홈 아래 소스 트리가 아니라 패키지 share 를 가리켜야 한다(설치본만 있는 로봇 가정).
    assert d['config_file'].startswith(SHARE), d['config_file']
    assert os.path.isfile(d['config_file']), d['config_file']
    # 코드가 vins_folder + "config/..." 로 이어붙이므로 끝 '/' 가 필수다.
    assert d['vins_folder'].endswith(os.sep), d['vins_folder']
    assert os.path.isfile(os.path.join(d['vins_folder'], 'config', 'fisheye_mask.jpg'))
