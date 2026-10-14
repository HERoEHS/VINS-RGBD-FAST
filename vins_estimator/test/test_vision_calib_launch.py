# [SW1-1962] edie_vslam.launch.py 가 edie_vision 캘리브 파일 경로를 VINS 의 camera_calib_file 로 넘기는지 확인.
# 값 읽기는 VINS(C++)가 한다 — 여기서는 "어느 파일을 넘기나"만 본다. 노드는 띄우지 않는다.
import importlib.util
import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('vins_estimator')
CONFIG = os.path.join(SHARE, 'config', 'elp_stereo_edie', 'vio_edie.yaml')

P_LEFT = [401.5, 0.0, 330.25, 0.0, 0.0, 402.5, 260.75, 0.0, 0.0, 0.0, 1.0, 0.0]


@pytest.fixture
def mod():
    path = os.path.join(SHARE, 'launch', 'edie', 'edie_vslam.launch.py')
    spec = importlib.util.spec_from_file_location('edie_vslam_under_test', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _make_vision(root, use_undistort=True, calib_name='pinhole_left_calib.yaml', make_calib=True):
    """가짜 /opt/edie/current/share/edie_vision 캘리브 (camera_v4l2.yaml 이 캘리브 이름을 가리킨다)."""
    d = root / 'edie_vision' / 'config' / 'gstreamer'
    d.mkdir(parents=True)
    (d / 'camera_v4l2.yaml').write_text(
        f'left:\n  calibration_file: {calib_name}\n  use_undistort: {str(use_undistort).lower()}\n')
    if make_calib:
        (d / calib_name).write_text(f'projection_matrix:\n  rows: 3\n  cols: 4\n  data: {P_LEFT}\n')
    return d


def _use_opt(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, '_OPT_SHARE_ROOT', str(tmp_path / 'opt'))


def test_calib_file_from_opt(mod, monkeypatch, tmp_path):
    d = _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    assert mod._vision_calib_file() == (str(d / 'pinhole_left_calib.yaml'), None)


def test_calibration_file_follows_camera_yaml(mod, monkeypatch, tmp_path):
    # vision 과 같은 규칙: camera_v4l2.yaml 옆의 left.calibration_file 을 읽는다(critic R1).
    d = _make_vision(tmp_path / 'opt', calib_name='robot_220_left.yaml')
    _use_opt(mod, monkeypatch, tmp_path)
    assert mod._vision_calib_file()[0] == str(d / 'robot_220_left.yaml')


def test_unrectified_camera_gives_no_calib(mod, monkeypatch, tmp_path):
    # 원본(정류 전) 영상이면 P 를 쓰면 안 된다 → 넘기지 않음(VINS 는 vio_edie.yaml 값).
    _make_vision(tmp_path / 'opt', use_undistort=False)
    _use_opt(mod, monkeypatch, tmp_path)
    path, why = mod._vision_calib_file()
    assert path is None and 'use_undistort' in why


def test_missing_calib_file_gives_no_calib(mod, monkeypatch, tmp_path):
    _make_vision(tmp_path / 'opt', make_calib=False)
    _use_opt(mod, monkeypatch, tmp_path)
    path, why = mod._vision_calib_file()
    assert path is None and 'not found' in why


def _node_params(mod, use_vision_calib):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'config_file': CONFIG, 'vins_folder': os.path.join(SHARE, ''), 'log_level': 'info',
        'use_sim_time': 'false', 'use_vision_calib': use_vision_calib})
    (node,) = mod._make_estimator(ctx)
    assert isinstance(node, Node)
    flat = {}
    for d in node._Node__parameters:  # launch_ros Node 는 parameters 를 공개 프로퍼티로 내주지 않는다
        for k, v in d.items():
            # 문자열 값은 launch_ros 가 YAML 문자열로 정규화해 TextSubstitution 에 담는다 → 다시 푼다.
            flat[perform_substitutions(ctx, list(k))] = yaml.safe_load(perform_substitutions(ctx, list(v)))
    return flat


def test_node_gets_calib_path_and_original_config(mod, monkeypatch, tmp_path):
    # 연결부: 노드는 원본 설정 + 캘리브 경로를 받는다(사본·/tmp 없음).
    d = _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    p = _node_params(mod, 'true')
    assert p['camera_calib_file'] == str(d / 'pinhole_left_calib.yaml')
    assert p['config_file'] == CONFIG


def test_node_gets_empty_calib_when_disabled(mod, monkeypatch, tmp_path):
    _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    p = _node_params(mod, 'false')
    # 빈 문자열은 YAML 로 풀면 None — 어느 쪽이든 "경로 없음"
    assert p['camera_calib_file'] in ('', None)
    assert p['config_file'] == CONFIG


def test_no_tmp_copy_code_left(mod):
    # C안(사용자 10-14): 설정 사본을 /tmp 에 만들던 코드는 없어야 한다.
    assert not hasattr(mod, '_write_patched_copy')
    assert not hasattr(mod, '_vision_calibrated_config')
