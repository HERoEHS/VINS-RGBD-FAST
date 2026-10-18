# [SW1-1962] edie_vslam.launch.py 가 edie_vision 캘리브 파일 경로를 VINS 의 camera_calib_file 로 넘기는지 확인.
# [SW1-1959] + 로봇 설정 폴더(camera/·vins/extrinsic.yaml) 먼저 찾기·extrinsic_file 넘기기.
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


@pytest.fixture(autouse=True)
def robot_home(monkeypatch, tmp_path):
    """[SW1-1959] 로봇 설정 폴더를 임시 HOME 아래로 — 실제 ~/.config/edie·/var/lib/edie 를 건드리지 않게.
    실행 계정에 상관없이 같은 분기를 타도록 uid 도 일반 계정으로 고정한다(root 분기는 따로 가짜 루트로 시험)."""
    home = tmp_path / 'home'
    home.mkdir()
    monkeypatch.setenv('HOME', str(home))
    monkeypatch.setattr(os, 'geteuid', lambda: 1000)
    return home / '.config' / 'edie'


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


# ---- [SW1-1959] 로봇 설정 폴더 camera/ 먼저 ----

def _make_robot_calib(robot_dir, name='pinhole_left_calib.yaml'):
    d = robot_dir / 'camera'
    d.mkdir(parents=True, exist_ok=True)
    (d / name).write_text(f'projection_matrix:\n  rows: 3\n  cols: 4\n  data: {P_LEFT}\n')
    return d / name


def test_robot_calib_wins_over_image(mod, monkeypatch, tmp_path, robot_home):
    _make_vision(tmp_path / 'opt')
    robot = _make_robot_calib(robot_home)
    _use_opt(mod, monkeypatch, tmp_path)
    assert mod._vision_calib_file() == (str(robot), None)


def test_robot_calib_uses_name_from_camera_yaml(mod, monkeypatch, tmp_path, robot_home):
    # 이름은 camera_v4l2.yaml(/opt 우선)의 left.calibration_file — 다른 이름의 로봇 파일은 안 쓴다.
    d = _make_vision(tmp_path / 'opt', calib_name='robot_220_left.yaml')
    _make_robot_calib(robot_home)  # pinhole_left_calib.yaml — 이름이 다르다
    _use_opt(mod, monkeypatch, tmp_path)
    assert mod._vision_calib_file()[0] == str(d / 'robot_220_left.yaml')
    robot = _make_robot_calib(robot_home, 'robot_220_left.yaml')
    assert mod._vision_calib_file()[0] == str(robot)


def test_missing_robot_calib_falls_back_and_creates_folder(mod, monkeypatch, tmp_path, robot_home):
    d = _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    assert mod._vision_calib_file() == (str(d / 'pinhole_left_calib.yaml'), None)
    # 빈 폴더만 만든다 — 기본값을 복사해 넣지 않는다(남의 값이 이 로봇 캘리브인 척하지 않게).
    assert (robot_home / 'camera').is_dir()
    assert list((robot_home / 'camera').iterdir()) == []


def test_unrectified_ignores_robot_calib(mod, monkeypatch, tmp_path, robot_home):
    _make_vision(tmp_path / 'opt', use_undistort=False)
    _make_robot_calib(robot_home)
    _use_opt(mod, monkeypatch, tmp_path)
    assert mod._vision_calib_file()[0] is None


def test_robot_config_dir_home_empty(mod, monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', '')
    assert mod._robot_config_dir() == ''
    path, looked = mod._robot_config_file('vins', 'extrinsic.yaml')
    assert path is None and 'HOME empty' in looked
    # 그래도 이미지 기본값으로 뜬다(시작 거부 없음).
    d = _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    assert mod._vision_calib_file() == (str(d / 'pinhole_left_calib.yaml'), None)


def test_robot_config_dir_root_uses_var_lib(mod, monkeypatch, tmp_path):
    # root(edieOS) 분기 — 진짜 /var/lib/edie 대신 가짜 루트. 없으면 만든다.
    fake = tmp_path / 'var_lib_edie'
    monkeypatch.setattr(mod, '_SYSTEM_STATE_ROOT', str(fake))
    monkeypatch.setattr(os, 'geteuid', lambda: 0)
    assert mod._robot_config_dir() == str(fake / 'config')
    assert fake.is_dir()


def test_robot_config_dir_root_unwritable_falls_back_to_home(mod, monkeypatch, tmp_path, robot_home):
    if os.getuid() == 0:
        pytest.skip('root 는 권한 없는 폴더도 쓸 수 있어 이 분기를 만들 수 없다')
    fake = tmp_path / 'ro'
    fake.mkdir()
    fake.chmod(0o500)
    try:
        monkeypatch.setattr(mod, '_SYSTEM_STATE_ROOT', str(fake))
        monkeypatch.setattr(os, 'geteuid', lambda: 0)
        assert mod._robot_config_dir() == str(robot_home)
    finally:
        fake.chmod(0o700)


def test_robot_config_file_folder_create_failure_is_ignored(mod, monkeypatch, tmp_path):
    # HOME 이 파일이라 폴더를 못 만듦 → 예외 없이 (None, 찾아본 경로)
    home_file = tmp_path / 'home_is_file'
    home_file.write_text('x')
    monkeypatch.setenv('HOME', str(home_file))
    path, looked = mod._robot_config_file('camera', 'a.yaml')
    assert path is None and looked.endswith(os.path.join('camera', 'a.yaml'))


def test_helpers_match_spec_copy(mod):
    # 규칙서(SPEC.md) 사본과 같은 모양 — 다른 저장소 사본과 어긋나지 않게 글자 그대로 둔다.
    import inspect
    src = inspect.getsource(mod._robot_config_dir) + inspect.getsource(mod._robot_config_file)
    assert "os.path.join(home, '.config', 'edie') if home else ''" in src
    assert "return (path, path) if os.path.isfile(path) else (None, path)" in src
    assert mod._SYSTEM_STATE_ROOT == '/var/lib/edie'


def _make_robot_extrinsic(robot_dir):
    d = robot_dir / 'vins'
    d.mkdir(parents=True, exist_ok=True)
    (d / 'extrinsic.yaml').write_text('%YAML:1.0\n')
    return d / 'extrinsic.yaml'


def _node_params(mod, use_vision_calib, use_robot_extrinsic='true'):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'config_file': CONFIG, 'vins_folder': os.path.join(SHARE, ''), 'log_level': 'info',
        'use_sim_time': 'false', 'use_vision_calib': use_vision_calib,
        'use_robot_extrinsic': use_robot_extrinsic})
    (node,) = mod._make_estimator(ctx)
    assert isinstance(node, Node)
    flat = {}
    for d in node._Node__parameters:  # launch_ros Node 는 parameters 를 공개 프로퍼티로 내주지 않는다
        for k, v in d.items():
            # 문자열 값은 launch_ros 가 YAML 문자열로 정규화해 TextSubstitution 에 담는다 → 다시 푼다.
            # bool 같은 값은 그대로 담긴다.
            flat[perform_substitutions(ctx, list(k))] = (
                v if isinstance(v, bool) else yaml.safe_load(perform_substitutions(ctx, list(v))))
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


def test_node_marks_robot_calib_source(mod, monkeypatch, tmp_path, robot_home):
    # launch 는 파일이 있는지만 본다 — 내용이 틀렸어도 넘기고, 로봇 폴더 파일이라고 표시해 VINS 가 ERROR 로 남기게 한다.
    _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    d = robot_home / 'camera'
    d.mkdir(parents=True)
    (d / 'pinhole_left_calib.yaml').write_text('not: a calib\n')
    p = _node_params(mod, 'true')
    assert p['camera_calib_file'] == str(d / 'pinhole_left_calib.yaml')
    assert p['camera_calib_from_robot'] is True


def test_node_marks_image_default_calib_source(mod, monkeypatch, tmp_path, robot_home):
    d = _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    p = _node_params(mod, 'true')
    assert p['camera_calib_file'] == str(d / 'pinhole_left_calib.yaml')
    assert p['camera_calib_from_robot'] is False
    assert _node_params(mod, 'false')['camera_calib_from_robot'] is False


def test_node_passes_invalid_robot_extrinsic_through(mod, monkeypatch, tmp_path, robot_home):
    # 형식 검사는 VINS(C++) 몫 — 틀린 파일도 경로는 넘기고, VINS 가 ERROR 후 config 값으로 간다.
    _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    d = robot_home / 'vins'
    d.mkdir(parents=True)
    (d / 'extrinsic.yaml').write_text('garbage')
    assert _node_params(mod, 'true')['extrinsic_file'] == str(d / 'extrinsic.yaml')


def test_node_gets_robot_extrinsic(mod, monkeypatch, tmp_path, robot_home):
    _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    ex = _make_robot_extrinsic(robot_home)
    p = _node_params(mod, 'true')
    assert p['extrinsic_file'] == str(ex)
    assert p['config_file'] == CONFIG  # 본체는 그대로, 덮어쓰기는 VINS(C++)가 한다


def test_node_gets_empty_extrinsic_when_missing(mod, monkeypatch, tmp_path, robot_home):
    _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    p = _node_params(mod, 'true')
    assert p['extrinsic_file'] in ('', None)
    assert (robot_home / 'vins').is_dir()  # 빈 자리만 만든다
    assert list((robot_home / 'vins').iterdir()) == []


def test_node_gets_empty_extrinsic_when_disabled(mod, monkeypatch, tmp_path, robot_home):
    _make_vision(tmp_path / 'opt')
    _use_opt(mod, monkeypatch, tmp_path)
    _make_robot_extrinsic(robot_home)
    p = _node_params(mod, 'true', use_robot_extrinsic='false')
    assert p['extrinsic_file'] in ('', None)


def test_no_tmp_copy_code_left(mod):
    # C안(사용자 10-14): 설정 사본을 /tmp 에 만들던 코드는 없어야 한다.
    assert not hasattr(mod, '_write_patched_copy')
    assert not hasattr(mod, '_vision_calibrated_config')
