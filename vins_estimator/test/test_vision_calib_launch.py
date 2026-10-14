# [SW1-1962] edie_vslam.launch.py 가 시작할 때 vio_edie.yaml 카메라 값(fx·fy·cx·cy)을
# edie_vision 캘리브(정류 P)로 덮어쓴 사본을 넘기는지 확인. 노드는 띄우지 않는다.
import importlib.util
import os
import re

import pytest
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


def _make_vision(root, use_undistort=True, p=P_LEFT, calib_name='pinhole_left_calib.yaml'):
    """가짜 /opt/edie/current/share/edie_vision 캘리브 두 파일 (camera_v4l2.yaml 이 캘리브 이름을 가리킨다)."""
    d = root / 'edie_vision' / 'config' / 'gstreamer'
    d.mkdir(parents=True)
    (d / 'camera_v4l2.yaml').write_text(
        f'left:\n  calibration_file: {calib_name}\n  use_undistort: {str(use_undistort).lower()}\n')
    (d / calib_name).write_text(f'projection_matrix:\n  rows: 3\n  cols: 4\n  data: {p}\n')


def _use(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, '_OPT_SHARE_ROOT', str(tmp_path / 'opt'))
    monkeypatch.setattr(mod, '_PATCHED_ROOT', str(tmp_path / 'patched'))


def _proj_values(text):
    vals, inside = {}, False
    for ln in text.splitlines():
        if ln.rstrip() == 'projection_parameters:':
            inside = True
            continue
        if inside and ln.strip() and not ln[0].isspace():
            break
        if inside and ':' in ln:
            k, v = ln.strip().split(':', 1)
            vals[k] = float(v)
    return vals


def test_patch_replaces_only_four_projection_lines(mod):
    text = open(CONFIG).read()
    out = mod._with_vision_intrinsics(text, (1.0, 2.0, 3.0, 4.0))
    assert _proj_values(out) == {'fx': 1.0, 'fy': 2.0, 'cx': 3.0, 'cy': 4.0}
    changed = [(a, b) for a, b in zip(text.splitlines(), out.splitlines()) if a != b]
    assert len(changed) == 4
    assert len(text.splitlines()) == len(out.splitlines())


def test_patch_rejects_missing_block(mod):
    with pytest.raises(ValueError):
        mod._with_vision_intrinsics('model_type: PINHOLE\nfx: 1\n', (1.0, 2.0, 3.0, 4.0))
    with pytest.raises(ValueError):
        mod._with_vision_intrinsics('projection_parameters:\n   fx: 1\n   fy: 1\n', (1.0, 2.0, 3.0, 4.0))


def test_config_uses_vision_calibration(mod, monkeypatch, tmp_path):
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)
    out = mod._vision_calibrated_config(CONFIG)
    assert os.path.dirname(out) == str(tmp_path / 'patched' / f'edie_vins-{os.getuid()}')
    assert re.fullmatch(r'vio_edie\.[0-9a-f]{8}\.yaml', os.path.basename(out))
    assert _proj_values(open(out).read()) == {'fx': 401.5, 'fy': 402.5, 'cx': 330.25, 'cy': 260.75}
    # 원본은 손대지 않는다
    assert _proj_values(open(CONFIG).read())['fx'] != 401.5


def test_unrectified_camera_keeps_original(mod, monkeypatch, tmp_path):
    # 원본(정류 전) 영상이면 P 를 넣으면 안 된다 → 원본 그대로.
    _make_vision(tmp_path / 'opt', use_undistort=False)
    _use(mod, monkeypatch, tmp_path)
    assert mod._vision_calibrated_config(CONFIG) == CONFIG
    assert not (tmp_path / 'patched').exists()


def test_broken_calibration_keeps_original(mod, monkeypatch, tmp_path):
    _make_vision(tmp_path / 'opt', p=[1.0, 2.0])
    _use(mod, monkeypatch, tmp_path)
    assert mod._vision_calibrated_config(CONFIG) == CONFIG


def test_patched_file_readable_by_opencv(mod, monkeypatch, tmp_path):
    # VINS 는 cv::FileStorage 로 읽는다 — 덮어쓴 사본도 같은 방식으로 읽혀야 한다.
    cv2 = pytest.importorskip('cv2')
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)
    fs = cv2.FileStorage(mod._vision_calibrated_config(CONFIG), cv2.FILE_STORAGE_READ)
    node = fs.getNode('projection_parameters')
    assert node.getNode('fx').real() == pytest.approx(401.5)
    assert node.getNode('cy').real() == pytest.approx(260.75)
    assert fs.getNode('image_width').real() == 640


def _copies(tmp_path):
    return sorted(os.listdir(tmp_path / 'patched' / f'edie_vins-{os.getuid()}'))


def test_same_config_reuses_one_copy(mod, monkeypatch, tmp_path):
    # 실행할 때마다 파일이 쌓이지 않는다: 같은 설정이면 같은 파일 하나(임시 파일도 안 남음).
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)
    first = mod._vision_calibrated_config(CONFIG)
    assert all(mod._vision_calibrated_config(CONFIG) == first for _ in range(3))
    assert _copies(tmp_path) == [os.path.basename(first)]


def test_different_config_gets_different_copy(mod, monkeypatch, tmp_path):
    # 설정이 다르면 이름이 달라 서로 덮어쓰지 않는다(critic M1 — 동시 실행).
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)
    a = mod._vision_calibrated_config(CONFIG)
    other = tmp_path / 'other' / 'vio_edie.yaml'
    other.parent.mkdir()
    other.write_text(open(CONFIG).read() + '\n# 다른 설정\n')
    b = mod._vision_calibrated_config(str(other))
    assert a != b
    assert len(_copies(tmp_path)) == 2
    assert open(b).read().endswith('# 다른 설정\n')


def test_stale_copy_with_same_name_is_rewritten(mod, monkeypatch, tmp_path):
    # 같은 이름 파일이 깨져 있으면(내용 불일치) 다시 쓴다.
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)
    out = mod._vision_calibrated_config(CONFIG)
    with open(out, 'w') as f:
        f.write('broken')
    assert mod._vision_calibrated_config(CONFIG) == out
    assert _proj_values(open(out).read())['fx'] == 401.5


def test_calibration_file_follows_camera_yaml(mod, monkeypatch, tmp_path):
    # vision 과 같은 규칙: camera_v4l2.yaml 옆의 left.calibration_file 을 읽는다(critic R1).
    _make_vision(tmp_path / 'opt', calib_name='robot_220_left.yaml')
    _use(mod, monkeypatch, tmp_path)
    k, src = mod._rect_intrinsics()
    assert k == pytest.approx((401.5, 402.5, 330.25, 260.75))
    assert src.endswith('robot_220_left.yaml')


def _launch_node_params(mod, use_vision_calib):
    from launch import LaunchContext
    from launch_ros.actions import Node
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'config_file': CONFIG, 'vins_folder': os.path.join(SHARE, ''), 'log_level': 'info',
        'use_sim_time': 'false', 'use_vision_calib': use_vision_calib})
    (node,) = mod._make_estimator(ctx)
    assert isinstance(node, Node)
    import yaml
    from launch.utilities import perform_substitutions
    flat = {}
    for d in node._Node__parameters:  # launch_ros Node 는 parameters 를 공개 프로퍼티로 내주지 않는다
        for k, v in d.items():
            # 문자열 값은 launch_ros 가 YAML 문자열로 정규화해 TextSubstitution 에 담는다 → 다시 푼다.
            flat[perform_substitutions(ctx, list(k))] = yaml.safe_load(perform_substitutions(ctx, list(v)))
    return flat


def test_node_gets_patched_config(mod, monkeypatch, tmp_path):
    # 연결부: OpaqueFunction 이 실제로 덮은 사본 경로를 노드 config_file 로 넘긴다(critic R7).
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)
    cfg = _launch_node_params(mod, 'true')['config_file']
    assert cfg.startswith(str(tmp_path / 'patched'))
    assert _proj_values(open(cfg).read())['fx'] == 401.5


def test_node_gets_original_when_disabled(mod, monkeypatch, tmp_path):
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)
    assert _launch_node_params(mod, 'false')['config_file'] == CONFIG


def test_write_failure_cleans_temp_and_keeps_original(mod, monkeypatch, tmp_path):
    # 바꿔 넣기(os.replace)가 실패하면 반쯤 쓴 임시 파일을 지우고 원본 경로로 돌아간다(critic R2).
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)

    def boom(*_a, **_k):
        raise OSError('replace 실패 흉내')
    monkeypatch.setattr(mod.os, 'replace', boom)
    assert mod._vision_calibrated_config(CONFIG) == CONFIG
    assert _copies(tmp_path) == []


def test_foreign_owned_dir_keeps_original(mod, monkeypatch, tmp_path):
    # 다른 계정이 만든 폴더면 쓰지 않고 원본 경로로 돌아간다(critic R4).
    _make_vision(tmp_path / 'opt')
    _use(mod, monkeypatch, tmp_path)
    real_lstat = os.lstat

    class _Foreign:
        def __init__(self, st):
            self.st = st
            self.st_uid = st.st_uid + 1

    monkeypatch.setattr(mod.os, 'lstat', lambda p: _Foreign(real_lstat(p)))
    assert mod._vision_calibrated_config(CONFIG) == CONFIG
