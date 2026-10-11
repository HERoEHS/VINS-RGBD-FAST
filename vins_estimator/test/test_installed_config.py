# [SW1-1948] 설치본(share)에 VINS 설정이 실제로 들어가고, launch 기본값이 그 파일을 가리키는지 확인하는 회귀 테스트.
#   배경: CMake 가 존재하지 않는 vins_estimator/config/ 를 OPTIONAL 로 설치해 share/config 가 비어 있었고,
#   launch 기본값은 ~/ros2_ws/src/edie9/... 소스 트리를 가정해 설치본만 있는 로봇에서 기동 즉시 abort 했다.
#   colcon test 는 install 뒤에 돌므로 ament_index 로 share 를 직접 본다.
import importlib.util
import inspect
import os

import pytest
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
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


OPT_SHARE = '/opt/edie/current/share/vins_estimator'


@pytest.mark.parametrize('launch_file', LAUNCH_FILES)
def test_launch_defaults_point_to_installed_files(launch_file):
    d = _launch_defaults(launch_file)
    # 기본값은 홈 아래 소스 트리가 아니라 설치본을 가리켜야 한다(설치본만 있는 로봇 가정).
    # [SW1-1959] /opt/edie 에 그 파일이 있으면 그쪽(1순위), 아니면 패키지 share.
    opt_cfg = os.path.join(OPT_SHARE, 'config', 'elp_stereo_edie', 'vio_edie.yaml')
    expected_root = OPT_SHARE if os.path.isfile(opt_cfg) else SHARE
    assert d['config_file'].startswith(expected_root), d['config_file']
    assert os.path.isfile(d['config_file']), d['config_file']
    # 코드가 vins_folder + "config/..." 로 이어붙이므로 끝 '/' 가 필수다.
    assert d['vins_folder'].endswith(os.sep), d['vins_folder']
    assert os.path.isfile(os.path.join(d['vins_folder'], 'config', 'fisheye_mask.jpg'))


# ---- [SW1-1959] launch 의 _share_file: 파일마다 /opt/edie 우선 → ament share → 1순위 후보 ----
# 실제 /opt·ament 상태에 의존하지 않도록 루트와 ament 조회를 임시 폴더로 바꿔 각 분기를 고정한다.

def _load_module(launch_file):
    path = os.path.join(SHARE, 'launch', 'edie', launch_file)
    mod_name = 'launch_mod_' + launch_file.replace('.', '_')
    spec = importlib.util.spec_from_file_location(mod_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _make(path, text='a: 1'):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


@pytest.fixture(params=LAUNCH_FILES)
def mod(request):
    return _load_module(request.param)


def test_share_file_opt_wins_over_dev(mod, monkeypatch, tmp_path):
    opt = _make(tmp_path / 'opt' / 'pkg' / 'config' / 'x.yaml')
    _make(tmp_path / 'dev' / 'pkg' / 'config' / 'x.yaml', 'a: 2')
    monkeypatch.setattr(mod, '_OPT_SHARE_ROOT', str(tmp_path / 'opt'))
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda p: str(tmp_path / 'dev' / p))
    assert mod._share_file('pkg', 'config/x.yaml') == str(opt)


def test_share_file_opt_dir_without_file_falls_back(mod, monkeypatch, tmp_path):
    (tmp_path / 'opt' / 'pkg' / 'config').mkdir(parents=True)  # 폴더만 있고 파일은 없음
    dev = _make(tmp_path / 'dev' / 'pkg' / 'config' / 'x.yaml')
    monkeypatch.setattr(mod, '_OPT_SHARE_ROOT', str(tmp_path / 'opt'))
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda p: str(tmp_path / 'dev' / p))
    assert mod._share_file('pkg', 'config/x.yaml') == str(dev)


def test_share_file_nothing_returns_opt_candidate(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, '_OPT_SHARE_ROOT', str(tmp_path / 'opt'))
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda p: str(tmp_path / 'dev' / p))
    assert mod._share_file('pkg', 'config/x.yaml') == os.path.join(
        str(tmp_path / 'opt'), 'pkg', 'config/x.yaml')


@pytest.mark.parametrize('exc', [PackageNotFoundError, ValueError, OSError])
def test_share_file_ament_error_returns_opt_candidate(mod, monkeypatch, tmp_path, exc):
    def _raise(pkg):
        raise exc(pkg)

    monkeypatch.setattr(mod, '_OPT_SHARE_ROOT', str(tmp_path / 'opt'))
    monkeypatch.setattr(mod, 'get_package_share_directory', _raise)
    assert mod._share_file('pkg', 'config/x.yaml') == os.path.join(
        str(tmp_path / 'opt'), 'pkg', 'config/x.yaml')


def test_vins_root_of_strips_config_rel(mod):
    assert mod._vins_root_of(os.path.join('/r', mod._CONFIG_REL)) == '/r' + os.sep


def _defaults_of(module):
    """이미 불러온 launch 모듈의 경로 인자 기본값(문자열)."""
    context = LaunchContext()
    return {e.name: perform_substitutions(context, e.default_value)
            for e in module.generate_launch_description().entities
            if isinstance(e, DeclareLaunchArgument) and e.name in PATH_ARGS}


def test_launch_defaults_use_opt_when_present(mod, monkeypatch, tmp_path):
    # 연결부: launch 기본값이 실제로 _share_file 을 거쳐 /opt 사본을 고르고, vins_folder 도 그 쪽 루트여야 한다.
    # (이 PC 에는 /opt 사본이 없어 위 test_launch_defaults_* 만으로는 이 경로가 안 돈다 — critic R1)
    opt_root = tmp_path / 'opt'
    cfg = _make(opt_root / 'vins_estimator' / mod._CONFIG_REL)
    monkeypatch.setattr(mod, '_OPT_SHARE_ROOT', str(opt_root))
    d = _defaults_of(mod)
    assert d['config_file'] == str(cfg)
    assert d['vins_folder'] == os.path.join(str(opt_root / 'vins_estimator'), '')


def test_launch_defaults_use_share_when_opt_empty(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, '_OPT_SHARE_ROOT', str(tmp_path / 'opt'))
    d = _defaults_of(mod)
    assert d['config_file'] == os.path.join(SHARE, mod._CONFIG_REL)
    assert d['vins_folder'] == os.path.join(SHARE, '')


def test_helper_copies_are_identical():
    mods = [_load_module(f) for f in LAUNCH_FILES]
    for name in ('_share_file', '_vins_root_of'):
        assert len({inspect.getsource(getattr(m, name)) for m in mods}) == 1, name
    assert {m._OPT_SHARE_ROOT for m in mods} == {'/opt/edie/current/share'}
    rel = os.path.join('config', 'elp_stereo_edie', 'vio_edie.yaml')
    assert {m._CONFIG_REL for m in mods} == {rel}
