# /joint_states 에 몸통 가상 관절(body_lift/body_pitch)만 담은 메시지가 끼어들어도
# 다리각 추출이 깨지지 않는지 확인 (bag_acceptance_check.py · eval/bias_separation.py 공통).
# 실행: colcon test --packages-select vins_estimator (vins_estimator/CMakeLists.txt 에 등록됨)
#       또는 손으로: source /opt/ros/humble/setup.bash && python3 -m pytest scripts/tests
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'eval'))

import bag_acceptance_check  # noqa: E402
import bias_separation  # noqa: E402

FUNCS = [bag_acceptance_check.leg_abs_max, bias_separation.leg_abs_max]

NORMAL = (['left_wheel_joint', 'right_wheel_joint', 'left_leg_joint', 'right_leg_joint',
           'head_joint', 'a_joint', 'b_joint'],
          [1.0, 2.0, -0.10, 0.05, 0.3, 0.4, 0.5])
VIRTUAL = (['body_lift_joint', 'body_pitch_joint'], [0.02, -0.07])


def extract(fn, msgs):
    """스크립트 루프와 같은 규칙: None 이면 시각·값 모두 건너뜀."""
    out = []
    for t, (names, pos) in msgs:
        v = fn(names, pos)
        if v is None:
            continue
        out.append((t, v))
    return out


@pytest.mark.parametrize('fn', FUNCS)
def test_normal_message(fn):
    assert fn(*NORMAL) == pytest.approx(0.10)


@pytest.mark.parametrize('fn', FUNCS)
def test_virtual_only_message_is_skipped(fn):
    assert fn(*VIRTUAL) is None
    assert extract(fn, [(0.0, NORMAL), (0.01, VIRTUAL), (0.02, NORMAL)]) == [
        (0.0, pytest.approx(0.10)), (0.02, pytest.approx(0.10))]


@pytest.mark.parametrize('fn', FUNCS)
def test_virtual_message_first(fn):
    # 예전 코드는 첫 메시지로 인덱스를 고정해 이 경우 다리각이 전부 0.0 이 됐다
    assert extract(fn, [(0.0, VIRTUAL), (0.01, NORMAL)]) == [(0.01, pytest.approx(0.10))]


@pytest.mark.parametrize('fn', FUNCS)
def test_legs_at_different_indices(fn):
    names = ['right_leg_joint', 'body_pitch_joint', 'x_joint', 'left_leg_joint']
    assert fn(names, [0.2, 9.0, 9.0, -0.3]) == pytest.approx(0.3)
    reordered = (['left_leg_joint', 'right_leg_joint'], [-0.01, 0.04])
    assert extract(fn, [(0.0, NORMAL), (0.01, reordered)]) == [
        (0.0, pytest.approx(0.10)), (0.01, pytest.approx(0.04))]
