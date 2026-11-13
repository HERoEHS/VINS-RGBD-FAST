# [SW1-1966] vio_edie.yaml 의 기본 카메라–IMU 외부값이 URDF 설계값인지 확인.
# 이 기본값은 로봇 설정 폴더 vins/extrinsic.yaml 이 없는 모든 로봇이 쓴다(VINS 출발값, 도킹의 중력 방향).
# 2026-11-13 전에는 옛 카메라 한 대에서 잰 부정확한 값(설계값에서 5.6°)이 들어 있었다 —
# 누가 특정 로봇의 실측값을 다시 여기에 넣으면 이 시험이 막는다(로봇 값은 로봇 설정 폴더에).
# 같은 숫자가 edie_jig edie_tui/tools/push_robot_calib.py 의 URDF_EXTRINSIC_R/T 에도 있다 — 한쪽을 바꾸면 같이 바꾼다.
import math
import os
import re

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', '..', 'config', 'elp_stereo_edie', 'vio_edie.yaml')

# URDF: chassis→imu_link, chassis→stereo_camera_left_optical_frame 에서 계산(광축이 수평 위 35°)
URDF_R = np.array([[0.0, 0.5736049112, 0.8191321052],
                   [-1.0, 0.0, 0.0],
                   [0.0, -0.8191321052, 0.5736049112]])
URDF_T = np.array([0.1701666, 0.0297, -0.0739434])


def _read():
    fs = cv2.FileStorage(CONFIG, cv2.FILE_STORAGE_READ)
    r = fs.getNode('extrinsicRotation').mat()
    t = fs.getNode('extrinsicTranslation').mat()
    td = fs.getNode('td').real()
    fs.release()
    return r, t, td


def test_default_extrinsic_is_urdf_design_value():
    r, t, _ = _read()
    assert r is not None and r.shape == (3, 3)
    assert t is not None and t.shape == (3, 1)
    np.testing.assert_allclose(r, URDF_R, atol=1e-9)
    np.testing.assert_allclose(t.ravel(), URDF_T, atol=1e-9)


def test_default_extrinsic_is_rotation_with_35deg_tilt():
    r, _, _ = _read()
    # 회전 행렬(직교·det 1) — VINS·도킹 허용치(1e-3)보다 훨씬 엄하게
    np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-8)
    assert abs(np.linalg.det(r) - 1.0) < 1e-8
    optical_axis = r[:, 2]   # 카메라 앞 방향을 IMU 좌표로
    assert abs(math.degrees(math.asin(optical_axis[2])) - 35.0) < 0.01   # 위로 35°
    assert abs(optical_axis[1]) < 1e-12                                  # 좌우 0
    assert abs(r[2, 0]) < 1e-12                                          # 옆 기울기 0


def test_default_td_stays_old_camera_value():
    # 공용 td 는 옛 카메라 값 그대로 — 새 롤링셔터 카메라의 약 35 ms 를 여기 넣으면 옛 카메라 로봇이 틀어진다.
    _, _, td = _read()
    assert abs(td - 0.012655922077) < 1e-12


def test_docking_extract_pattern_still_finds_the_blocks():
    # edie_station_docking/cmake/extract_extrinsic.cmake 와 같은 무늬: 키 이름 뒤 쌍점부터 첫 닫는 대괄호까지.
    # 주석에 '키 이름+쌍점'이나 대괄호를 쓰면 도킹이 엉뚱한 글자를 뽑는다 — 뽑힌 숫자가 실제 값과 같은지 본다.
    text = open(CONFIG, encoding='utf-8').read()
    for key, want in (('extrinsicRotation', URDF_R.ravel()), ('extrinsicTranslation', URDF_T)):
        block = re.search(key + r':[^\]]*\]', text)
        assert block, key
        data = re.search(r'data:\s*\[([^\]]*)\]', block.group(0))
        assert data, key
        got = np.array([float(x) for x in data.group(1).split(',')])
        np.testing.assert_allclose(got, want, atol=1e-9)
