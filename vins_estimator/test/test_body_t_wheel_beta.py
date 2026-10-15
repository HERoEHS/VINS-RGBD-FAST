# [SW1-1959] vio_edie.yaml body_T_wheel(휠→IMU 거리)이 베타 하드웨어(.251·.52) 설계도와 같은지 확인.
# 이 값은 제자리 회전 때 속도 제약(v_imu = v_wheel + ω×r)에 바로 들어간다. 설계도
# (edie_common edie_description test_beta_mount_positions.py 의 IMU_IN_BASE)만 바뀌고
# 여기가 그대로면 git 은 충돌로 알려 주지 않는다 — 한쪽을 바꾸면 두 시험을 같이 고친다.
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', '..', 'config', 'elp_stereo_edie', 'vio_edie.yaml')

# IMU 위치(base_link 기준) = (-0.0908084, 0, 0.100187) → 역변환 t
BETA_T_IO = (0.0908084, 0.0, -0.100187)


def test_body_t_wheel_matches_beta_imu_position():
    fs = cv2.FileStorage(CONFIG, cv2.FILE_STORAGE_READ)
    m = fs.getNode('body_T_wheel').mat()
    fs.release()
    assert m is not None and m.shape == (4, 4)
    np.testing.assert_allclose(m[:3, :3], np.eye(3), atol=1e-12)  # 자세는 그대로(회전 없음)
    np.testing.assert_allclose(m[:3, 3], BETA_T_IO, atol=1e-9)
