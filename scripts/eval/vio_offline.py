#!/usr/bin/env python3
"""녹화된 VIO 출력 bag(/vins_estimator/odometry, /extrinsic)을 오프라인으로 읽어
발산/스케일/extrinsic 수렴을 정량 보고. (라이브 구독 아님 → harness 시간제한 무관)
사용: python3 /tmp/vio_offline.py /tmp/vio_out"""
import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/vio_out"
reader = SequentialReader()
reader.open(StorageOptions(uri=path, storage_id="sqlite3"),
            ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}

pos, ext_t = [], []
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == "/vins_estimator/odometry":
        m = deserialize_message(data, Odometry)
        p = m.pose.pose.position; pos.append((p.x, p.y, p.z))
    elif topic == "/vins_estimator/extrinsic":
        m = deserialize_message(data, Odometry)
        p = m.pose.pose.position; ext_t.append((p.x, p.y, p.z))

p = np.array(pos)
print(f"[vio_offline] odometry {len(p)}개, extrinsic {len(ext_t)}개")
if len(p) >= 5:
    mag = np.linalg.norm(p, axis=1)
    ext = p.max(axis=0) - p.min(axis=0)
    gap = np.linalg.norm(p[-1] - p[0])
    print("── 궤적 ──")
    print(f"  |pos| max : {mag.max():.3f} m  ({'✗ 발산 의심(>20m)' if mag.max()>20 else '✅ 방 규모'})")
    print(f"  최종 pos  : {p[-1][0]:+.3f} {p[-1][1]:+.3f} {p[-1][2]:+.3f}")
    print(f"  범위 x/y/z: {ext[0]:.2f}/{ext[1]:.2f}/{ext[2]:.2f} m")
    print(f"  시작-끝 갭: {gap:.3f} m (루프면 작을수록 좋음)")
else:
    print("✗ odometry 부족 — VIO init 실패 의심")
if len(ext_t) >= 5:
    e = np.array(ext_t); k = max(1, len(e)//5)
    it, ft, fs = e[:k].mean(0), e[-k:].mean(0), e[-k:].std(0)
    print("── extrinsic(cam-IMU) 온라인 추정 ──")
    print(f"  t 초기→최종: [{it[0]:+.4f} {it[1]:+.4f} {it[2]:+.4f}] → [{ft[0]:+.4f} {ft[1]:+.4f} {ft[2]:+.4f}]")
    print(f"  이동량    : {np.linalg.norm(ft-it)*1000:.1f} mm,  최종 std {fs[0]:.4f}/{fs[1]:.4f}/{fs[2]:.4f} (작을수록 수렴)")
