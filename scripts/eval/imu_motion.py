#!/usr/bin/env python3
"""bag의 IMU(offset_imu)를 15초 윈도우로 분석해 회전(gyro)·병진여기(accel std)를 본다.
VIO 발산이 회전 우세/병진 빈약 구간과 상관되는지 진단용.
사용: python3 imu_motion.py [bag_dir] [imu_topic]"""
import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

bag = sys.argv[1] if len(sys.argv) > 1 else "/home/higony/ros2_ws/bag/klt_gate"
imu_topic = sys.argv[2] if len(sys.argv) > 2 else "/edie/sensor/offset_imu"

r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
t = []; a = []; g = []
while r.has_next():
    tp, d, _ = r.read_next()
    if tp != imu_topic:
        continue
    m = deserialize_message(d, Imu)
    t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    a.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    g.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
if not t:
    print(f"✗ {imu_topic} 메시지 없음"); sys.exit(1)
t = np.array(t); t -= t[0]; a = np.array(a); g = np.array(g)
amag = np.linalg.norm(a, axis=1); gmag = np.linalg.norm(g, axis=1)
print(f"{bag.split('/')[-1]}: IMU {len(t)}개, {t[-1]:.1f}s, topic={imu_topic}")
print("15s 윈도우 | accel std(병진여기) | |a|평균 | gyro평균 | gyro max(rad/s)")
for s in range(0, int(t[-1]) + 1, 15):
    msk = (t >= s) & (t < s + 15)
    if msk.sum() == 0:
        continue
    print(f"  {s:3d}~{s+15:3d} | {np.std(amag[msk]):6.3f} | {amag[msk].mean():6.2f} | "
          f"{gmag[msk].mean():6.3f} | {gmag[msk].max():5.2f}")
