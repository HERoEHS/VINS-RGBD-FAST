#!/usr/bin/env python3
"""bag의 depth(32FC1 m)를 오프라인으로 까서 거리 구간별 유효 픽셀 비율을 본다.
근거리(<1.5m, 정확) vs 원거리(노이즈) 앵커 가용성 진단용.
사용: python3 depth_check.py [bag_dir] [depth_topic] [n_frames]"""
import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

bag = sys.argv[1] if len(sys.argv) > 1 else "/home/higony/ros2_ws/bag/klt_gate"
depth_topic = sys.argv[2] if len(sys.argv) > 2 else "/edie/sensors/camera/left/depth"
n_max = int(sys.argv[3]) if len(sys.argv) > 3 else 40

r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
n = 0; v15 = []; v35 = []; meanvalid = []; enc = "?"; wh = "?"
while r.has_next() and n < n_max:
    t, data, _ = r.read_next()
    if t != depth_topic:
        continue
    m = deserialize_message(data, Image)
    enc = m.encoding; wh = f"{m.width}x{m.height}"
    a = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)
    fin = np.isfinite(a) & (a > 0); tot = a.size
    in15 = fin & (a >= 0.3) & (a <= 1.5)
    in35 = fin & (a >= 0.3) & (a <= 3.5)
    v15.append(in15.sum() / tot * 100); v35.append(in35.sum() / tot * 100)
    if in35.sum() > 0:
        meanvalid.append(a[in35].mean())
    n += 1
if n == 0:
    print(f"✗ {depth_topic}(32FC1) 프레임 없음"); sys.exit(1)
print(f"depth {n}프레임 (encoding={enc}, {wh}), topic={depth_topic}")
print(f"유효 픽셀  0.3~1.5m : {np.mean(v15):.1f}%  (근거리=신뢰 앵커)")
print(f"유효 픽셀  0.3~3.5m : {np.mean(v35):.1f}%")
print(f"유효 depth 평균거리(0.3~3.5): {np.mean(meanvalid):.2f} m")
