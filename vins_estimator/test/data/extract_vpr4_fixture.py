#!/usr/bin/env python3
"""SW1-1936 gtest 발췌: vpr4 앵커 pose(xy 신규 래치 259.342) → 가드 발동·시드 캡처(280.870) 구간의
IMU 원시 자이로 + 휠 이동 플래그(odom twist max(|v|,|w|) ≥ 0.05, 코드의 last_wheel_speed_ 와 같은 정의).
sqlite mode=ro 로만 읽는다(재생 없음). 출력: t_rel,gx,gy,gz,wheel_moving (t_rel = IMU 스탬프 − 앵커 pose 시각)."""
import sqlite3, sys, bisect
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
DB = "/home/higony/ros2_ws/bag/vpr4_0928_1744/vpr4_0928_1744_0.db3"
T_ANCHOR, T_CAPTURE = 1790586259.342, 1790586280.870
out = sys.argv[1]
c = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
tops = {r[1]: r[0] for r in c.execute("select id,name from topics")}
lo, hi = int((T_ANCHOR - 5) * 1e9), int((T_CAPTURE + 5) * 1e9)
odo = []
for ts, b in c.execute("select timestamp,data from messages where topic_id=? and timestamp between ? and ?",
                       (tops["/edie/diff_drive_controller/odom"], lo, hi)):
    m = deserialize_message(b, Odometry)
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    v = m.twist.twist.linear; w = m.twist.twist.angular
    odo.append((t, max((v.x**2 + v.y**2 + v.z**2) ** 0.5, (w.x**2 + w.y**2 + w.z**2) ** 0.5)))
odo.sort(); ot = [o[0] for o in odo]
rows = []
for ts, b in c.execute("select timestamp,data from messages where topic_id=? and timestamp between ? and ?",
                       (tops["/edie/sensor/offset_imu"], lo, hi)):
    m = deserialize_message(b, Imu)
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if T_ANCHOR <= t <= T_CAPTURE:
        k = bisect.bisect_right(ot, t) - 1
        wm = 1 if (k >= 0 and odo[k][1] >= 0.05) else 0
        rows.append((t - T_ANCHOR, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z, wm))
rows.sort()
with open(out, "w") as f:
    f.write("# vpr4_0928_1744 IMU(/edie/sensor/offset_imu) 원시 자이로, 앵커 pose 1790586259.342 기준 상대시각 → 시드 캡처 1790586280.870\n")
    f.write("# 열: t_rel[s],gx,gy,gz[rad/s],wheel_moving(odom twist max(|v|,|w|)>=0.05). 추출: bag/analysis/reboot_seed_stale_anchor_20260929/extract_vpr4_fixture.py\n")
    for r in rows:
        f.write(f"{r[0]:.6f},{r[1]:.6f},{r[2]:.6f},{r[3]:.6f},{r[4]}\n")
print(len(rows), "rows; wheel_moving", sum(r[4] for r in rows))
