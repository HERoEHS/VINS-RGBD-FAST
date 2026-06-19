import sys, numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
r=SequentialReader(); r.open(StorageOptions(uri=sys.argv[1],storage_id="sqlite3"),ConverterOptions("",""))
ts=[]; pos=[]
while r.has_next():
    t,d,stamp=r.read_next()
    if t=="/vins_estimator/odometry":
        m=deserialize_message(d,Odometry); p=m.pose.pose.position
        ts.append(stamp); pos.append((p.x,p.y,p.z))
p=np.array(pos); ts=np.array(ts); ts=(ts-ts[0])/1e9
mag=np.linalg.norm(p,axis=1)
print(f"{sys.argv[1].split('/')[-1]}: {len(p)}개, 총 {ts[-1]:.1f}s")
print("  시간(s) | |pos|(m) | 직전대비증가(m)  [10% 간격 샘플]")
idx=np.linspace(0,len(p)-1,11).astype(int)
prev=0
for i in idx:
    print(f"   {ts[i]:6.1f} | {mag[i]:10.3f} | {mag[i]-prev:+8.3f}")
    prev=mag[i]
# 프레임간 최대 점프
jumps=np.abs(np.diff(mag))
print(f"  프레임간 |pos| 최대 점프: {jumps.max():.3f} m @ t={ts[1+jumps.argmax()]:.1f}s")
print(f"  처음 5초 |pos| 최대: {mag[ts<5].max() if (ts<5).any() else 0:.3f} m")
