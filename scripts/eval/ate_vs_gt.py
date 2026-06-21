#!/usr/bin/env python3
"""VIO 추정 궤적(/vins_estimator/odometry)을 정답(/gt, OpenLORIS TFMessage)과 비교해
ATE(Absolute Trajectory Error) RMSE를 보고. Umeyama 유사변환 정렬 후 오차 계산.

OpenLORIS /gt 는 tf2_msgs/TFMessage(들). gt frame은 보통 첫 transform을 사용.
사용: python3 ate_vs_gt.py <recorded_out_dir> [gt_child_frame]
  예: python3 ate_vs_gt.py /tmp/vio_ol_wheel
A/B 비교: baseline / wheel 두 out_dir 에 각각 실행해 RMSE 비교.
"""
import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

path = sys.argv[1]
gt_child = sys.argv[2] if len(sys.argv) > 2 else None

reader = SequentialReader()
reader.open(StorageOptions(uri=path, storage_id="sqlite3"), ConverterOptions("", ""))

est = []   # (t, x, y, z)
gt = []    # (t, x, y, z)
gt_frames = {}
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == "/vins_estimator/odometry":
        m = deserialize_message(data, Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        est.append((t, p.x, p.y, p.z))
    elif topic == "/gt":
        m = deserialize_message(data, TFMessage)
        for tr in m.transforms:
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            tl = tr.transform.translation
            gt_frames.setdefault(tr.child_frame_id, []).append((t, tl.x, tl.y, tl.z))

if not est:
    print("✗ /vins_estimator/odometry 없음 — VIO init 실패 의심"); sys.exit(1)
if not gt_frames:
    print("✗ /gt 없음 — 녹화에 /gt 포함됐는지 확인"); sys.exit(1)

# gt child frame 선택 (지정 없으면 샘플 가장 많은 프레임)
if gt_child is None:
    gt_child = max(gt_frames, key=lambda k: len(gt_frames[k]))
gt = gt_frames[gt_child]
print(f"[ate] est {len(est)}개, gt[{gt_child}] {len(gt)}개")

est = np.array(est); gt = np.array(gt)

# 타임스탬프 최근접 association (gt 기준으로 est 매칭, 50ms 이내)
gt_t = gt[:, 0]; est_t = est[:, 0]
pairs = []
for i, t in enumerate(gt_t):
    j = np.argmin(np.abs(est_t - t))
    if abs(est_t[j] - t) < 0.05:
        pairs.append((j, i))
if len(pairs) < 10:
    print(f"✗ 매칭 쌍 부족({len(pairs)}) — 시간축/스탬프 정렬 확인"); sys.exit(1)
P = np.array([est[j, 1:] for j, _ in pairs])   # est xyz
Q = np.array([gt[i, 1:] for _, i in pairs])    # gt xyz
print(f"[ate] 매칭 쌍 {len(pairs)}개")

# Umeyama 유사변환(scale+R+t) 정렬: P -> Q
def umeyama(X, Y):
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    S = Yc.T @ Xc / len(X)
    U, D, Vt = np.linalg.svd(S)
    d = np.sign(np.linalg.det(U @ Vt))
    W = np.diag([1, 1, d])
    R = U @ W @ Vt
    var = (Xc ** 2).sum() / len(X)
    c = (D * np.array([1, 1, d])).sum() / var
    t = my - c * R @ mx
    return c, R, t

c, R, t = umeyama(P, Q)
P_aligned = (c * (R @ P.T).T) + t
err = np.linalg.norm(P_aligned - Q, axis=1)
rmse = np.sqrt((err ** 2).mean())
print("── ATE (Umeyama 정렬 후) ──")
print(f"  RMSE : {rmse:.4f} m")
print(f"  mean : {err.mean():.4f} m   median : {np.median(err):.4f} m   max : {err.max():.4f} m")
print(f"  scale: {c:.4f}  (1.0에서 멀수록 스케일 오차)")
print(f"  궤적 길이(gt): {np.linalg.norm(np.diff(Q, axis=0), axis=1).sum():.2f} m")
