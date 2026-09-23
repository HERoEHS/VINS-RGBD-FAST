#!/usr/bin/env python3
"""v15 핀 규약 xy 오차 평가 (08-10 REPORT_정량평가.md 규약 재현).

GT   : map→apriltag_gt_camera(/tf, 태그 관측 시점만) ∘ apriltag_gt_camera→gt/base_footprint(/tf_static)
휠   : map→odom(/tf_static 핀) ∘ odom→base_footprint(/tf)          ← 대조군, bag 그대로라 재생 간 변동 없음
VINS : /vins_estimator/odometry(map 프레임 body 자세, TUM) ∘ body→base_footprint(0.09293, 0, −0.1341)
사후 궤적 정렬(Umeyama 등)은 하지 않는다(핀 규약).

사용:
  python3 gt_xy_eval.py <bag_dir> [vins.tum ...] [--win 0 160] [--win 160 400] [--csv out.csv]
vins.tum 없이 돌리면 휠 대조군만 계산한다(평가기 자체 검증용: 보고서 휠 RMS 0.473 m 재현 여부).
"""
import argparse
import glob
import sqlite3

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation as R, Slerp
from tf2_msgs.msg import TFMessage

LEVER = np.array([0.09293, 0.0, -0.1341])   # body→base_footprint (vio_edie.yaml body_T_wheel + 바퀴 반경 0.04)


def tf_to_mat(tr):
    t = tr.transform.translation
    q = tr.transform.rotation
    m = np.eye(4)
    m[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def read_bag(bag_dir):
    meta = yaml.safe_load(open(f"{bag_dir}/metadata.yaml"))
    t0 = meta["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"] * 1e-9
    gt_dyn, odom, static = [], [], {}
    for db in sorted(glob.glob(f"{bag_dir}/*.db3")):          # ★조각 전부 읽는다
        con = sqlite3.connect(db)
        ids = {n: i for i, n in con.execute("select id,name from topics")}
        for name in ("/tf", "/tf_static"):
            if name not in ids:
                continue
            for (b,) in con.execute("select data from messages where topic_id=?", (ids[name],)):
                for tr in deserialize_message(b, TFMessage).transforms:
                    key = (tr.header.frame_id, tr.child_frame_id)
                    st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9   # ★header 스탬프로 통일
                    if name == "/tf_static":
                        static[key] = tf_to_mat(tr)
                    elif key == ("map", "apriltag_gt_camera"):
                        gt_dyn.append((st, tf_to_mat(tr)))
                    elif key == ("odom", "base_footprint"):
                        odom.append((st, tf_to_mat(tr)))
    T_cam_gtfp = static[("apriltag_gt_camera", "gt/base_footprint")]
    T_map_odom = static[("map", "odom")]
    gt_dyn.sort(key=lambda x: x[0])
    odom.sort(key=lambda x: x[0])
    gt_t = np.array([t for t, _ in gt_dyn])
    gt_xy = np.array([(m @ T_cam_gtfp)[:2, 3] for _, m in gt_dyn])
    od_t = np.array([t for t, _ in odom])
    od_xy = np.array([(T_map_odom @ m)[:2, 3] for _, m in odom])
    return t0, gt_t, gt_xy, od_t, od_xy


def interp_xy(t_q, t, xy, max_gap):
    """t_q 시점의 xy를 앞뒤 표본으로 선형 보간. 앞뒤 간격이 max_gap보다 크면 NaN."""
    i = np.searchsorted(t, t_q)
    out = np.full((len(t_q), 2), np.nan)
    ok = (i > 0) & (i < len(t))
    i0, i1 = i[ok] - 1, i[ok]
    gap = t[i1] - t[i0]
    good = gap <= max_gap
    w = ((t_q[ok] - t[i0]) / np.maximum(gap, 1e-9))[:, None]
    val = xy[i0] * (1 - w) + xy[i1] * w
    val[~good] = np.nan
    out[ok] = val
    return out


def load_vins_footprint(tum):
    d = np.loadtxt(tum)
    d = d[np.argsort(d[:, 0])]
    rot = R.from_quat(d[:, 4:8])
    fp = d[:, 1:4] + rot.apply(LEVER)          # body 자세에 레버암을 돌려 붙인다
    return d[:, 0], fp[:, :2]


def stats(err):
    err = err[~np.isnan(err)]
    if len(err) == 0:
        return dict(n=0)
    return dict(n=len(err), rms=float(np.sqrt(np.mean(err ** 2))), med=float(np.median(err)),
                p95=float(np.percentile(err, 95)), max=float(err.max()))


def fmt(s):
    if s["n"] == 0:
        return "n=0"
    return f"n={s['n']:4d}  RMS {s['rms']:.3f}  med {s['med']:.3f}  p95 {s['p95']:.3f}  max {s['max']:.3f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("tums", nargs="*")
    ap.add_argument("--win", nargs=2, type=float, action="append", default=None)
    ap.add_argument("--max-gap", type=float, default=0.25, help="보간 허용 앞뒤 간격(s)")
    ap.add_argument("--csv")
    a = ap.parse_args()
    wins = a.win or [(0, 160), (160, 1e9)]

    t0, gt_t, gt_xy, od_t, od_xy = read_bag(a.bag)
    rel = gt_t - t0
    wheel_err = np.linalg.norm(interp_xy(gt_t, od_t, od_xy, a.max_gap) - gt_xy, axis=1)
    print(f"GT 관측 {len(gt_t)}회 (bag 시작 기준 {rel.min():.1f}~{rel.max():.1f} s)")
    print(f"[휠 대조군] 전체  {fmt(stats(wheel_err))}")
    for lo, hi in wins:
        m = (rel >= lo) & (rel < hi)
        print(f"[휠 대조군] {lo:g}~{min(hi, 999):g}s {fmt(stats(wheel_err[m]))}")

    rows = {}
    for tum in a.tums:
        vt, vxy = load_vins_footprint(tum)
        e = np.linalg.norm(interp_xy(gt_t, vt, vxy, a.max_gap) - gt_xy, axis=1)
        rows[tum] = e
        print(f"\n[VINS] {tum}")
        print(f"  전체  {fmt(stats(e))}")
        for lo, hi in wins:
            m = (rel >= lo) & (rel < hi)
            print(f"  {lo:g}~{min(hi, 999):g}s {fmt(stats(e[m]))}")
    if a.csv and rows:
        with open(a.csv, "w") as f:
            f.write("t_rel,wheel_err_xy," + ",".join(f"vins{i}" for i in range(len(rows))) + "\n")
            for k in range(len(gt_t)):
                f.write(f"{rel[k]:.4f},{wheel_err[k]:.4f}," + ",".join(f"{rows[x][k]:.4f}" for x in rows) + "\n")


if __name__ == "__main__":
    main()
