#!/usr/bin/env python3
"""계획서 KPI 경로(vins/base_footprint TF) 직접 검증.

런마다 녹화한 tfbag 에서 map→body(동적) ∘ body→vins/base_link→vins/base_footprint(정적)를 합성하고,
  ① odometry TUM + 레버암(gt_xy_eval.py 방식)과의 위치 차이
  ② GT(gt/base_footprint) 대비 xy 오차 (TF 경로 vs odometry 경로)
  ③ 시작 정지 구간 오차 벡터 vs R(t)·LEVER (표시 앵커가 body 를 핀 위치에 맞추는지)
를 출력한다.
사용: python3 tf_kpi_check.py <gt_bag_dir> <out_dir> <label> [<label> ...]
"""
import glob
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from gt_xy_eval import LEVER, interp_xy, load_vins_footprint, read_bag, stats  # noqa: E402


def tf_mat(tr):
    t, q = tr.transform.translation, tr.transform.rotation
    m = np.eye(4); m[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix(); m[:3, 3] = [t.x, t.y, t.z]
    return m


def read_vins_tf(tfbag):
    dyn, static = [], {}
    for db in sorted(glob.glob(f"{tfbag}/*.db3")):
        con = sqlite3.connect(db)
        ids = {n: i for i, n in con.execute("select id,name from topics")}
        for name in ("/tf", "/tf_static"):
            if name not in ids:
                continue
            for (b,) in con.execute("select data from messages where topic_id=?", (ids[name],)):
                for tr in deserialize_message(b, TFMessage).transforms:
                    key = (tr.header.frame_id, tr.child_frame_id)
                    if name == "/tf_static":
                        static[key] = tf_mat(tr)
                    elif key == ("map", "body"):
                        dyn.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, tf_mat(tr)))
    chain = static[("body", "vins/base_link")] @ static[("vins/base_link", "vins/base_footprint")]
    dyn.sort(key=lambda x: x[0])
    t = np.array([a for a, _ in dyn]); fp = np.array([(m @ chain)[:2, 3] for _, m in dyn])
    yaw = np.array([R.from_matrix(m[:3, :3]).as_euler("zyx")[0] for _, m in dyn])
    return t, fp, yaw, chain


gt_bag, out = sys.argv[1], sys.argv[2]
t0, gt_t, gt_xy, _, _ = read_bag(gt_bag)
rel = gt_t - t0
for label in sys.argv[3:]:
    tt, tfp, tyaw, chain = read_vins_tf(f"{out}/{label}.tfbag")
    ot, ofp = load_vins_footprint(f"{out}/{label}.tum")
    print(f"\n[{label}] 정적 체인 body→vins/base_footprint 이동 = {np.round(chain[:3, 3], 4)} (평가기 LEVER {LEVER})")
    # ① 두 경로 차이 (odometry 시각에서 TF 보간)
    d = np.linalg.norm(interp_xy(ot, tt, tfp, 0.25) - ofp, axis=1)
    d = d[~np.isnan(d)]
    print(f"  ① TF경로 vs odometry+레버: n={len(d)} 중앙값 {np.median(d)*100:.2f} cm, p95 {np.percentile(d,95)*100:.2f} cm, 최대 {d.max()*100:.2f} cm")
    # ② GT 오차
    for name, (tq, xy) in (("TF 경로", (tt, tfp)), ("odom+레버", (ot, ofp))):
        e = np.linalg.norm(interp_xy(gt_t, tq, xy, 0.25) - gt_xy, axis=1)
        s_all, s_early = stats(e), stats(e[rel < 160])
        print(f"  ② {name:9s} 전체 RMS {s_all['rms']:.3f} (n={s_all['n']}) | 0~160s RMS {s_early['rms']:.3f}")
    # ③ 시작 10 s 오차 벡터 vs R(yaw)·LEVER
    m = rel < 10
    ev = interp_xy(gt_t[m], tt, tfp, 0.25) - gt_xy[m]
    y0 = np.interp(gt_t[m], tt, np.unwrap(tyaw))
    rl = np.c_[np.cos(y0) * LEVER[0] - np.sin(y0) * LEVER[1], np.sin(y0) * LEVER[0] + np.cos(y0) * LEVER[1]]
    ok = ~np.isnan(ev[:, 0])
    print(f"  ③ 시작 10 s 오차 벡터 평균 {np.round(np.nanmean(ev, 0), 4)} vs R(yaw)·LEVER 평균 {np.round(rl[ok].mean(0), 4)} "
          f"(차이 {np.linalg.norm(np.nanmean(ev, 0) - rl[ok].mean(0))*100:.1f} cm)")
