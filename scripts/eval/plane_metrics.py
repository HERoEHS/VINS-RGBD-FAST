#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 평면성 5지표 평가기 — plane/vert 제약 A/B·재현 실험용 (SW1-1837)
#
# [무엇을 하나]
#   VINS 궤적(TUM)을 AprilTag GT(TUM)와 비교해 5개 지표를 한 줄 표로 출력한다.
#   07-01 세션의 인라인 스니펫(kabsch xy-APE + 평면 lstsq tilt)을 재사용 가능한
#   스크립트로 고정한 것 — 재현 실험은 "같은 정의"로 비교해야 의미가 있다.
#
# [지표 (2계열)]
#   GT 기반 (SE3 Kabsch 정렬 후, scale 미보정):
#     · xy-APE RMSE [m] : 수평 오차 (AprilTag 평면 정확 → 신뢰 지표)
#     · z-APE  RMSE [m] : 수직 오차 (⚠️ GT z 노이즈 + SE3 정렬이 tilt 은폐 → 약한 지표)
#   GT 무관 (VINS 궤적 자체, 평지 주행 가정):
#     · 전역 tilt [deg]  : 궤적 전체 lstsq 평면의 기울기 — z 판정의 주지표
#     · 비평면성 RMS [mm]: lstsq 평면에서의 잔차 RMS (국소 울퉁불퉁함)
#     · z 범위 [mm]      : raw z의 max-min (드리프트 총폭)
#
# [사용법]
#   python3 scripts/eval/plane_metrics.py --gt <gt.tum> <est1.tum> [<est2.tum> ...]
#   예) python3 scripts/eval/plane_metrics.py --gt ~/ros2_ws/bag/run_off_vertical_vel_soft_gt.tum \
#         ~/ros2_ws/bag/run_off_vertical_vel_soft_vins.tum
import argparse
import os
import sys

import numpy as np


def load_tum(path):
    """TUM 파일 → (N,8) [t x y z qx qy qz qw]. 주석/빈 줄 무시."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.split()
            if len(v) >= 8:
                rows.append([float(x) for x in v[:8]])
    if not rows:
        sys.exit(f"✗ {path}: 유효한 TUM 행 없음")
    return np.array(rows)


def associate(gt, est, max_dt=0.02):
    """GT 각 시각에 최근접 est 매칭 (evo --t_max_diff 0.02와 동일 규칙)."""
    est_t = est[:, 0]
    pairs = []
    for i, t in enumerate(gt[:, 0]):
        j = int(np.argmin(np.abs(est_t - t)))
        if abs(est_t[j] - t) <= max_dt:
            pairs.append((i, j))
    return pairs


def kabsch_se3(P, Q):
    """P(est)→Q(gt) 최적 SE3 (회전+병진, scale 없음 = evo -a와 동일)."""
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = mq - R @ mp
    return R, t


def plane_fit_metrics(xyz):
    """궤적 자체에 평면 z=ax+by+c lstsq 피팅 → (tilt[deg], 잔차RMS[mm], z범위[mm])."""
    A = np.c_[xyz[:, 0], xyz[:, 1], np.ones(len(xyz))]
    coef, *_ = np.linalg.lstsq(A, xyz[:, 2], rcond=None)
    a, b, _ = coef
    tilt_deg = np.degrees(np.arctan(np.hypot(a, b)))
    resid = xyz[:, 2] - A @ coef
    rms_mm = float(np.sqrt((resid ** 2).mean()) * 1000.0)
    zrange_mm = float((xyz[:, 2].max() - xyz[:, 2].min()) * 1000.0)
    return tilt_deg, rms_mm, zrange_mm


def evaluate(gt_path, est_path, max_dt):
    gt = load_tum(gt_path)
    est = load_tum(est_path)
    pairs = associate(gt, est, max_dt)
    if len(pairs) < 10:
        sys.exit(f"✗ 매칭 쌍 부족({len(pairs)}) — 시간축 확인: {est_path}")
    Q = gt[[i for i, _ in pairs], 1:4]
    P = est[[j for _, j in pairs], 1:4]
    R, t = kabsch_se3(P, Q)
    Pa = (R @ P.T).T + t
    d = Pa - Q
    xy_ape = float(np.sqrt((d[:, 0] ** 2 + d[:, 1] ** 2).mean()))
    z_ape = float(np.sqrt((d[:, 2] ** 2).mean()))
    tilt, nonplanar, zrange = plane_fit_metrics(est[:, 1:4])
    return dict(name=os.path.basename(est_path), pairs=len(pairs),
                xy_ape=xy_ape, z_ape=z_ape, tilt=tilt,
                nonplanar=nonplanar, zrange=zrange)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True, help="AprilTag GT TUM 파일")
    ap.add_argument("est", nargs="+", help="VINS TUM 파일(들)")
    ap.add_argument("--max-dt", type=float, default=0.02)
    args = ap.parse_args()

    print(f"| run | 쌍 | xy-APE [m] | z-APE [m] | 전역tilt [°] | 비평면성 [mm] | z범위 [mm] |")
    print(f"|---|---|---|---|---|---|---|")
    for est_path in args.est:
        m = evaluate(args.gt, est_path, args.max_dt)
        print(f"| {m['name']} | {m['pairs']} | {m['xy_ape']:.4f} | {m['z_ape']:.4f} "
              f"| {m['tilt']:.2f} | {m['nonplanar']:.0f} | {m['zrange']:.0f} |")


if __name__ == "__main__":
    main()
