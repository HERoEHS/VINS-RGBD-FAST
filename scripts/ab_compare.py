#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# acc_bias_prior live A/B 비교 헬퍼 (Run A: prior OFF vs Run B: prior ON)
#
# [왜 만드나]
#   acc_bias_prior(w_z=0, w_xy=20) 효과는 bag 재생이 비결정성(±2.8m)이라 신뢰 못 한다.
#   → 녹화 OFF 라이브 연속주행으로 OFF/ON 두 번 돌려 z drift가 실제로 줄었는지(자세 전가
#     없이) 정량 비교해야 한다. 이 스크립트가 두 run의 TUM 4개를 받아 A/B 표를 찍는다.
#
# [무엇을 비교하나]
#   · z-APE  (주지표): SE3 정렬 후 z 오차 RMS/최대 — 'z drift'가 prior로 줄었나?
#   · xy-APE (가드레일): 수평 오차 RMS — prior가 수평을 망치진 않았나?
#   · 자세 잔차(roll/pitch/yaw RMS): 회전정렬 후 — bias를 누르며 자세로 전가됐나?
#   (정지구간 검출/자세 로직은 attitude_stationary_eval.py 함수 재사용)
#
# ────────────────────────────────────────────────────────────────────────────
# [기록 레시피 — 각 run마다 (A=OFF, B=ON)]
#   0) config 전환:  vio_edie.yaml 의 use_acc_bias_prior  →  A:0 / B:1  (저장 후 VINS 재실행)
#   1) VINS 라이브 실행 (녹화 OFF):
#        ros2 launch vins_estimator edie_vslam.launch.py use_sim_time:=false
#   2) GT 추출+기록 (자기 TUM 직접 기록):
#        python3 scripts/april_gt/gt_apriltag_4board.py \
#          --K 397.64,397.64,339.36,270.85 --tag-size 0.024 --allowed-boards AR1,AR2,AR3 \
#          --frame-id map --child-frame-id apriltag_gt_camera \
#          --gt-odom-topic /apriltag_gt/odom --gt-path-topic /apriltag_gt/path \
#          --output ~/ros2_ws/bag/runA_gt.tum        # B는 runB_gt.tum
#   3) VINS 포즈 기록:
#        python3 scripts/vins_tum_recorder.py --output ~/ros2_ws/bag/runA_vins.tum   # B는 runB_vins.tum
#   4) 같은 경로를 비슷하게 주행 → 끝나면 2),3) 둘 다 Ctrl+C
#   ※ AprilTag 연속 가시성 유지(시야 이탈 시 데이터공백→보간 오염). 정지를 몇 번 섞으면 지표 신뢰↑.
#
# [비교 실행]
#   python3 scripts/ab_compare.py \
#     --a-gt ~/ros2_ws/bag/runA_gt.tum --a-vins ~/ros2_ws/bag/runA_vins.tum \
#     --b-gt ~/ros2_ws/bag/runB_gt.tum --b-vins ~/ros2_ws/bag/runB_vins.tum --plot
# ────────────────────────────────────────────────────────────────────────────
#
# [의존성] numpy, scipy, (옵션)matplotlib  +  같은 폴더의 attitude_stationary_eval.py

import argparse
import os
import sys

import numpy as np

# 같은 scripts/ 폴더의 헬퍼 재사용 (정지검출·정합·오일러 등)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from scipy.spatial.transform import Rotation
    from attitude_stationary_eval import load_tum, associate, detect_stationary, group_segments
except ImportError as e:
    sys.exit(f"[오류] 의존성 누락: {e}  (pip install scipy / attitude_stationary_eval.py 같은 폴더 필요)")


def kabsch(P, Q):
    """R,t 추정: R@P+t ≈ Q (scale 없음, SE3). evo -a(SE3 Umeyama)와 동일 계열."""
    cP, cQ = P.mean(0), Q.mean(0)
    H = (P - cP).T @ (Q - cQ)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = cQ - R @ cP
    return R, t


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def analyze_run(gt_path, vins_path, tol, win, pos_thresh, min_dur, min_n):
    """한 run의 (gt, vins) TUM → 지표 딕셔너리."""
    t_gt, p_gt, q_gt = load_tum(gt_path)
    t_v, p_v, q_v = load_tum(vins_path)
    gi, vi = associate(t_gt, t_v, tol)
    if len(gi) < 5:
        sys.exit(f"[오류] 정합쌍 {len(gi)}개뿐: {os.path.basename(vins_path)} (--t-max-diff 확인)")
    t = t_gt[gi]
    pg, qg = p_gt[gi], q_gt[gi]
    pv, qv = p_v[vi], q_v[vi]

    # 위치 SE3 정렬 → 축별 오차
    R, tt = kabsch(pv, pg)
    pv_al = (R @ pv.T).T + tt
    err = pv_al - pg
    z_ape = np.abs(err[:, 2])
    xy_ape = np.linalg.norm(err[:, :2], axis=1)
    ape = np.linalg.norm(err, axis=1)

    # 자세 1-측 회전정렬 → 축별 잔차(roll/pitch/yaw)
    Rg, Rv = Rotation.from_quat(qg), Rotation.from_quat(qv)
    R_off = (Rg * Rv.inv()).mean()
    re = ((R_off * Rv).inv() * Rg).as_euler("xyz", degrees=True)

    # 정지구간 수 (참고용 신뢰도 지표)
    stationary, _ = detect_stationary(t, pg, win, pos_thresh)
    segs = group_segments(stationary, t, min_dur, min_n)

    elapsed = t - t[0]
    return {
        "n": len(t), "dur": float(t[-1] - t[0]), "n_seg": len(segs),
        "z_ape_rmse": rms(z_ape), "z_ape_max": float(z_ape.max()),
        "xy_ape_rmse": rms(xy_ape), "ape_rmse": rms(ape),
        "roll_rmse": rms(re[:, 0]), "pitch_rmse": rms(re[:, 1]), "yaw_rmse": rms(re[:, 2]),
        "elapsed": elapsed, "z_ape": z_ape,  # plot용
    }


def main():
    ap = argparse.ArgumentParser(description="acc_bias_prior live A/B 비교 (OFF vs ON)")
    ap.add_argument("--a-gt", required=True, help="Run A(prior OFF) GT TUM")
    ap.add_argument("--a-vins", required=True, help="Run A(prior OFF) VINS TUM")
    ap.add_argument("--b-gt", required=True, help="Run B(prior ON) GT TUM")
    ap.add_argument("--b-vins", required=True, help="Run B(prior ON) VINS TUM")
    ap.add_argument("--a-label", default="A: prior OFF")
    ap.add_argument("--b-label", default="B: prior ON")
    ap.add_argument("--t-max-diff", type=float, default=0.05, help="시각 정합 허용오차[s]")
    ap.add_argument("--win", type=float, default=0.6, help="정지 검출 시간창[s]")
    ap.add_argument("--pos-thresh", type=float, default=0.03, help="정지 위치 변동폭 임계[m]")
    ap.add_argument("--min-dur", type=float, default=0.5)
    ap.add_argument("--min-n", type=int, default=3)
    ap.add_argument("--xy-tol", type=float, default=0.10, help="xy 악화 허용비율(가드레일, 기본 10%)")
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    A = analyze_run(args.a_gt, args.a_vins, args.t_max_diff, args.win, args.pos_thresh, args.min_dur, args.min_n)
    B = analyze_run(args.b_gt, args.b_vins, args.t_max_diff, args.win, args.pos_thresh, args.min_dur, args.min_n)

    print(f"\n[Run A] {args.a_label}: {A['n']}쌍, {A['dur']:.1f}s, 정지구간 {A['n_seg']}개  ({os.path.basename(args.a_vins)})")
    print(f"[Run B] {args.b_label}: {B['n']}쌍, {B['dur']:.1f}s, 정지구간 {B['n_seg']}개  ({os.path.basename(args.b_vins)})")

    # ---- A/B 표 ----------------------------------------------------------------
    def row(name, key, unit, better="down"):
        a, b = A[key], B[key]
        d = b - a
        # 개선 화살표: down=작을수록 좋음
        arrow = "↓개선" if (d < 0) else ("↑악화" if d > 0 else "→")
        print(f"  {name:<22} {a:>10.4f} {b:>10.4f} {d:>+10.4f} {unit:<4} {arrow}")

    print("\n========== A/B 비교표  [낮을수록 좋음] ==========")
    print(f"  {'지표':<22} {'A(OFF)':>10} {'B(ON)':>10} {'Δ(B-A)':>10}")
    print("  " + "-" * 66)
    row("z-APE RMSE",  "z_ape_rmse",  "m")   # ★주지표
    row("z-APE max",   "z_ape_max",   "m")
    row("xy-APE RMSE", "xy_ape_rmse", "m")   # 가드레일
    row("full APE RMSE", "ape_rmse",  "m")
    row("roll 잔차 RMSE",  "roll_rmse",  "°")
    row("pitch 잔차 RMSE", "pitch_rmse", "°")
    row("yaw 잔차 RMSE",   "yaw_rmse",   "°")

    # ---- 판정 ------------------------------------------------------------------
    print("\n========== 판정 ==========")
    z_impr = (A["z_ape_rmse"] - B["z_ape_rmse"]) / max(A["z_ape_rmse"], 1e-9) * 100
    xy_chg = (B["xy_ape_rmse"] - A["xy_ape_rmse"]) / max(A["xy_ape_rmse"], 1e-9) * 100
    pitch_impr = (A["pitch_rmse"] - B["pitch_rmse"]) / max(A["pitch_rmse"], 1e-9) * 100

    z_ok = B["z_ape_rmse"] < A["z_ape_rmse"]
    xy_ok = xy_chg <= args.xy_tol * 100
    print(f"  · z drift     : {'개선 ✅' if z_ok else '악화/무변화 ❌'}  ({z_impr:+.1f}%)   ← 주지표")
    print(f"  · xy 가드레일 : {'유지 ✅' if xy_ok else '악화 ⚠️'}  ({xy_chg:+.1f}%, 허용 +{args.xy_tol*100:.0f}%)")
    print(f"  · pitch 전가  : {'완화 ✅' if pitch_impr >= 0 else '증가 ⚠️'}  ({pitch_impr:+.1f}%)")
    if z_ok and xy_ok:
        print("  → 결론: prior ON 채택 권장 (z drift 개선 & 수평 미악화).")
    elif z_ok and not xy_ok:
        print("  → 결론: z는 좋아졌으나 xy 악화 → w_xy 더 낮춰 재시도(예: 10).")
    else:
        print("  → 결론: z 개선 없음 → w_xy 더 강하게(예: 40) 또는 prior 외 원인(extrinsic/depth-init) 재검토.")
    print("  ⚠️ 단 1쌍 비교는 노이즈 큼 — 가능하면 A/B 각 2~3회 반복해 경향 확인.")

    # ---- plot ------------------------------------------------------------------
    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n[plot] matplotlib 없음 → 건너뜀")
            return
        out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")
        out_dir = os.path.abspath(out_dir)
        os.makedirs(out_dir, exist_ok=True)
        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        # (좌) z-오차 시계열
        ax[0].plot(A["elapsed"], A["z_ape"] * 100, color="tab:red", lw=1, label=args.a_label)
        ax[0].plot(B["elapsed"], B["z_ape"] * 100, color="tab:blue", lw=1, label=args.b_label)
        ax[0].set_xlabel("elapsed [s]"); ax[0].set_ylabel("z error [cm]")
        ax[0].set_title("z-APE over time"); ax[0].legend(fontsize=9)
        # (우) z-오차 누적분포(CDF) — 시간축 달라도 직접 비교 가능
        for run, c, lab in ((A, "tab:red", args.a_label), (B, "tab:blue", args.b_label)):
            s = np.sort(run["z_ape"]) * 100
            cdf = np.linspace(0, 100, len(s))
            ax[1].plot(s, cdf, color=c, lw=1.5, label=lab)
        ax[1].set_xlabel("z error [cm]"); ax[1].set_ylabel("percentile [%]")
        ax[1].set_title("z-APE CDF (left=better)"); ax[1].legend(fontsize=9)
        ax[1].grid(alpha=0.3)
        fig.suptitle("acc_bias_prior A/B: z drift comparison")
        fig.tight_layout()
        out = os.path.join(out_dir, "ab_compare.png")
        fig.savefig(out, dpi=110)
        print(f"\n[plot] 저장: {out}")


if __name__ == "__main__":
    main()
