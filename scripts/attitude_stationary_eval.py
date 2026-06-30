#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 정지 구간 자세 오차 분리 측정기 (VINS vs AprilTag GT)
#
# [왜 만드나]
#   rpy plot에서 roll/pitch가 GT와 ~5~6° 어긋나 보였다. 그런데 이게
#     (a) 진짜 VINS 자세 오차인지,
#     (b) AprilTag GT의 "평면 밖(roll/pitch) 노이즈" + 데이터 공백 보간 아티팩트인지
#   를 구분하지 못하면 "고칠 게 있는지"조차 알 수 없다. 이 스크립트가 그 둘을 분리한다.
#
# [핵심 아이디어 — 정렬 가정이 필요 없는 지표부터]
#   로봇이 "정지"한 구간에서는 참 자세가 '상수'다. 따라서:
#     · 그 구간 안에서 자세가 얼마나 흔들리는가(표준편차) = 그 센서의 자세 노이즈.
#       → VINS 두 월드/IMU-extrinsic 같은 '상수 오프셋'은 흔들림에 영향 없음 → 정렬 불필요!
#     · GT roll/pitch std ≫ VINS roll/pitch std 이면, 보이던 5~6°의 상당부분은 GT(AprilTag) 탓.
#
# [세 가지 지표]
#   지표A) 정지 구간 내부 자세 std (roll/pitch/yaw, deg) — GT vs VINS.   ← 정렬 불필요, 가장 신뢰
#   지표B) 연속 정지 구간 '사이' 상대 회전 각도 — GT vs VINS.            ← 상수 프레임오프셋 상쇄(=드리프트)
#   지표C) 전역 1-측 회전정렬 후 축별 잔차(roll/pitch/yaw RMS).          ← roll/pitch가 yaw보다 나쁜지 한눈에
#
# [정지 검출]
#   GT 위치(=기준)를 시간창(window) 안에서 보고, 위치 변동폭(span)이 pos_thresh 미만이면 정지로 본다.
#   순간 속도보다 'span' 방식이 AprilTag 위치 노이즈에 강건하다.
#
# [사용법]
#   python3 scripts/attitude_stationary_eval.py \
#     --gt ~/ros2_ws/bag/live_apriltag_gt.tum \
#     --vins ~/ros2_ws/bag/vins_camera.tum \
#     --plot
#   (출력 plot은 GUI 없이 output/ 폴더에 PNG로 저장 — 현재 Qt(xcb) 깨져 있어 파일 저장 방식 사용)
#
# [의존성] numpy, scipy, (옵션)matplotlib
#   없으면:  pip install numpy scipy matplotlib   또는  sudo apt install python3-numpy python3-scipy python3-matplotlib

import argparse
import os
import sys

import numpy as np

try:
    from scipy.spatial.transform import Rotation
except ImportError:
    sys.exit("[오류] scipy 필요: pip install scipy  또는  sudo apt install python3-scipy")


# ----------------------------------------------------------------------------- I/O
def load_tum(path):
    """TUM 파일(t tx ty tz qx qy qz qw) → (t[N], p[N,3], q[N,4] xyzw). 주석/빈줄 무시."""
    path = os.path.expanduser(path)
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.split()
            if len(v) < 8:
                continue
            rows.append([float(x) for x in v[:8]])
    if not rows:
        sys.exit(f"[오류] 포즈 0개: {path}")
    a = np.asarray(rows, dtype=np.float64)
    t = a[:, 0]
    order = np.argsort(t)            # 시각 오름차순 보장
    a = a[order]
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def associate(t_gt, t_vins, max_diff):
    """GT 각 프레임에 '가장 가까운' VINS 프레임을 매칭. tol 초과는 버림. (gt_idx, vins_idx) 반환."""
    gi, vi = [], []
    j = 0
    n = len(t_vins)
    for i, tg in enumerate(t_gt):
        # tg에 가장 가까운 vins 인덱스로 j를 전진
        while j + 1 < n and abs(t_vins[j + 1] - tg) <= abs(t_vins[j] - tg):
            j += 1
        if abs(t_vins[j] - tg) <= max_diff:
            gi.append(i)
            vi.append(j)
    return np.asarray(gi, dtype=int), np.asarray(vi, dtype=int)


# ------------------------------------------------------------------- 정지 구간 검출
def detect_stationary(t, p, win, pos_thresh):
    """시간창 win[s] 안에서 위치 변동폭(축별 max-min의 최댓값)이 pos_thresh[m] 미만이면 정지(True)."""
    n = len(t)
    stationary = np.zeros(n, dtype=bool)
    span = np.zeros(n)
    for i in range(n):
        lo, hi = t[i] - win / 2.0, t[i] + win / 2.0
        m = (t >= lo) & (t <= hi)
        chunk = p[m]
        s = (chunk.max(axis=0) - chunk.min(axis=0)).max()  # 가장 크게 움직인 축의 변동폭
        span[i] = s
        stationary[i] = s < pos_thresh
    return stationary, span


def group_segments(stationary, t, min_dur, min_n):
    """연속 True 구간을 묶어 (start_idx, end_idx) 리스트로. 너무 짧은(시간/샘플) 구간은 버림."""
    segs = []
    i, n = 0, len(stationary)
    while i < n:
        if stationary[i]:
            j = i
            while j + 1 < n and stationary[j + 1]:
                j += 1
            dur = t[j] - t[i]
            if dur >= min_dur and (j - i + 1) >= min_n:
                segs.append((i, j))
            i = j + 1
        else:
            i += 1
    return segs


# --------------------------------------------------------------------- 자세 헬퍼
def euler_deg(q_xyzw):
    """쿼터니언(xyzw) → extrinsic xyz 오일러(deg). evo의 'sxyz'와 동일 관례 → 기존 rpy plot과 일치."""
    return Rotation.from_quat(q_xyzw).as_euler("xyz", degrees=True)


def circ_std_deg(angles_deg):
    """각도(deg)의 '원형(circular)' 표준편차 — ±180 wrap에 안전. 정지구간 흔들림 측정용."""
    a = np.deg2rad(angles_deg)
    C, S = np.cos(a).mean(), np.sin(a).mean()
    R = np.hypot(C, S)
    R = min(R, 1.0)
    return np.rad2deg(np.sqrt(-2.0 * np.log(max(R, 1e-12))))


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="정지 구간 자세 오차 분리 측정 (VINS vs AprilTag GT)")
    ap.add_argument("--gt", default="~/ros2_ws/bag/live_apriltag_gt.tum")
    ap.add_argument("--vins", default="~/ros2_ws/bag/vins_camera.tum")
    ap.add_argument("--t-max-diff", type=float, default=0.05, help="시각 정합 허용오차[s]")
    ap.add_argument("--win", type=float, default=0.6, help="정지 검출 시간창[s]")
    ap.add_argument("--pos-thresh", type=float, default=0.03, help="정지 위치 변동폭 임계[m]")
    ap.add_argument("--min-dur", type=float, default=0.5, help="정지 구간 최소 지속[s]")
    ap.add_argument("--min-n", type=int, default=3, help="정지 구간 최소 샘플수")
    ap.add_argument("--out-dir", default=None, help="plot 저장 폴더(기본: 이 스크립트의 ../output)")
    ap.add_argument("--plot", action="store_true", help="결과 plot을 PNG로 저장")
    args = ap.parse_args()

    # 1) 로드 + 시각 정합
    t_gt, p_gt, q_gt = load_tum(args.gt)
    t_vins, p_vins, q_vins = load_tum(args.vins)
    gi, vi = associate(t_gt, t_vins, args.t_max_diff)
    if len(gi) < 5:
        sys.exit(f"[오류] 정합쌍 {len(gi)}개뿐. --t-max-diff 늘리거나 파일 확인.")
    t = t_gt[gi]
    pg, qg = p_gt[gi], q_gt[gi]
    pv, qv = p_vins[vi], q_vins[vi]
    print(f"[정합] GT {len(t_gt)} / VINS {len(t_vins)} → 매칭쌍 {len(gi)} "
          f"(tol={args.t_max_diff*1000:.0f}ms), 구간 {t[-1]-t[0]:.1f}s")

    # 2) 정지 구간 검출 (GT 위치 기준)
    stationary, span = detect_stationary(t, pg, args.win, args.pos_thresh)
    segs = group_segments(stationary, t, args.min_dur, args.min_n)
    print(f"[정지검출] win={args.win}s, pos_thresh={args.pos_thresh*100:.1f}cm "
          f"(span 중앙값 {np.median(span)*100:.1f}cm) → 정지 프레임 {stationary.sum()}/{len(t)}, "
          f"정지 구간 {len(segs)}개")
    if not segs:
        sys.exit("[중단] 정지 구간 없음. --pos-thresh 늘리거나 --win 조절해 재시도.")

    eg = euler_deg(qg)   # GT  오일러 (deg) [N,3] = roll,pitch,yaw
    ev = euler_deg(qv)   # VINS 오일러 (deg)
    axis = ("roll", "pitch", "yaw")

    # ---- 지표A) 정지 구간 내부 std (정렬 불필요) -----------------------------------
    print("\n========== 지표A) 정지 구간 내부 자세 흔들림 std [deg] — 정렬 불필요(가장 신뢰) ==========")
    print("  (구간 안에서 참 자세는 상수 → std가 곧 그 센서의 자세 노이즈. GT≫VINS면 5~6°는 GT 탓)")
    hdr = f"  {'seg':>3} {'t0':>7} {'dur':>5} {'n':>4} | " \
          f"{'GT_roll':>8}{'GT_pitch':>9}{'GT_yaw':>8} | {'VI_roll':>8}{'VI_pitch':>9}{'VI_yaw':>8}"
    print(hdr)
    gt_std_all, vi_std_all = [], []
    for s, (a, b) in enumerate(segs):
        sl = slice(a, b + 1)
        gstd = [circ_std_deg(eg[sl, k]) for k in range(3)]
        vstd = [circ_std_deg(ev[sl, k]) for k in range(3)]
        gt_std_all.append(gstd)
        vi_std_all.append(vstd)
        print(f"  {s:>3} {t[a]-t[0]:>7.1f} {t[b]-t[a]:>5.1f} {b-a+1:>4} | "
              f"{gstd[0]:>8.2f}{gstd[1]:>9.2f}{gstd[2]:>8.2f} | "
              f"{vstd[0]:>8.2f}{vstd[1]:>9.2f}{vstd[2]:>8.2f}")
    gt_std_all = np.asarray(gt_std_all)
    vi_std_all = np.asarray(vi_std_all)
    print("  " + "-" * 86)
    print(f"  {'평균':>3} {'':>7} {'':>5} {'':>4} | "
          f"{gt_std_all[:,0].mean():>8.2f}{gt_std_all[:,1].mean():>9.2f}{gt_std_all[:,2].mean():>8.2f} | "
          f"{vi_std_all[:,0].mean():>8.2f}{vi_std_all[:,1].mean():>9.2f}{vi_std_all[:,2].mean():>8.2f}")

    # ---- 지표B) 연속 정지 구간 '사이' 상대 회전 각도 (드리프트) -----------------------
    print("\n========== 지표B) 연속 정지 구간 사이 상대 회전 각도 [deg] — 상수 프레임오프셋 상쇄 ==========")
    print("  (정지→정지 사이 '얼마나 돌았나'를 GT/VINS 각각 측정. 두 값 차이 = 순수 자세 드리프트)")
    print(f"  {'seg i→i+1':>10} {'GT_angle':>9} {'VINS_angle':>11} {'차이':>8}")
    Rg_mean = [Rotation.from_quat(qg[a:b + 1]).mean() for (a, b) in segs]
    Rv_mean = [Rotation.from_quat(qv[a:b + 1]).mean() for (a, b) in segs]
    drift_diffs = []
    for s in range(len(segs) - 1):
        ang_g = np.rad2deg((Rg_mean[s].inv() * Rg_mean[s + 1]).magnitude())
        ang_v = np.rad2deg((Rv_mean[s].inv() * Rv_mean[s + 1]).magnitude())
        drift_diffs.append(abs(ang_g - ang_v))
        print(f"  {s:>4}→{s+1:<4} {ang_g:>9.2f} {ang_v:>11.2f} {abs(ang_g-ang_v):>8.2f}")
    if drift_diffs:
        print(f"  → 상대회전 각도 차이(드리프트) 평균 {np.mean(drift_diffs):.2f}°, "
              f"최대 {np.max(drift_diffs):.2f}°")

    # ---- 지표C) 전역 1-측 회전정렬 후 축별 잔차 RMS --------------------------------
    # R_gt ≈ R_off · R_vins 가 되도록 상수 R_off 를 정지 프레임으로 추정(=상대회전들의 평균).
    # 잔차 E = (R_off·R_vins)^-1 · R_gt → 평균~0, 축별 RMS가 의미.  (정지 vs 이동 비교)
    print("\n========== 지표C) 전역 회전정렬 후 축별 잔차 RMS [deg] — roll/pitch가 yaw보다 나쁜가? ==========")
    st = stationary.copy()
    Rg = Rotation.from_quat(qg)
    Rv = Rotation.from_quat(qv)
    R_off = (Rg[st] * Rv[st].inv()).mean()      # 정지 프레임으로 상수 오프셋 추정
    resid = (R_off * Rv).inv() * Rg              # 잔차 회전 (작아야 정상)
    re = resid.as_euler("xyz", degrees=True)     # [N,3] 축별 잔차

    def rms(x):
        return float(np.sqrt(np.mean(np.square(x))))

    for name, mask in (("정지 프레임", st), ("이동 프레임", ~st)):
        if mask.sum() == 0:
            continue
        r = re[mask]
        print(f"  [{name}] n={mask.sum():>4} | "
              f"roll RMS {rms(r[:,0]):>6.2f}  pitch RMS {rms(r[:,1]):>6.2f}  yaw RMS {rms(r[:,2]):>6.2f}  "
              f"| 전체각 RMS {rms(np.rad2deg(resid[mask].magnitude())):>6.2f}")

    print("\n[해석 가이드]")
    print("  · 지표A에서 GT roll/pitch std ≫ VINS면 → 보이던 5~6°는 상당부분 AprilTag 노이즈(고칠 것 적음).")
    print("  · 지표A에서 VINS도 크거나, 지표B 드리프트가 크면 → 진짜 VINS 자세 오차 → extrinsic/gyro/leveling 검토.")
    print("  · 지표C에서 정지<이동이고 roll/pitch≫yaw면 → 운동 중 여기부족으로 새는 자세(저여기 구간 주의).")

    # ---- plot 저장 -------------------------------------------------------------
    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")                # GUI 없이 파일로 저장 (Qt xcb 깨짐 회피)
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n[plot] matplotlib 없음 → 건너뜀 (pip install matplotlib)")
            return
        out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")
        out_dir = os.path.abspath(out_dir)
        os.makedirs(out_dir, exist_ok=True)
        tt = t - t[0]
        fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
        # (0) 위치 변동폭 span + 정지 음영
        ax[0].plot(tt, span * 100, color="k", lw=1)
        ax[0].axhline(args.pos_thresh * 100, color="r", ls="--", lw=1, label=f"thresh {args.pos_thresh*100:.0f}cm")
        ax[0].set_ylabel("pos span [cm]")
        ax[0].legend(loc="upper right", fontsize=8)
        # (1~3) 축별 잔차  (plot 라벨은 폰트 호환 위해 영문)
        for k, name in enumerate(axis):
            ax[k + 1].plot(tt, re[:, k], color="tab:blue", lw=1, label=f"{name} residual")
            ax[k + 1].axhline(0, color="gray", lw=0.6)
            ax[k + 1].set_ylabel(f"{name} err [deg]")
            ax[k + 1].legend(loc="upper right", fontsize=8)
        # 정지 구간 음영 (모든 subplot)
        for (a, b) in segs:
            for axx in ax:
                axx.axvspan(tt[a], tt[b], color="tab:green", alpha=0.15)
        ax[-1].set_xlabel("t [s]  (green shade = stationary segments)")
        ax[0].set_title("Stationary attitude error (per-axis residual after rotation alignment)")
        fig.tight_layout()
        out = os.path.join(out_dir, "attitude_stationary_eval.png")
        fig.savefig(out, dpi=110)
        print(f"\n[plot] 저장: {out}")


if __name__ == "__main__":
    main()
