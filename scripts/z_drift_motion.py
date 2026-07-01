#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 무정지(연속 주행) 구간 z-drift 측정기 — "정지에 의존하지 않고도 z가 유계인가?" 검증
#
# [왜]
#   z-drift는 정지/저속 구간에서 수렴한다(관측). 그래서 "운영이 자주 정지→자연 양호"였다.
#   그러나 B2C 로봇은 주행 패턴이 정형화 안 됨 → 긴 '무정지' 구간이 있을 수 있고, 그때 z가
#   유계인지 발산인지는 '측정 안 된 가정'이다. 이 스크립트가 그 전제를 직접 잰다:
#     연속 주행(정지 없이 움직이는) 구간마다 z가 얼마나 새는지 + 그게 이동거리에 비례해
#     커지는지(=무정지 길수록 위험)를 정량화.
#
# [핵심 지표]
#   · 각 무정지 구간: Δz(구간 시작 대비 순 변화), max 이탈, 이동거리, 지속시간, drift율(mm/m)
#   · |Δz| vs 이동거리 회귀 기울기 = "1m 주행당 z 드리프트" → 이게 크고 거리에 비례하면 B2C 위험 실재
#   · 정지 사이 z 리셋 여부(수렴 주장 확인)
#   ※ 평지 가정: 참 z는 상수 → VINS z의 구간내 이탈 = 드리프트. (GT 있으면 gt z 이탈로 교차확인)
#
# [입력] VINS TUM (필수). GT TUM (옵션, 평지·GT노이즈 교차확인용).
#   ⚠️ wheel-ON(정상 유계) 데이터로 볼 것. wheel-OFF 발산 데이터는 무의미.
#
# [사용법]
#   python3 scripts/z_drift_motion.py --vins ~/ros2_ws/bag/runA_vins.tum --gt ~/ros2_ws/bag/runA_gt.tum --plot
#
# [기존 대안(스크립트 없이도)]
#   · VINS CSV 로그(output_path의 vins_result*.csv): t,x,y,z,q,vx,vy,vz — z·vz 직접 있음.
#   · rviz에서 /vins_estimator/path 의 z 축 눈으로.
#   · scripts/eval/vio_timeseries.py: 녹화 bag |pos| 시계열(발산 점진/급격).

import argparse
import os
import numpy as np

sys_dir = os.path.dirname(os.path.abspath(__file__))
import sys
sys.path.insert(0, sys_dir)
try:
    from attitude_stationary_eval import load_tum, associate, detect_stationary
except ImportError as e:
    sys.exit(f"[오류] attitude_stationary_eval.py 임포트 실패: {e} (같은 scripts/ 폴더 필요)")


def moving_segments(stationary, t, min_dur, min_n):
    """정지(True) 사이의 '연속 이동(False)' 구간을 (start,end) 리스트로. 짧은 구간은 버림."""
    segs = []
    i, n = 0, len(stationary)
    while i < n:
        if not stationary[i]:
            j = i
            while j + 1 < n and not stationary[j + 1]:
                j += 1
            if (t[j] - t[i]) >= min_dur and (j - i + 1) >= min_n:
                segs.append((i, j))
            i = j + 1
        else:
            i += 1
    return segs


def main():
    ap = argparse.ArgumentParser(description="무정지 구간 z-drift 측정")
    ap.add_argument("--vins", required=True, help="VINS TUM (wheel-ON 정상 데이터)")
    ap.add_argument("--gt", default=None, help="GT TUM (옵션, 교차확인)")
    ap.add_argument("--t-max-diff", type=float, default=0.05)
    ap.add_argument("--win", type=float, default=0.6, help="정지검출 시간창[s]")
    ap.add_argument("--pos-thresh", type=float, default=0.03, help="정지 위치변동폭 임계[m]")
    ap.add_argument("--min-move-dur", type=float, default=3.0, help="'긴 무정지'로 볼 최소 지속[s]")
    ap.add_argument("--min-move-n", type=int, default=10, help="무정지 구간 최소 샘플수")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()

    tv, pv, qv = load_tum(args.vins)
    t = tv - tv[0]
    # 정지 검출은 VINS 위치 기준(GT 공백 무관하게 전 구간 커버). xy로 이동판정(z 드리프트를 판정에 섞지 않음)
    stationary, span = detect_stationary(t, pv[:, :2], args.win, args.pos_thresh)
    segs = moving_segments(stationary, t, args.min_move_dur, args.min_move_n)

    # GT z (옵션): 같은 시각의 gt z (평지면 상수여야)
    gt_z_at = None
    if args.gt:
        tg, pg, qg = load_tum(args.gt)
        gi, vi = associate(tg, tv, args.t_max_diff)
        # vins index -> gt z 매핑(있는 것만)
        gt_z_at = {int(v): float(pg[g, 2]) for g, v in zip(gi, vi)}

    print(f"[입력] VINS {len(tv)}포즈, {t[-1]:.1f}s. 정지검출 win={args.win}s thr={args.pos_thresh*100:.0f}cm "
          f"→ 정지 {stationary.sum()}/{len(t)}, 무정지 구간 {len(segs)}개 (≥{args.min_move_dur}s)")
    if not segs:
        sys.exit("[중단] 조건 만족 무정지 구간 없음. --min-move-dur 줄이거나 데이터 확인.")

    print("\n=== 무정지 구간별 z-drift ===")
    print(f"  {'seg':>3}{'t0[s]':>7}{'dur[s]':>7}{'거리[m]':>8}{'Δz[mm]':>8}{'max이탈[mm]':>11}"
          f"{'drift[mm/m]':>12}{'GTΔz[mm]':>9}")
    dists, adz = [], []
    for s, (a, b) in enumerate(segs):
        z = pv[a:b + 1, 2]
        z_rel = z - z[0]
        dz = z_rel[-1] * 1000                       # 순 변화
        exc = np.max(np.abs(z_rel)) * 1000          # 최대 이탈
        dist = np.sum(np.linalg.norm(np.diff(pv[a:b + 1, :2], axis=0), axis=1))
        rate = dz / dist if dist > 1e-6 else 0.0    # mm per m
        # GT z 변화(있으면)
        gtxt = "  -  "
        if gt_z_at is not None:
            zs = [gt_z_at[i] for i in range(a, b + 1) if i in gt_z_at]
            if len(zs) >= 2:
                gtxt = f"{(zs[-1]-zs[0])*1000:+.0f}"
        dists.append(dist); adz.append(abs(dz))
        print(f"  {s:>3}{t[a]:>7.1f}{t[b]-t[a]:>7.1f}{dist:>8.2f}{dz:>+8.1f}{exc:>11.1f}"
              f"{rate:>+12.1f}{gtxt:>9}")

    dists = np.array(dists); adz = np.array(adz)
    # |Δz| vs 거리 회귀(원점 통과): slope = 1m당 z 드리프트
    slope = float(np.dot(dists, adz) / np.dot(dists, dists)) if np.dot(dists, dists) > 0 else 0.0
    # 상관(거리 길수록 드리프트 커지나)
    corr = float(np.corrcoef(dists, adz)[0, 1]) if len(dists) > 1 and dists.std() > 0 else float("nan")

    print("\n=== 종합 ===")
    longest = int(np.argmax(dists))
    la, lb = segs[longest]
    print(f"  · 최장 무정지: {dists[longest]:.2f}m / {t[lb]-t[la]:.1f}s 동안 z 순변화 {adz[longest]:.1f}mm")
    print(f"  · z 드리프트율(|Δz| vs 거리 원점회귀): {slope:.2f} mm/m  (= {slope/10:.2f}% 경사 상당)")
    print(f"  · 거리↔|Δz| 상관: {corr:+.2f}  ({'거리비례=누적드리프트' if corr>0.5 else '무상관=거리무관(유계)' if abs(corr)<0.3 else '약상관'})")

    print("\n[해석]")
    print("  · slope 작고(예: <5mm/m) 상관 낮으면 → 무정지여도 z 유계 → B2C 걱정 기우, 상보제약 불필요.")
    print("  · slope 크고 상관 높으면 → 무정지 길수록 z 누적 → B2C 위험 실재 → 소프트 z제약 등 검토.")
    print("  · GTΔz가 0 근처여야 평지 가정 유효(=Δz는 VINS 드리프트). GTΔz가 크면 바닥 경사/ GT노이즈.")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n[plot] matplotlib 없음 → 건너뜀"); return
        out_dir = args.out_dir or os.path.join(sys_dir, "..", "output")
        out_dir = os.path.abspath(out_dir); os.makedirs(out_dir, exist_ok=True)
        # 속도(이동판정용) 재계산
        dt = np.diff(t); d = np.linalg.norm(np.diff(pv[:, :2], axis=0), axis=1)
        spd = np.concatenate([[0], d / np.maximum(dt, 1e-9)])
        fig, ax = plt.subplots(3, 1, figsize=(12, 10))
        ax[0].plot(t, spd, 'k', lw=0.8); ax[0].set_ylabel("xy speed [m/s]")
        ax[1].plot(t, (pv[:, 2] - pv[0, 2]) * 100, 'tab:blue', lw=1, label="VINS z")
        if gt_z_at is not None:
            gi_sorted = sorted(gt_z_at)
            ax[1].plot([t[i] for i in gi_sorted],
                       [(gt_z_at[i] - pv[0, 2]) * 100 for i in gi_sorted],
                       'tab:green', lw=1, ls='--', label="GT z")
        ax[1].set_ylabel("z - z0 [cm]"); ax[1].legend(fontsize=8)
        for (a, b) in segs:                       # 무정지 구간 음영
            for axx in ax[:2]:
                axx.axvspan(t[a], t[b], color='tab:orange', alpha=0.15)
        ax[0].set_title("non-stop segments (orange) — does z drift while moving?")
        ax[2].scatter(dists, adz, c='tab:red'); ax[2].plot([0, dists.max()], [0, slope * dists.max()], 'k--',
                     label=f"{slope:.1f} mm/m")
        ax[2].set_xlabel("segment distance [m]"); ax[2].set_ylabel("|Δz| [mm]")
        ax[2].set_title("z drift vs distance (slope up = accumulates)"); ax[2].legend(fontsize=8)
        ax[1].set_xlabel("t [s]")
        fig.tight_layout()
        out = os.path.join(out_dir, "z_drift_motion.png"); fig.savefig(out, dpi=110)
        print(f"\n[plot] 저장: {out}")


if __name__ == "__main__":
    main()
