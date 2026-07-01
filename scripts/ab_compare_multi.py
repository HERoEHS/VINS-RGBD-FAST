#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# acc_bias_prior 다회(N-run) 평균 A/B 비교 — 배포조건(max_solver_time=0.04)에서 노이즈를 평균으로 제거
#
# [왜 만드나]
#   max_solver_time를 올려 결정론을 만들면 LIVE(0.04)와 조건이 달라져 튜닝이 전이되지 않는다.
#   → 배포조건 0.04를 '그대로' 두고, 대신 OFF/ON을 각각 여러 번 기록해 '평균'을 비교한다.
#     run마다 솔버 컷오프 비결정성(z-APE ±~3.6mm)이 있어도, 평균의 표준오차는 √N로 줄어
#     N회 반복하면 작은 prior 효과도 노이즈 위로 드러난다(또는 "효과 없음"이 확정된다).
#
# [무엇을 하나]
#   OFF run들과 ON run들을 각각 받아, 각 run의 지표(z-APE RMSE 등)를 ab_compare.analyze_run으로 계산,
#   조건별 평균±표준편차(N)와 Δ(ON-OFF), 그리고 분리가 유의한지(Δ/표준오차) 출력.
#
# [사용법]  (gt/vins는 run 순서대로 짝 맞춰 나열)
#   python3 scripts/ab_compare_multi.py \
#     --off-gt   runA1_gt.tum runA2_gt.tum runA3_gt.tum \
#     --off-vins runA1_vins.tum runA2_vins.tum runA3_vins.tum \
#     --on-gt    runB1_gt.tum runB2_gt.tum runB3_gt.tum \
#     --on-vins  runB1_vins.tum runB2_vins.tum runB3_vins.tum
#
# [의존성] numpy, scipy + 같은 폴더의 ab_compare.py / attitude_stationary_eval.py

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from ab_compare import analyze_run
except ImportError as e:
    sys.exit(f"[오류] ab_compare.py 임포트 실패: {e} (같은 scripts/ 폴더에 있어야 함)")

# (표시이름, 키, 단위, 작을수록 좋음) — 비교/판정에 쓸 지표
METRICS = [
    ("z-APE RMSE",   "z_ape_rmse",  "m"),   # ★주지표
    ("z-APE max",    "z_ape_max",   "m"),
    ("xy-APE RMSE",  "xy_ape_rmse", "m"),   # 가드레일
    ("full APE RMSE","ape_rmse",    "m"),
    ("pitch 잔차 RMSE","pitch_rmse", "°"),
]


def collect(gts, vinss, tol, win, pos_thresh, min_dur, min_n, label):
    """한 조건(OFF 또는 ON)의 run들을 분석 → {key: np.array([run별 값])}."""
    if len(gts) != len(vinss):
        sys.exit(f"[오류] {label}: gt {len(gts)}개 vs vins {len(vinss)}개 — 개수가 같아야 짝이 맞습니다.")
    runs = []
    for i, (g, v) in enumerate(zip(gts, vinss)):
        m = analyze_run(g, v, tol, win, pos_thresh, min_dur, min_n)
        runs.append(m)
        print(f"  [{label} #{i+1}] {os.path.basename(v):<28} "
              f"z-APE {m['z_ape_rmse']*100:5.2f}cm  xy-APE {m['xy_ape_rmse']*100:6.2f}cm  "
              f"pitch {m['pitch_rmse']:.2f}°  (정지구간 {m['n_seg']})")
    out = {}
    for _, key, _ in METRICS:
        out[key] = np.array([r[key] for r in runs], dtype=float)
    return out


def main():
    ap = argparse.ArgumentParser(description="acc_bias_prior 다회 평균 A/B (배포조건 0.04)")
    ap.add_argument("--off-gt",   nargs="+", required=True, help="OFF run들의 GT TUM (run 순서대로)")
    ap.add_argument("--off-vins", nargs="+", required=True, help="OFF run들의 VINS TUM (run 순서대로)")
    ap.add_argument("--on-gt",    nargs="+", required=True, help="ON run들의 GT TUM")
    ap.add_argument("--on-vins",  nargs="+", required=True, help="ON run들의 VINS TUM")
    ap.add_argument("--t-max-diff", type=float, default=0.05)
    ap.add_argument("--win", type=float, default=0.6)
    ap.add_argument("--pos-thresh", type=float, default=0.03)
    ap.add_argument("--min-dur", type=float, default=0.5)
    ap.add_argument("--min-n", type=int, default=3)
    args = ap.parse_args()

    print("=== 개별 run 분석 ===")
    OFF = collect(args.off_gt, args.off_vins, args.t_max_diff, args.win, args.pos_thresh,
                  args.min_dur, args.min_n, "OFF")
    ON = collect(args.on_gt, args.on_vins, args.t_max_diff, args.win, args.pos_thresh,
                 args.min_dur, args.min_n, "ON")
    nA, nB = len(args.off_vins), len(args.on_vins)

    # ---- 조건별 평균±표준편차 + Δ + 유의성 ----
    print(f"\n========== 다회 평균 A/B  (OFF n={nA}, ON n={nB})  [낮을수록 좋음] ==========")
    print(f"  {'지표':<16}{'OFF 평균±std':>20}{'ON 평균±std':>20}{'Δ(ON-OFF)':>13}{'Δ/SE':>8}  판정")
    print("  " + "-" * 86)
    for name, key, unit in METRICS:
        a, b = OFF[key], ON[key]
        ma, sa = a.mean(), a.std(ddof=1) if nA > 1 else 0.0
        mb, sb = b.mean(), b.std(ddof=1) if nB > 1 else 0.0
        d = mb - ma
        # 표준오차(SE): 두 평균 차이의 불확실성. |Δ/SE|≥2 ≈ 95% 유의(정규근사)
        se = np.sqrt((sa**2 / max(nA, 1)) + (sb**2 / max(nB, 1)))
        ratio = d / se if se > 1e-12 else float("inf")
        if abs(ratio) < 2:
            verdict = "≈ 차이 불명확"
        elif d < 0:
            verdict = "↓개선(유의)"
        else:
            verdict = "↑악화(유의)"
        # 단위 스케일(° 는 그대로, m 는 그대로 출력)
        print(f"  {name:<16}{ma:8.4f}±{sa:7.4f}{mb:9.4f}±{sb:7.4f}{d:>+13.4f}{ratio:>8.1f}  {verdict} [{unit}]")

    # ---- 종합 판정 (주지표 z-APE RMSE 기준) ----
    za, zb = OFF["z_ape_rmse"], ON["z_ape_rmse"]
    xa, xb = OFF["xy_ape_rmse"], ON["xy_ape_rmse"]
    dz = zb.mean() - za.mean()
    sez = np.sqrt((za.std(ddof=1)**2 / nA if nA > 1 else 0) + (zb.std(ddof=1)**2 / nB if nB > 1 else 0))
    dx = xb.mean() - xa.mean()
    sex = np.sqrt((xa.std(ddof=1)**2 / nA if nA > 1 else 0) + (xb.std(ddof=1)**2 / nB if nB > 1 else 0))

    print("\n========== 종합 판정 ==========")
    z_sig = sez > 1e-12 and abs(dz / sez) >= 2
    x_worse_sig = sex > 1e-12 and (dx / sex) >= 2
    if not z_sig:
        print(f"  · z drift: ON-OFF 차이 {dz*1000:+.2f}mm 가 노이즈에 묻힘(|Δ/SE|<2) → 효과 불명확.")
        print("    → run 수를 늘리거나(√N), 효과 자체가 작다는 뜻(=prior 실익 적음).")
    elif dz < 0 and not x_worse_sig:
        print(f"  · z drift {dz*1000:+.2f}mm 유의 개선 & xy 미악화 → ★prior ON 채택 권장.")
    elif dz < 0 and x_worse_sig:
        print(f"  · z는 개선이나 xy {dx*100:+.2f}cm 유의 악화 → w_xy 낮춰 재시도.")
    else:
        print(f"  · z drift {dz*1000:+.2f}mm 유의 악화 → ON 불리. w_xy 방향 재검토 또는 prior 외 원인.")

    if nA < 3 or nB < 3:
        print(f"  ⚠️ N이 작습니다(OFF {nA}, ON {nB}). 정규근사 유의성은 참고용 — 각 3~5회 이상 권장.")


if __name__ == "__main__":
    main()
