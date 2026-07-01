#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# TUM 궤적의 '점프(비물리적 급이동)' 검출기 — wheel ON/OFF 등 안정성 비교용
#
# [왜]
#   VINS 추정이 프레임 사이에 로봇 물리한계(|v|max≈0.6 m/s)를 넘어 급이동하면 = 실제 이동이 아니라
#   추정값 재조정(글리치/스냅). 휠 글리치·약한 비전 등이 원인. 이 도구로 점프 빈도·크기·축을
#   정량화해 'wheel 제약 제거가 xy를 안정화하는가'를 숫자로 비교한다.
#
# [사용법]
#   python3 scripts/jump_check.py ~/ros2_ws/bag/runA_vins.tum [더 많은 tum ...]
#   python3 scripts/jump_check.py --vmax 0.6 a.tum b.tum   # 물리 |v| 한계 지정
#
# 출력: 파일별 step거리 통계, 비물리 점프 개수(|v|>vmax*배수), 최대 점프(축 분해), xy/z 점프 분리.

import argparse
import os
import numpy as np


def analyze(path, vmax, mult):
    a = np.loadtxt(os.path.expanduser(path))
    t = a[:, 0]
    p = a[:, 1:4]
    order = np.argsort(t)
    t, p = t[order], p[order]
    dt = np.diff(t)
    dp = np.diff(p, axis=0)
    d = np.linalg.norm(dp, axis=1)
    v = d / np.maximum(dt, 1e-9)
    thr = vmax * mult                      # 점프 판정 속도 임계 (물리한계 × 배수)
    jump = v > thr
    # xy / z 점프 기여 분리
    dxy = np.linalg.norm(dp[:, :2], axis=1)
    dz = np.abs(dp[:, 2])
    return {
        "name": os.path.basename(path),
        "n": len(t), "dur": float(t[-1] - t[0]), "dist": float(d.sum()),
        "step_med": float(np.median(d)), "step_p95": float(np.percentile(d, 95)),
        "step_max": float(d.max()), "v_max": float(v.max()),
        "n_jump": int(jump.sum()),
        "jump_dist_xy": float(dxy[jump].sum()) if jump.any() else 0.0,
        "jump_dist_z": float(dz[jump].sum()) if jump.any() else 0.0,
        "max_idx": int(np.argmax(d)), "max_dp": dp[int(np.argmax(d))],
        "max_t": float(t[int(np.argmax(d))] - t[0]),
        # 안정성 지표: 비물리 점프가 차지하는 이동거리 비율(낮을수록 안정)
        "jump_frac": float(d[jump].sum() / max(d.sum(), 1e-9)),
    }


def main():
    ap = argparse.ArgumentParser(description="TUM 궤적 점프(비물리 급이동) 검출")
    ap.add_argument("tum", nargs="+", help="비교할 TUM 파일들")
    ap.add_argument("--vmax", type=float, default=0.6, help="로봇 물리 선속도 한계 [m/s] (기본 0.6)")
    ap.add_argument("--mult", type=float, default=1.5, help="점프 판정 배수 (기본 1.5 → 0.9m/s 초과를 점프로)")
    args = ap.parse_args()
    thr = args.vmax * args.mult
    print(f"점프 임계: |v| > {thr:.2f} m/s  (물리한계 {args.vmax} × {args.mult})\n")
    print(f"  {'파일':<26}{'N':>5}{'거리[m]':>8}{'step95[mm]':>11}{'stepmax[mm]':>12}"
          f"{'vmax[m/s]':>10}{'점프수':>6}{'점프거리비':>9}")
    print("  " + "-" * 92)
    rows = []
    for f in args.tum:
        r = analyze(f, args.vmax, args.mult)
        rows.append(r)
        print(f"  {r['name']:<26}{r['n']:>5}{r['dist']:>8.2f}{r['step_p95']*1000:>11.1f}"
              f"{r['step_max']*1000:>12.1f}{r['v_max']:>10.2f}{r['n_jump']:>6}{r['jump_frac']*100:>8.2f}%")
    print()
    for r in rows:
        dx, dy, dz = r["max_dp"] * 1000
        axis = "z축" if abs(dz) > np.hypot(dx, dy) else "xy평면"
        print(f"  [{r['name']}] 최대점프 t={r['max_t']:.1f}s: Δ=({dx:+.0f},{dy:+.0f},{dz:+.0f})mm → {axis}; "
              f"점프 이동거리 xy {r['jump_dist_xy']*1000:.0f}mm / z {r['jump_dist_z']*1000:.0f}mm")
    if len(rows) >= 2:
        print("\n  [비교] '점프수'·'점프거리비'가 낮을수록 xy 안정. wheel OFF가 ON보다 낮으면 → 휠이 점프 주범.")


if __name__ == "__main__":
    main()
