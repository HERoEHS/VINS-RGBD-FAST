#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[SW1-1837] 정지 자세 오차의 bias/자세 분리 — 평가 지표의 성격을 바꾸는 도구.

[왜 만드나 — 지표 오염 사고 (2026-07)]
  '정지 자세 오차 ~3°'를 VINS 자세 오차로 알고 수 주간 중력 재정렬을 쫓았다. 실제로는
  그 3°의 대부분이 부팅 캘리브에 구워진 가속도계 bias 아티팩트였다(단일자세 정적 캘리브가
  tilt/bias를 구분 못하는 원리적 한계 + turn-on bias의 세션 가변성). raw-acc 기준 정지
  자세 오차는 '진짜 자세 드리프트'와 '이 bias'가 뒤섞인 오염된 지표였다.

[수학 — 왜 분리가 되나]
  정지에서 raw accel은 yaw 불변이라 측정 '위' u_meas 는 몸체 고정 상수다. VINS 자세로
  world에 옮긴 m = R_wb · u_meas 를 1차 전개하면:

      m − e_z  ≈  ε(t)  +  [Rz(ψ) − Rz(ψ_init)] · p₀

    · ε(t)  : VINS 월드 자세의 진짜 수평 오차 (우리가 알고 싶은 것)
    · p₀    : 몸체 고정 bias 투영 (heading ψ 따라 도는 아티팩트, init에서 0)
  → heading 의존항을 빼면 진짜 자세 오차만 남는다. p 는 세션 QC 값으로 따로 보고.

[성립 조건 — 없으면 쓰레기]
  p(heading 따라 돎)와 ε(t)(시간 따라 돎)를 가르려면 정지들이 여러 heading에 퍼지고
  heading이 시간과 비상관이어야 한다(bag_acceptance_check.py C6~C8). 뒤엉키면 퇴화.

[남는 가정 — 정직하게]
  진짜 자세 오차 ε(t)가 heading과 무관(시간에만 의존)하다고 가정한다. yaw-커플링으로 ε이
  heading에 묶이면 p 가 부분 오염된다 → p 는 절대값 아닌 다-run 통계로, 큰 잔차 run은
  낮은 신뢰로 다뤄야 한다.

라이브러리(import)로도, CLI로도 쓴다.
  CLI: python3 bias_separation.py <bag> <tum...> [--yaml imu_offsets.yaml]
"""
import argparse
import math

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry

# vio_edie.yaml extrinsicRotation = R_bc (카메라→IMU/바디)
R_BC = np.array([[0.06190178, 0.59267346, 0.80306061],
                 [-0.99551980, -0.02095485, 0.09220204],
                 [0.07147371, -0.80517021, 0.58872102]])
# 정지 판정 임계 (bag_acceptance_check 와 동일 기준)
GYR_MAX, ACC_STD, WIN, STEP = 0.05, 0.15, 0.6, 0.2
LEG_MOTION_MAX = 2.0  # [deg] 정지 창 내 다리 움직임 폭 상한 — 초과 시 그 정지는 오염


def _quat2R(q):  # [qx, qy, qz, qw]
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _Rz2(psi):  # 수평 2D 회전
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s], [s, c]])


def read_bag(bag_path):
    """bag → 정지 검출·heading·다리 게이팅에 필요한 배열 묶음."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_path, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    ts, acc, gyr = [], [], []
    ot, ov, ow, oyaw = [], [], [], []
    jt, jleg = [], []
    leg_idx = None
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/edie/sensor/offset_imu':
            m = deserialize_message(data, Imu)
            ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
            gyr.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        elif topic == '/edie/diff_drive_controller/odom':
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            ot.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            ov.append(m.twist.twist.linear.x)
            ow.append(m.twist.twist.angular.z)
            oyaw.append(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        elif topic == '/joint_states':
            m = deserialize_message(data, JointState)
            if leg_idx is None:
                leg_idx = [i for i, n in enumerate(m.name) if 'leg' in n.lower()]
            jt.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            jleg.append(max(abs(m.position[i]) for i in leg_idx) if leg_idx else 0.0)
    return dict(ts=np.array(ts), acc=np.array(acc), gyr=np.array(gyr),
                ot=np.array(ot), ov=np.array(ov), ow=np.array(ow), oyaw=np.unwrap(np.array(oyaw)),
                jt=np.array(jt), jleg=np.array(jleg))


def detect_stops(bag):
    """IMU 정온 AND 휠 정지 구간을 (시작, 끝, 평균 acc 방향, 다리움직임폭) 리스트로."""
    ts, acc, gyr = bag['ts'], bag['acc'], bag['gyr']
    ot, ov, ow = bag['ot'], bag['ov'], bag['ow']
    gn = np.linalg.norm(gyr, axis=1)
    win = []
    for e in np.arange(ts[0], ts[-1], STEP):
        m = (ts >= e) & (ts < e + WIN)
        if m.sum() < 15:
            continue
        mo = (ot >= e) & (ot < e + WIN)
        wheel_still = mo.any() and np.abs(ov[mo]).max() < 0.02 and np.abs(ow[mo]).max() < 0.05
        win.append((e + WIN / 2, wheel_still and gn[m].max() < GYR_MAX
                    and acc[m].std(axis=0).max() < ACC_STD))
    groups, cur = [], None
    for tc, ok in win:
        if ok:
            cur = [tc, tc] if cur is None else [cur[0], tc]
        elif cur is not None:
            groups.append(cur); cur = None
    if cur is not None:
        groups.append(cur)
    out = []
    for a, b in groups:
        m = (ts >= a - 0.3) & (ts <= b + 0.3)
        if m.sum() < 15:
            continue
        up = acc[m].mean(axis=0); up = up / np.linalg.norm(up)
        jm = (bag['jt'] >= a) & (bag['jt'] <= b)
        legmot = (np.degrees(bag['jleg'][jm].max() - bag['jleg'][jm].min()) if jm.any() else 0.0)
        out.append((a - 0.3, b + 0.3, up, legmot))
    return out


def stop_observations(bag, tum):
    """정지별 관측 (t_rel, ψ_rel[rad], m_horiz[2]) — 다리 움직인 정지·GT 미매칭 제외.
      m = R_wb · u_meas 의 수평 성분. ψ = 초기 정지 대비 상대 heading(VINS 자세 기준)."""
    d = np.loadtxt(tum)
    t0 = bag['ts'][0]
    vt = d[:, 0] - t0
    obs, psi0 = [], None
    for a, b, up, legmot in detect_stops(bag):
        if legmot >= LEG_MOTION_MAX:
            continue
        tc = (a + b) / 2 - t0
        i = int(np.argmin(np.abs(vt - tc)))
        if abs(vt[i] - tc) > 0.5:
            continue
        R_wb = _quat2R(d[i, 4:8]) @ R_BC.T
        psi = math.atan2(R_wb[1, 0], R_wb[0, 0])
        if psi0 is None:
            psi0 = psi
        w = R_wb @ up
        obs.append((tc, psi - psi0, np.array([w[0], w[1]])))
    return obs


def estimate_p_global(obs):
    """전역 fit: m = d0 + d1·t + Rz(ψ)·p. 반환 (p[2], residual_deg)."""
    N = len(obs)
    T = np.array([o[0] for o in obs])
    PSI = np.array([o[1] for o in obs])
    M = np.vstack([o[2] for o in obs])
    A = np.hstack([np.tile(np.eye(2), (N, 1)),
                   np.vstack([np.eye(2) * t for t in T]),
                   np.vstack([_Rz2(p) for p in PSI])])
    x, *_ = np.linalg.lstsq(A, M.reshape(-1), rcond=None)
    res = (A @ x - M.reshape(-1)).reshape(-1, 2)
    return x[4:6], math.degrees(math.sqrt((res ** 2).sum(axis=1).mean()))


def estimate_p_cluster(obs, window=20.0, min_spread_deg=40.0, min_pts=3):
    """드리프트-프리 클러스터 차분: 짧은 시간 창 내 heading이 크게 도는 묶음에서
      (Rz(ψi)−Rz(ψ0))·p = m_i−m_0 로 p 를 풀어 드리프트 모델 가정을 제거.
      반환 (p[2] 또는 None, residual_deg, 사용점수, ψ폭_deg)."""
    T = np.array([o[0] for o in obs])
    PSI = np.array([o[1] for o in obs])
    M = np.vstack([o[2] for o in obs])
    best = None
    for a in range(len(obs)):
        idx = [k for k in range(len(obs)) if 0 <= T[k] - T[a] <= window]
        if len(idx) < min_pts:
            continue
        spread = math.degrees(PSI[idx].max() - PSI[idx].min())
        if spread >= min_spread_deg and (best is None or spread > best[1]):
            best = (idx, spread)
    if best is None:
        return None, float('nan'), 0, 0.0
    idx = best[0]
    A = np.vstack([_Rz2(PSI[k]) - _Rz2(PSI[idx[0]]) for k in idx[1:]])
    y = np.concatenate([M[k] - M[idx[0]] for k in idx[1:]])
    p, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = (A @ p - y).reshape(-1, 2)
    return p, math.degrees(math.sqrt((res ** 2).sum(axis=1).mean())), len(idx), best[1]


def attitude_errors(obs, p):
    """정지별 (t, raw_err_deg, drift_vec[2]).
      drift_vec = m − Rz(ψ)·p = D(t) : heading 의존 bias 를 제거한 잔여 자세 드리프트.
      ⚠️ D(t) 의 '절대 오프셋'은 init 프레임 기울기(≈p)로, 정지-up 측정만으론 분리 불가
        (절대 중력 기준 필요). 따라서 gauge-free 관측량은 D(t) 의 '시간 변동'이다 —
        VINS 자세가 궤적 동안 실제로 얼마나 흔들렸는지. 절대 크기가 아니라 변동을 봐야 한다."""
    rows = []
    for tc, psi, m in obs:
        raw = math.degrees(np.linalg.norm(m))
        drift = m - _Rz2(psi) @ p
        rows.append((tc, raw, drift))
    return rows


def heading_confound(obs):
    """corr(시간, |heading|) — 성립조건 진단(|c|<0.5 권장). 관측 4개 미만이면 nan."""
    if len(obs) < 4:
        return float('nan')
    t = np.array([o[0] for o in obs])
    d = np.abs([math.degrees(o[1]) for o in obs])
    return float(np.corrcoef(t, d)[0, 1])


def separate(bag, tum):
    """한 run 종합 결과 dict — 라이브러리 진입점."""
    obs = stop_observations(bag, tum)
    if len(obs) < 4:
        return dict(n=len(obs), error='관측 정지 4개 미만 — 분리 불가')
    p_c, res_c, npts, spread = estimate_p_cluster(obs)
    p_g, res_g = estimate_p_global(obs)
    p_use = p_c if p_c is not None else p_g
    errs = attitude_errors(obs, p_use)
    drift = np.vstack([e[2] for e in errs])            # D(t) 벡터열
    drift_dev = drift - drift.mean(axis=0)             # 평균 제거 = gauge-free 변동
    drift_std = math.degrees(math.sqrt((drift_dev ** 2).sum(axis=1).mean()))
    drift_ptp = math.degrees(np.linalg.norm(drift.max(axis=0) - drift.min(axis=0)))
    return dict(n=len(obs), confound=heading_confound(obs),
                p_cluster=p_c, res_cluster=res_c, cluster_pts=npts, cluster_spread=spread,
                p_global=p_g, res_global=res_g,
                raw_mean=float(np.mean([e[1] for e in errs])),
                raw_max=float(np.max([e[1] for e in errs])),
                drift_std=drift_std, drift_ptp=drift_ptp, errs=errs)


def _mag_dir(p):
    if p is None:
        return 'n/a'
    return f"{math.degrees(np.linalg.norm(p)):.2f}° ({np.linalg.norm(p) * 9.8:.3f} m/s², dir {math.degrees(math.atan2(p[1], p[0])):.0f}°)"


def main():
    ap = argparse.ArgumentParser(description='정지 자세 오차 bias/자세 분리 (SW1-1837)')
    ap.add_argument('bag')
    ap.add_argument('tums', nargs='+')
    ap.add_argument('--yaml', default=None, help='imu_offsets.yaml — 레벨링 각↔p 교차검증')
    args = ap.parse_args()

    bag = read_bag(args.bag)
    lvl = None
    if args.yaml:
        import yaml as _yaml
        cfg = _yaml.safe_load(open(args.yaml))
        lvl = cfg.get('level_angle_deg')

    ps = []
    for tum in args.tums:
        name = tum.split('/')[-1]
        r = separate(bag, tum)
        if r.get('error'):
            print(f"[{name}] {r['error']}"); continue
        flag = '' if abs(r['confound']) < 0.5 else '  ⚠️heading-시간 뒤엉킴(성립조건 약함)'
        pc = r['p_cluster']
        print(f"\n[{name}] 정지 {r['n']}개, corr(t,|ψ|)={r['confound']:+.2f}{flag}")
        print(f"  ── p (bias 아티팩트, QC 값) ──")
        print(f"     클러스터({r['cluster_pts']}점·ψ폭 {r['cluster_spread']:.0f}°): {_mag_dir(pc)}  잔차 {r['res_cluster']:.3f}°")
        print(f"     전역 fit: {_mag_dir(r['p_global'])}  잔차 {r['res_global']:.3f}°")
        print(f"  ── 자세 오차 (지표 성격 전환) ──")
        print(f"     raw(오염, 옛 지표): 평균 {r['raw_mean']:.2f}° / 최대 {r['raw_max']:.2f}°")
        print(f"     bias 제거 후 자세 드리프트 변동(gauge-free): std {r['drift_std']:.2f}° / p2p {r['drift_ptp']:.2f}°")
        print(f"        (VINS 자세가 궤적 동안 흔들린 폭 — 절대값 아닌 '변동'이 관측 가능량)")
        if pc is not None:
            ps.append(math.degrees(np.linalg.norm(pc)))

    if lvl is not None and ps:
        pm = float(np.mean(ps))
        agree = '일치' if abs(pm - lvl) < 1.0 else '불일치(주의)'
        print(f"\n═══ QC 교차검증 ═══")
        print(f"  wrapper 레벨링 각 {lvl:.2f}°  vs  회귀 |p| 평균 {pm:.2f}°  →  {agree}")
        print(f"  (일치 = 부팅 캘리브에 구워진 bias 를 회귀가 독립 재현 = 인과 확증)")


if __name__ == '__main__':
    main()
