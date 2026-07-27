#!/usr/bin/env python3
"""[SW1-1837] 정지+동적 장애물 실험 평가 — 참값=무운동(로봇 정지 녹화 전제).
사용: dynobs_stationary_eval.py <tum1> [tum2 ...]
기준 자세/위치 = 각 run의 초반 3~8s 평균(초기화 과도 제외).
출력: 2s 버킷 |Δyaw|·|Δxy|·|Δz| 타임라인 + run 요약(피크·종점) + 판정.
판정선 근거: 재생 노이즈 바닥 실측 0.003m/0.04°(v8 정지 창) 대비 25~30배 여유.
프로토콜: scratchpad/dynobs_protocol.md
"""
import sys, math
import numpy as np

# 카메라 pose(tum) → body/축중점 환산 상수 (vio_edie.yaml extrinsic과 동기)
RIC = np.array([[0.06190178, 0.59267346, 0.80306061],
                [-0.9955198, -0.02095485, 0.09220204],
                [0.07147371, -0.80517021, 0.58872102]])
TIC = np.array([0.1866642923, 0.0206996488, -0.0955281329])
TIO = np.array([0.1056, 0.0, -0.0941])


def q2R(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def body_states(A):
    """tum 행렬 → (t_rel, axle_xy[N,2], yaw[N], z[N])"""
    t0 = A[0, 0]
    ax = np.zeros((len(A), 2))
    yaw = np.zeros(len(A))
    z = np.zeros(len(A))
    for i, a in enumerate(A):
        Rwb = q2R(*a[4:8]) @ RIC.T
        P = a[1:4] - Rwb @ TIC + Rwb @ TIO
        ax[i] = P[:2]
        z[i] = P[2]
        yaw[i] = math.atan2(Rwb[1, 0], Rwb[0, 0])
    return A[:, 0] - t0, ax, np.unwrap(yaw), z


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for path in sys.argv[1:]:
        A = np.array([[float(v) for v in l.split()] for l in open(path)])
        t, ax, yaw, z = body_states(A)
        ref = (t >= 3) & (t <= 8)
        if ref.sum() < 5:
            print(f"{path}: 기준 창(3~8s) 표본 부족 — skip")
            continue
        ax0 = ax[ref].mean(0)
        yaw0 = yaw[ref].mean()
        z0 = z[ref].mean()
        dxy = np.hypot(ax[:, 0] - ax0[0], ax[:, 1] - ax0[1])
        dyaw = np.degrees(yaw - yaw0)
        dz = z - z0
        print(f"\n═══ {path}  (길이 {t[-1]:.0f}s, 기준=3~8s 평균) ═══")
        print(f"{'t[s]':>5s} {'|Δxy|[m]':>9s} {'Δyaw[°]':>8s} {'Δz[m]':>7s}")
        for tb in range(0, int(t[-1]) + 1, 2):
            m = (t >= tb) & (t < tb + 2)
            if m.sum() < 2:
                continue
            print(f"{tb:5d} {dxy[m].max():9.3f} {dyaw[m][np.argmax(np.abs(dyaw[m]))]:8.2f} "
                  f"{dz[m][np.argmax(np.abs(dz[m]))]:7.3f}")
        i_pk_y = int(np.argmax(np.abs(dyaw)))
        i_pk_p = int(np.argmax(dxy))
        tail = t >= t[-1] - 5
        print(f"-- 요약: yaw 피크 {dyaw[i_pk_y]:+.2f}° @{t[i_pk_y]:.0f}s / "
              f"xy 피크 {dxy[i_pk_p]:.3f}m @{t[i_pk_p]:.0f}s / "
              f"종점(마지막 5s) yaw {dyaw[tail][np.argmax(np.abs(dyaw[tail]))]:+.2f}° · xy {dxy[tail].max():.3f}m")
        v_yaw = "합격(<1°)" if np.abs(dyaw).max() < 1.0 else ("취약 실증(≥2°)" if np.abs(dyaw).max() >= 2.0 else "회색지대(1~2°)")
        v_xy = "합격(<0.03)" if dxy.max() < 0.03 else ("취약 실증(≥0.05)" if dxy.max() >= 0.05 else "회색지대")
        print(f"-- 판정: yaw {v_yaw} / xy {v_xy}")


if __name__ == "__main__":
    main()
