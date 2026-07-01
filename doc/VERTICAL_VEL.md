# Vertical-velocity soft constraint (planar-motion Level 1) — SW1-1837

지면 로봇의 z-drift를 억제하기 위한 **월드 수직속도 소프트 제약**. `VerticalVelocityFactor`
(`vins_estimator/src/factor/vertical_velocity_factor.h`).

## 무엇을 하나
- 지면 로봇은 평지 주행 시 월드 프레임 수직속도 `Vz ≈ 0`. z-drift = ∫Vz 이므로 `Vz`를 0으로
  약하게 상시 당기면 z 누적을 억제한다.
- residual = `weight · Vz` (`para_SpeedBias[i]`의 인덱스 2). `SizedCostFunction<1,9>`.
- 휠 factor는 평면 속도(vx,vy)만, ZUPT는 정지 프레임만 제약 → **주행 중 z 전용 제약의 공백을 메움.**

## config (`vio_edie.yaml`)
```yaml
use_vertical_vel: 0          # 0=미사용(기존 동작). 1=활성
vertical_vel_weight: 20.0    # σ≈1/w [m/s]. 드리프트 vz(~mm/s)를 물려면 수백 필요
```

## ⚠️ 실측 결과 (2026-07-01, edie_wheel_clean_3) — 증상 치료의 한계
| weight | z 범위 | z drift율 | xy-APE | 비고 |
|---|---|---|---|---|
| OFF | 55cm | 33mm/m | 0.112m | baseline |
| 20 | 67cm | 35mm/m | — | **무효** (σ=0.05m/s ≫ 드리프트 vz~8mm/s → 안 물림) |
| 300 | **20cm** | **7.3mm/m (−78%)** | **0.172m (+54%)** | z 잡으나 xy 희생 |

- **z-drift는 잡힌다**(개념 검증). 단 weight를 충분히(수백) 줘야 물린다.
- **그러나 근본 pitch(~2°)를 안 고쳐** 오차가 **xy로 전가(+54% 악화)**. **vz-only는 증상 치료.**
- → **근본 해법 = VIW-Fusion `plane_factor.h` 이식**(z 위치 + roll/pitch 자세 동시 제약). 진행 중.

## 검증 도구
- `scripts/z_drift_motion.py` — 무정지 구간 z-drift (VINS z 자기이탈, GT 없어도 가능)
- `scripts/ab_compare.py` / `ab_compare_multi.py` — OFF/ON z-APE·xy-APE 비교 (배포조건 다회 평균)
- `scripts/jump_check.py` — 비물리 점프(발산·과제약) 확인
