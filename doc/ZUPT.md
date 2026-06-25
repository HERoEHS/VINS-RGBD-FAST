# ZUPT — Zero-velocity Update (SW1-1837)

정지(zero-velocity) 구간에서 body 속도를 0으로 제약해 **z drift 누적을 완화**하는 기능.

## 왜 필요한가 (배경)

EDIE는 평지를 주로 주행한다. 평지에서는 VINS의 **수평 가속도 bias(acc bias x/y) 관측성이 약해**,
VINS가 이 bias를 과대·오추정한다. 잘못 추정된 수평 bias는 body 자세(pitch)를 약 -1.0° 기울여 추정하게
만들고, 전진할 때 이 pitch 오차가 z축으로 새어 나가 **z drift(높이 흘러내림)** 가 누적된다.

- 휠 factor는 **평면 속도만** 제약하므로 z velocity를 못 잡는다.
- ZUPT은 정지 구간마다 속도(**z 포함**)를 0으로 끊어 누적을 막아 z drift를 보완한다.

> 근본 원인 분석 전체는 메모리 `edie-vio-heading-drift` 참조. ZUPT은 근본 pitch bias를 고치는 게 아니라
> z **누적**만 끊는 완화책이다.

## 동작 방식

최적화 루프(`estimator.cpp` `optimization()`, `[SW1-1837] ZUPT` 블록)에서:

1. `USE_ZUPT && USE_WHEEL` 일 때만 활성화. **휠 데이터(use_wheel:1)가 전제**다 — 정지 판정을 휠
   preintegration으로 하기 때문.
2. 각 프레임 `i`에 대해 휠 preintegration으로 평균 속도를 구한다:
   - 평균 선속도 `v_avg = |delta_p| / sum_dt` [m/s]
   - 평균 각속도 `w_avg = 2·acos(|delta_q.w|) / sum_dt` [rad/s]
3. `v_avg < zupt_vel_thresh && w_avg < zupt_gyr_thresh` 이면 정지로 판정하고,
   해당 프레임 `para_SpeedBias[i]`의 속도(0:3)에 `ZeroVelocityFactor`(잔차 = weight·V)를 추가한다.

잔차 정의(`factor/zero_velocity_factor.h`, `ceres::SizedCostFunction<3,9>`):

```
residual = weight * V           // 속도 0:3 성분만, bias 성분은 무시
J(0,0)=J(1,1)=J(2,2)=weight     // 나머지 자코비안 원소는 0
```

## 파라미터 (config: `config/elp_stereo_edie/vio_edie.yaml`)

| 키 | 기본값 | 의미 |
|----|--------|------|
| `use_zupt` | `0`(키 없으면 비활성) | 1=ZUPT 사용. **use_wheel:1 동반 필요** |
| `zupt_vel_thresh` | `0.02` | 정지 판정 평균 선속도 임계 [m/s] |
| `zupt_gyr_thresh` | `0.02` | 정지 판정 평균 각속도 임계 [rad/s] |
| `zupt_weight` | `100.0` | zero-velocity 잔차 가중치 (클수록 강하게 0) |

운영 config는 `use_zupt: 1`, `zupt_weight: 100.0` 으로 확정.

## A/B 검증 결과 (닫힌루프, 중복 stamp 0 = 깨끗)

| 데이터셋 | 정지비율 | z drift OFF → ON | 비고 |
|----------|----------|------------------|------|
| `edie_wheel_base_old` (연속주행) | 7.2% | 1.238 → 0.999 m (**-19%**) | xy 약간 악화(+0.11m) |
| `edie_wheel_base` (운영유사) | 30.6% | 0.145 → 0.099 m (**-32%**) | 경로 과대 교정(개선) |

- 두 경우 모두 **발산 없음**(|pos| 정상). pitch -1.0°는 그대로(ZUPT은 근본 pitch가 아닌 z 누적만 완화).
- **weight sweep** (`_old`, 0/100/500/1000): z는 켜는 것 자체가 효과(~1.0m, weight 무관). weight↑는 z 추가
  감소 없이 **xy만 단조 악화**(0.62→1.21m) → **weight 100 확정**(최소가 최적).
- 운영(자주 정지)에서 z·경로 둘 다 개선, 발산 없음.

## 회귀 테스트

`vins_estimator/test/test_zero_velocity_factor.cpp` (gtest) — `ZeroVelocityFactor`의 잔차 공식과
자코비안 구조를 고정한다.

```bash
# ~/ros2_ws 에서
colcon build --packages-select vins_estimator --cmake-args -DBUILD_TESTING=ON
colcon test  --packages-select vins_estimator --ctest-args -R test_zero_velocity_factor
colcon test-result --verbose
```

> 정지 판정 임계 로직(`v_avg/w_avg`)은 `optimization()` 안에서 `pre_integrations_wheel`에 강하게
> 결합돼 있어 단위 테스트 대상에서 제외했다. 통합 동작은 위 A/B(bag 재현)로 검증됨.

## 한계 / 알려진 사항

- **`edie_wheel_base_gt`에서는 단독 ZUPT 검증 불가**: ZUPT은 `use_wheel:1`을 강제하는데, 그 bag은 wheel
  velocity 글리치(106 m/s)가 있어 `use_wheel:1`이면 VINS가 발산한다(메모리 `edie-wheel-odom-glitch`).
  `_gt`에서 검증하려면 wheel velocity outlier 게이팅 선행 필요(다음 과제).
- ZUPT은 z **누적 완화**책이지 pitch bias 근본 해결책이 아니다.
- 연속 장거리(정지 적은) 시나리오에서는 효과가 작다(_old -19% vs 운영유사 -32%).
