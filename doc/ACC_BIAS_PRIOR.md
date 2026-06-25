# Accelerometer-bias prior (SW1-1836)

## 왜 필요한가 (근본원인)

평지 주행에서는 **수평 가속도계 bias(ax/ay)의 관측성이 약하다**(평면 운동이라 bias와 기울기를
분리할 단서가 부족). 그 결과 VINS가 수평 bias를 **과대추정**한다 — 정지 실측 기준 진짜
`ay ≈ -0.056`인데 VINS 추정 `Ba_y ≈ -0.21`(약 3.75배). 이 오차가 body **pitch**로 새고,
전진 시 중력이 z축으로 누설되어 **z drift**가 누적된다(연속주행 bag에서 1.2m, 10.5mm/s).

> 자세한 근본원인 추적은 메모리/핸드오프(`edie-vio-heading-drift`) 참조.

## 무엇을 하는가

관측성이 약한 양은 데이터에만 맡기지 말고 **사전지식으로 구속**하는 것이 정석이다.
IMU 드라이버(imu-ros2-wrapper)가 **부팅 시 정지 상태에서 acc bias를 ~0으로 보정**하므로,
세션 시작 시 참 bias ≈ 0이다. 따라서 윈도 각 프레임의 acc bias를 **target=0** 으로 약하게
당기는 prior factor를 추가해 과대추정만 억제한다.

- 잔차: `residual_i = w_i * (Ba_i - target_i)`,  `i ∈ {x,y,z}`, target=0
- 대상: `para_SpeedBias[i]` 의 acc bias 성분(인덱스 3:6). 속도·gyro bias는 건드리지 않음.
- **축별 분리**: `az`(수직)는 이미 정확히 추정됨(실측 0.031 ≈ 진짜 0.032) → `w_z=0`.
  과대추정되는 **수평(ax/ay)만** `w_xy`로 당긴다.

### config-only(acc_w 축소)와의 차이

`acc_w`(bias random-walk) 축소는 bias의 **변화율**을 묶어 초기값 0 근처에 "얼린다". 이는
진짜 bias가 0이 아닐 때 수평 적분 오차를 만들고 과도구속 위험이 있다(÷100에서 역효과 관측).
이 prior factor는 **절대값을 올바른 target(0)에 앵커**하고 `acc_w`는 물리값(Allan 실측)으로
두므로, 느린 실제 드리프트는 허용하면서 과대추정만 막는다.

## 파라미터 (config/elp_stereo_edie/vio_edie.yaml)

| 키 | 기본 | 의미 |
|---|---|---|
| `use_acc_bias_prior` | 0 | 사용 여부(0=기존 동작 유지) |
| `acc_bias_prior_w_xy` | 50.0 | 수평(ax,ay) 가중치(클수록 강하게 0) |
| `acc_bias_prior_w_z` | 0.0 | 수직(az) 가중치(이미 정확 → 0) |

## ⚠️ 검증 상태 — 미확정

- **bag A/B는 신뢰 불가**: 실시간 재생 비결정성으로 동일 config 반복 시 xy 변위 0.65~3.47m(±2.8m),
  원본 bag IMU 갭(최대 26ms)도 존재. 단일 run 비교로 효과/부작용을 판정할 수 없다.
- config-only(acc_w÷10) 흉내 실험에서 **연속주행(_old) z drift −74%**(1.08→0.28m)의 방향성은
  관측됐으나, xy 트레이드오프 주장은 run 노이즈로 철회됨.
- **확정 절차**: 로봇 **실시간(녹화 OFF)** 연속주행에서 `use_acc_bias_prior` on/off A/B + 진짜 GT로
  z drift 감소·xy 무손상 확인 후 `w_xy` 튜닝. 그때까지 **기본 OFF**.

## 관련

- `src/factor/acc_bias_prior_factor.h` — factor 구현
- `test/test_acc_bias_prior_factor.cpp` — 잔차·자코비안 회귀 테스트
- `doc/ZUPT.md` — 같은 z drift 문제를 정지구간에서 다루는 보완 메커니즘
