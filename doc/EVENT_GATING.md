# 이벤트 게이팅 (Phase 1: 다리각) — SW1-1837

## 문제

EDIE는 다리 인출입으로 몸체 자세(pitch)와 휠-IMU extrinsic이 바뀌는 로봇이다.
다리가 움직이는 동안 VINS의 세 가지 제약이 전부 **틀린 주장**을 한다:

| 제약 | 다리 이벤트 중 실제 | 제약의 주장 | 결과 |
|---|---|---|---|
| WheelFactor | 몸체가 회전/기울어짐 (휠은 정지) | v=w=0 | 모순이 자세로 전가 |
| PlaneFactor | 몸체 pitch가 실제로 변함 | roll/pitch 수평 + 고도 고정 | 실제 자세 변화에 저항 |
| VerticalVelocityFactor | 다리가 몸체를 들어올림 (vz≠0) | vz=0 | 수직 운동을 오차로 오인 |
| ZUPT | 휠 정지 ≠ 몸체 정지 | V=0 | 몸체 운동 중 속도 0 강제 |

**실측 근거 (2026-07-13, `odom_fix_check` bag 3-run)**: t=40~45s 다리 0.33rad 스윙 순간
VINS 자세 오차(정지구간 IMU 중력 대조)가 0.04° → **1.6~3.4°로 일괄 주입된 뒤 끝까지 유지**.
이것이 궤적 전역 tilt 4.32±0.50°의 원인. 반면 대회전 1375°는 무해(오차 불변),
바닥 경사도 무죄(≤0.6°). 즉 자세 오차는 서서히 새는 게 아니라 **이벤트에서 한 번에 주입**된다.

## 해법: factor-구간 skip

검증된 글리치 게이트(`use_wheel_vel_gate`)와 동일한 패턴 — **preintegration은 유지하고
그 시각 구간의 factor만 추가하지 않는다**(적분 일관성 유지, 구간은 IMU/비전이 받침).

- **신호**: ① `/joint_states`의 `left/right_leg_joint` 실측 각도(변화율 검출)
  ② 위치 명령 토픽(`/edie/{l,r}_leg_position_controller/command`, `std_msgs/Float64` — PassthroughController DataType) —
  실측보다 선행하므로 물리 반응 전에 게이트를 미리 연다.
- **구간 마킹**: 이벤트 [시작−`leg_pre_margin`, 종료+`leg_post_margin`].
  실측 검출이 늦는 문제는 소급 마진(pre)으로 보상.
- **게이팅 대상**: WheelFactor(구간), PlaneFactor(프레임), VerticalVelocityFactor(프레임), ZUPT(구간).
- **안전장치**: 연속 게이팅 상한 `gate_max_duration`(기본 2s) — 휠 factor는 필수 스케일
  앵커라(끄면 발산, 07-01 실측 |pos| 70m) 무한정 뺄 수 없다. 상한 초과 시 강제 해제하고,
  정지 샘플을 관측하기 전엔 재점화하지 않는다.

## 구성 요소

| 파일 | 역할 |
|---|---|
| `src/utility/leg_event_detector.h` | 구간 마킹 순수 로직 (ROS 미의존, gtest 대상) |
| `src/estimator/estimator.{h,cpp}` | `inputLegState/inputLegCommand/isLegGated` + 4개 factor 지점 skip |
| `src/estimator_nodelet.cpp` | joint_states(BEST_EFFORT)·명령(RELIABLE) 구독 → estimator 전달 |
| `src/utility/parameters.{h,cpp}` | config 파라미터 (기본 전부 off) |
| `test/test_leg_event_detector.cpp` | 계약 테스트 7건(LSB 플리커 회귀 포함) |

## 설정 (`vio_edie.yaml`)

```yaml
use_event_gating: 0     # 마스터. 0=기존 동작 그대로 (A/B 검증 후 기본값 결정)
gate_leg: 1
leg_rate_min: 0.05      # [rad/s]
leg_cmd_pos_min: 0.02   # [rad]
leg_pre_margin: 0.3     # [s]
leg_post_margin: 0.5    # [s]
gate_max_duration: 2.0  # [s]
leg_state_topic: "/joint_states"
leg_cmd_topic_l: "/edie/l_leg_position_controller/command"
leg_cmd_topic_r: "/edie/r_leg_position_controller/command"
```

발동 로그: `[EVENT-GATE]` (logger `vins_event_gate`, INFO 1s 스로틀 — 이벤트 없으면 무출력).

## 검증

1. **gtest**: `test_leg_event_detector` — 무동작/마진 마킹/명령 선행/무의미 명령/상한 강제해제·재무장/LSB 플리커 면역/정리 7건.
2. **재생 A/B (성공 판정)**: `odom_fix_check` bag, `use_event_gating` 0 vs 1, 3-run:
   - 40~46s 구간 `[EVENT-GATE]` 발동 확인
   - 전역 tilt 4.32±0.50° → 챔피언 분포(2.19±0.97°) 회복
   - xy-APE(0.091±0.016) 무손해, z범위(223±7mm) 악화 없음
   - 평가: `scripts/eval/plane_metrics.py` + 자세 오차 직접 측정(IMU 중력 대조)
3. **회귀**: 다리 이벤트 없는 구간에서 발동 0회, 지표 불변.

## 한계 / 후속 (Phase 2+)

- **`leg_rate_min`의 실제 의미(07-14)**: 다리각은 펌웨어가 정수 도(°) 단위로 보고한다
  (`fifo_comms.hpp` int32_t → `edie_hardware.cpp:381` ×π/180 = **구조적** 1°(0.0175rad) 격자,
  bag 3개 교차 실측으로도 확인). 따라서 순간 변화율은 '0 또는 스파이크(1°/dt)'뿐이다
  (스윙 중에도 0.05~1.0rad/s 샘플 0개) — 현재 입력에선 rate_min이 **잠자는 파라미터**
  (연속값 입력으로 바뀌면 노이즈/움직임을 가르는 진짜 문턱으로 살아남).
  스파이크 최저값은 0.87rad/s(실측 dt 중앙값 10ms·최대 20ms) → rate_min은 (0, 0.87)에서
  불감이며, 정착 판정의 실질 노브는 `leg_post_margin`. 스윙 중 스파이크 간격은 중앙값
  30ms·최대 50ms(실측) → post_margin 0.5s의 조기 닫힘 여지는 10배 여유로 없음.
  ※녹화 갭 bag에선 dt가 커져 경계가 더 내려가나 갭 구간은 휠 데이터도 없어 무의미.
  정착 중 플리커는 버스트당 최대 ~post_margin 연장 가능하나 실측상 연쇄 불가
  (124s 전수: 버스트 ≤0.05s, ~20s당 1회, 0.5s-연쇄 최장 0.05s) — 연장의 하드 상한은
  `gate_max_duration`이 보장.
- 이미 주입된 자세 오차의 **사후 교정은 범위 밖** — 게이팅은 주입 방지만 한다.
- 다리각 변화 후 휠-IMU extrinsic이 새 상수로 바뀌는 문제(FK 갱신)는 SW1-1836 영역.
- 슬립(휠-IMU 각속도 불일치), 범프(수직 acc), 경사(pitch rate) 이벤트는 Phase 2/3.
- `USE_BODY_NHC`(기본 off, 경사 실험용)는 이번 게이팅 대상에서 제외 — 활성화해 쓸 때 추가.
