# 정지 + 동적 장애물 실험 프로토콜 v2 (SW1-1866, 07-29 현행화)

> v1(07-27)은 "취약한가?" 판정용이었다. 그 답은 이미 나왔다 — `edie_dynamic_obs` bag에서
> **yaw 영구 폭주 실증**(−90°/s 연속 회전, 종점까지 −2224°) 후 2계층 방어 커밋 `c4c8483`
> (StillMotionFactor + gaugeSlideGuard, 기본 ON, 재생에서 ±1.2°/7mm 완치).
> **v2의 목적 = 그 방어가 실기(로봇 실시간)에서도 작동하는지 최종 검증.**

## 녹화 준비
- 로봇에 **방어 포함 빌드 배포**(`c4c8483` 이후, `use_still_motion_lock: 1`·`use_gauge_slide_guard: 1` 기본)
- ⚠️ **/dev/shm에 녹화 후 복사** — 디스크 직녹화는 CM 스톨 이력에 더해, obs bag에서
  상시 수신 지터 실측됨(IMU p99 22.7ms vs 정상 4.9ms). v11처럼 깨끗하게 찍을 것
- 토픽: edie_gate_verify 시리즈와 동일 세트 (image_gray + depth + offset_imu + diff_drive odom + joint_states + tf)
- rviz 실시간 비교 원하면: `publish_vins_footprint_tf: 1`(기본) + `map2odom` alias → base_link vs vins/base_link

## 안무 — 실증된 폭주 유발 조건을 그대로 재현
obs bag에서 폭주를 일으킨 조건 = **초근접(0.3~0.5m) × 저속 통과 × 화면 60~80% 점령 ×
통과 후 근처에 서 있기** (특징점 풍부한 옷 — 줄무늬·체크 등 — 이면 더 강한 스트레스).

**시나리오 A — 실전 재현 (obs bag 동형, 필수)**: 주행 이력이 marg prior에 구워진 상태가
실제 발화 조건이었으므로 주행을 선행한다.
| 구간 | 행동 |
|---|---|
| A1 | 일반 주행 코스(스핀 포함) 후 시작점 복귀 |
| A2 | **완전 정지** 10s (조용한 기준선) |
| A3 ★ | 사람이 **0.3~0.5m 앞을 아주 천천히** 좌→우 통과(편도 ~8s, FOV 크게 가림) 후 **화면 가장자리에 30s 서 있기** |
| A4 | 사람 퇴장, 30s 정지 유지 (복원/잔존 판정) |

**시나리오 B — 전 구간 정지 (선택, 정량판정 보조)**: v1의 P0~P5 안무 그대로
(참값=무운동이라 GT 불요·판정 가장 깨끗). 시간 여유 있을 때만.

## 판정
- bag을 `~/ros2_ws/bag/`에 복사 → `bash scripts/eval/dynobs_eval.sh <bag이름>` (repo 루트, 재생 3run 자동)
- **합격(방어 ON)**: 정지 창 |Δyaw| < 2° · |Δxy| < 0.03m, 사람 퇴장 후 잔존 드리프트 없음,
  z·tilt 전가 없음 (재생 실측 근거: still-window ±1.2°/7mm)
- **대조(선택)**: yaml에서 `use_still_motion_lock: 0` + `use_gauge_slide_guard: 0`으로 재생하면
  base 거동 확인 가능 (obs bag에선 −4100°~−5000° 폭주)
- 실시간(라이브) 거동은 로봇에서 rviz로 직접 관찰 — 재생과 라이브는 발화 속도가 달랐던
  이력(90 vs 160°/s)이 있으므로 라이브 확인이 최종 심판
- 진단 필요 시: `VINS_SLIDE_DIST_LOG=1`(슬라이드 분포+정지 사유), `VINS_DYNOBS_LOG=1`(특징 구성)
