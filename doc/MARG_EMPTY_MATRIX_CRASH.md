# 마지널라이즈 빈 행렬 크래시(m=0 abort) — 수사 보고서

> 작성: 2026-09-04 · 상태: **D1+ⓒ 구현·회귀 테스트·음성검증·critic 2회·bag A/B(최종 바이너리) — ⓑ는 A/B 반례로 기각·분리** (TASK-20260904-vins-marg-empty-guard) · 이슈: **SW1-1883 = SW1-1881(동일 결함)** · 귀책: **업스트림 상속**(marginalize() m=0 무가드·10 s 게이트·SECOND_NEW 누적 모두 VINS-Mono 계열 원형) + 우리 gauge guard(SW1-1866)가 방아쇠를 키움

## 요약

`vins_estimator`가 Eigen assert(`Redux.h:411 "you are using an empty matrix"`)로 abort(exit −6)한다.
지점은 `factor/marginalization_factor.cpp` `MarginalizationInfo::marginalize()`의 `SelfAdjointEigenSolver(Amm)` —
`Amm = A.block(0,0,m,m)`이 **0×0**일 때. 실기 2건: 09-02 정지 중 사람 통과(SW1-1881, 코어덤프 확보), 09-04 도킹 접점
카메라 가림 후 후진(SW1-1883). 두 건은 트리거만 다르고 같은 결함이다. 도킹은 마지막 10~15 cm가 매번 카메라 가림(정상 상황)
이라 재시작으로 끝낼 수 없다.

## 원인 사슬 (코드 + 09-02 코어 gdb + 독립 critic 검증)

`m`은 factor 수가 아니라 **drop_set에 든 파라미터 블록 local size 합**이다. MARGIN_OLD에서 `Pose[0]`/`SpeedBias[0]`을
drop하는 factor가 하나도 없어야 m=0이 되므로, 아래 세 조건이 한 solve에 겹쳐야 한다.

| # | 조건 | 어떻게 생기나 | 09-02 코어 |
|---|---|---|---|
| 1 | prior의 drop_set이 빔 | (i) gauge guard 절제(비강체 `gaugeSlideGuard` / 강체 5연속)로 `last_marginalization_info=nullptr` (ii) prior는 있으나 Pose[0] 미포함(장구간 2연속) | nullptr, 절제 streak 2 |
| 2 | IMU·휠 factor 없음 | `pre_integrations[1]->sum_dt ≥ 게이트(10 s)` 탈락. 긴 구간이 슬롯 1에 오는 경로: **(a) SECOND_NEW 누적** — `slideWindow`가 버린 프레임 IMU/휠을 슬롯 9에 이어 붙여 정지(특징 ≥20·저시차) 중 무한 성장, 키프레임 8연속이면 슬롯 1 도달 **(b) 특징 0개 프레임 미전달** — `estimator_nodelet.cpp` `if (!image.empty())`로 processImage 미호출, 적분이 슬롯 10에 통째로 | `sum_dt[1]=167.03 s`, `[2]=0.21 s`, Headers[0]=…282, [1]=…449 (경로 a 확정 — 정지 167 s 동안 solve는 계속 돌았음) |
| 3 | 비전 factor 없음 | frame-0 특징이 후속 프레임에 미관측(가림 후 새 ID) 또는 전부 `is_dynamic` 낙인 | `last_track_num=1` |

**도킹 비정상 조건 메모(09-04 사용자).** 프로토타입 스테이션의 입구 턱·가이드 수정 중이라 조이스틱 수동 도킹이었고, roll이 기울어진 채 도킹·제자리 헛돎이 있었다. 크래시 구조(m=0)는 이와 무관(SW1-1881이 도킹 없이 같은 assert)하지만, 방아쇠(비강체 슬라이드)에는 기여했을 수 있다. 특히 **plane factor의 '법선=세계 위쪽 고정' 가정은 roll 기울어짐과 충돌**한다 — 리비전 스테이션에서 정상 자율 도킹으로 재현 bag을 찍은 뒤 ⓐ/D2/D3를 설계한다.

**가드는 분단 중 무력.** 후 부분창은 prior가 있든 없든 자유롭다(prior는 전 부분창만 잡음). 절제(종전)는 전 부분창 앵커까지 끊어 m=0만 만들고, 보류(ⓑ)는 로그+역변환 미적용이 전부다. 도킹처럼 이동 중이라 STILL-DRIFT·CUM-GUARD가 발동하지 않는 상황에서 후 부분창을 묶는 것은 **ⓐ 체인 보존의 영역**이며 가드 조치로는 해결되지 않는다.

**가드와 한 뿌리.** 최적화(`optimization()`)도 같은 게이트로 IMU·휠 factor를 빼므로, 게이트를 넘는 슬롯이 있으면 창이
전/후 부분창으로 **분단**된다(10 s 넘는 모든 정지에서 매 solve 상시). 후 부분창은 `StillMotionFactor`(게이트 없음)·plane만으로
묶여 통째로 돌 수 있고, 그 모습이 가드에는 '비강체 슬라이드'(한 프레임만 다름)로 보인다. 가드의 "prior가 한 프레임을 당긴다"는
진단은 이 상황에선 **오진**이고, 절제는 전 부분창의 마지막 앵커(prior)까지 끊어 조건 1을 스스로 완성한다(자충).
절제 3연속이면 escalation(`guard_escalation_max`)이 재부팅으로 선점하므로 실기 결과는 **크래시/재부팅/생존 3원**이다
(09-02는 streak 2에서 크래시 = 재부팅과 1 solve 차).

## 처방 계층 (critic 판정, 머지 범위 = D1+ⓒ)

| 층 | 처방 | 구현 |
|---|---|---|
| 증상·필수 | **D1** `marginalize()` m/n 분기 — n==0(keep 없음)이면 `false` 반환, 호출부 `discardMarginalizationPrior()`가 새 info·옛 prior 폐기·`MarginalizationFactor` 생성 금지(n==0 factor는 잔차 0개 → ceres CHECK 재abort). m==0·n>0은 Schur 생략 통과(정보 보존). | `marginalization_factor.{h,cpp}`, `estimator.cpp` MARGIN_OLD/SECOND_NEW 호출부 |
| 근본 일부(기각) | **ⓑ** 분단 중 비강체 절제 보류 — 구현·A/B했으나 **기각**: 최종 바이너리 v16 3-run에서 교차 xy 0.040 m > B내부 0.027(무손실 미달), r2에서 **902 m 폭주 pose 1회 발행**(보류 → 누적 클램프 0.227 m → 강체 5연속 경로 절제 → 분단 중 prior 소실로 전 부분창 완전 자유 → 폭주 → STILL-DRIFT 재부팅). 보류를 절제 횟수에 안 세니 escalation 백스톱이 늦어진 것이 노출 증가 원인. 분단 중 모든 절제 경로 보류(3-3)와 묶어 별도 태스크. | 코드 제거, `preintGapSlot()`은 진단(MARG-GUARD 로그)용으로만 잔존 |
| 규약 | **ⓒ** 하드코딩 10.0 4곳 → YAML `preint_max_dt_s`(기본 10.0, 동작 불변) | `parameters.{h,cpp}`, `vio_edie.yaml` |
| 근본 | ⓐ 게이트 탈락 구간의 체인 대체(휠·still-lock·약한 상대 prior)를 최적화·marg 양쪽에 남겨 m>0 보장 | 별도 설계 |
| 예방 | D2(i) 슬롯 9 적분 상한 강제 키프레임 / D2(ii) 특징 0개 프레임 전달(NON_LINEAR 한정) — 정지 창 회전 부작용, v16 bag A/B 필수 | 별도 태스크 |
| 방아쇠 | D3 휠 헛돎 오염원 취급(도킹 제안) — m=0 사슬과 무관. 기준은 dl≫dr이 아니라 휠 yaw vs gyro yaw 불일치 | 도킹 품질 |

**D1의 주장 범위 = "abort → 생존"까지.** D1 후 solve는 prior 없이 돈다. 이것이 무해하다는 증거는 없다 — 반대로 ⓑ A/B의 A3 r2가
보여주듯 **분단 중 prior가 없으면 전 부분창이 IMU 링크도 prior도 없이 완전 자유**가 되어 902 m급 pose가 발행될 수 있다(가드 절제로
생긴 상태이지만 D1 직후 상태와 구조가 같다). 정지 중이면 STILL-DRIFT 발산 가드가, 이동 중이면 failureDetection(FAILURE_DP_MAX)만이
백스톱이다. v16 bag은 m=0에 도달하지 않아(MARG-GUARD 0/13런) **D1 이후 동작은 미검증**이며, 도킹 접점 재현 bag(프로토콜 v2)이 있어야
판정할 수 있다. m==0·n>0 통과 분기도 로그를 남겨 실기에서 발동 여부를 볼 수 있게 했다. 다음 MARGIN_OLD가 IMU·비전
factor로 prior를 재건한다. 단 가림 후 창은 가림 후 프레임만 남고 prior는 상대 정보만 가지므로 월드 앵커는 "그 순간 위치"에
고정된다 — 인계문 §2-3 "느리게 틀리는 것 허용"의 실제 모습이며, ⓐ·D2가 다루는 영역.

## 검증

- 회귀 테스트 `test/test_marginalization_empty.cpp`: T1 factors 0개(코어 상태) → false, T2 전 블록 drop(m>0,n=0) → false,
  T3 drop 없음(m=0,n>0) → 통과·정보 보존, T4 정상 경로 불변, T5·T6 진단 함수 `preintGapSlot`(코어 형상 분단 슬롯 1 / 무분단·nullptr 안전).
  `test_yaw_slide_guard.cpp` FirstGapSlot(경계값 = 통과). 전체 스위트 164/164.
- 음성검증: D1 두 분기를 되돌린 빌드에서 T1·T2 abort(assert 메시지 재현) — 결과는 EXEC_PLAN §2.
- bag A/B: `edie_gate_verify_v16_play` 3-run × 2(A=D1+ⓒ 최종 바이너리, B=265fc6b) — 결과는 EXEC_PLAN §2 최종 항목. 이 bag은 m=0에
  도달하지 않으므로(MARG-GUARD 0) 여기서 확인되는 것은 "기존 동작 무손실"이지 "크래시 이후 동작"이 아니다. ⓑ 포함 빌드의 A/B(교차
  0.040 m, 902 m 폭주 1회)는 기각 근거로 보존(scratchpad ab1883_A3).
- 실기: 도킹 세션 재현 프로토콜 v2(`~/ros2_ws/bag/dock_seat_0904/PROTOCOL_VINS_repro_bag.md`) A런 abort 0.

## 자료

- 인계문·크래시 로그·critic 보고 전문·프로토콜: `~/ros2_ws/bag/dock_seat_0904/`
- 09-02 코어: 로봇 `~/vins_crash_0902/core.vins_estimator_.11746` (바이너리 `stat -L` 09-01 21:51 빌드와 심볼 일치)
