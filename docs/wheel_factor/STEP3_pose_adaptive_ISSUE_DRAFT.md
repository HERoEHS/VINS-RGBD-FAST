# [VIO] 휠 factor 자세 적응형(pose-adaptive) 융합 — Linear 이슈 초안

> 추후 생성용 드래프트. 현 작업(SW1-1829)은 **기본(공칭) 자세 단일 주행**만 다루며,
> 다자세 주행 대응은 본 항목으로 분리한다.

## 제목
[VIO] 휠 factor 자세 적응형 융합 — 다리 자세별 extrinsic/intrinsic

## 연관
- 부모/선행: SW1-1829 (휠 factor tight-coupling, Step1 고정 calib)
- 선행 검증: OpenLORIS office1-1 A/B에서 무발산 + scale 오차 −33% 확인

## 배경 / 문제
EDIE는 다리(leg) revolute joint(좌우 독립, axis=Y)로 바퀴를 인출입하여
**여러 자세에서 각각 주행**한다(차체 roll/pitch/z 자유도 보유).
→ 휠-IMU extrinsic(`body_T_wheel`=rio/tio)과 intrinsic(sx/sy/sw, 유효 트레드)이
자세(다리각)마다 다른 상수다. 현재 estimator는 상수 1세트만 지원.

단일 online extrinsic 추정은 부적합: 자세 전환 시 추정값이 움직이는 표적을 쫓아
매번 재수렴하며 오염되고, 한 자세 내부 주행은 평면이라 6-DOF 관측성이 약하다.

## 범위 (Step3)
- [ ] `joint_states`(다리각) 구독 → 현재 자세 식별 (estimator_nodelet)
- [ ] extrinsic: 다리각 **FK 산출**로 자세별 `body_T_wheel` 선택 (단일 online 금지)
- [ ] intrinsic(sx/sy/sw): 자세별 1회 오프라인 캘리브 → lookup 테이블
- [ ] 자세 전환 과도구간: 휠 factor **게이팅(off)** + 휠 preintegration 리셋
- [ ] td_wheel: 자세 무관 → 상시 online 허용
- [ ] estimator: rio/tio·sx/sy/sw를 상수 1세트 → per-pose 선택 구조로 변경

## 전제조건
- EDIE 다자세 주행 odom + joint_states 포함 bag
- DESIGN.md 동작정의 반영 (동작 정의 변경 → 사용자 확인 필수, CLAUDE.md 위임 정책)

## 현재 결정 (스코프 아웃)
기본(공칭) 자세 단일 calib로 우선 발산 여부 검증(SW1-1829).
다자세 대응은 본 이슈로 분리하여 추후 진행.
