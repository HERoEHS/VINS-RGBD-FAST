# SW2 Lifelong SLAM 조사 보고 (2026-08-03)

> 목적: 타 팀(SW2 Nav & Manipulation)의 lifelong SLAM 스택에서 EDIE VINS 개발에
> 재사용할 수 있는 설계·교훈을 조회한다. **조회·분석 전용 — 코드 변경 없음.**
> 원문 접근법과 상세 근거는 아래 "원문 위치" 참조.

## 원문 위치 (재현 방법)

- 저장소: `HERoEHS/alice_navigation`, **`feature/m2_camera` 브랜치**의
  `localization/lifelong_slam/` (main 브랜치엔 없음)
- 접근: GitHub MCP 토큰은 404(조직 미승인), **로컬 SSH 키는 통함** —
  `git clone --depth 1 --filter=blob:none git@github.com:HERoEHS/alice_navigation.git`
- 구성: `lifelong_slam_plan.md`(설계·결정 이력) + `lifelong_slam_setup.md`(세션 로그
  인덱스) + `sessions/` 상세 로그 7편 + `aeirobot_lifelong/` 코어 코드
- Linear 정본: SW2-241 (이슈 본문 = 문서 요약본, 담당 도현구)
- RTAB-Map 코어 개조는 서브모듈 `HERoEHS/rtabmap_0.23.4` (자체 deb 0.23.4-7까지)

## ⚠️ 제품화 리스크 — VINS 계열 GPL 배제 선례

plan.md 결정 이력 원문: **"GPL 계열 제외: ORB-SLAM3, VINS-Fusion, OpenVINS,
stella_vslam — 비공개 상용 배포와 충돌"**. SW2는 라이선스 사유로 VINS 계열을
탈락시키고 MIT-SPARK 스택(BSD/Apache, 라이선스 실확인 수행)을 채택했다.
**EDIE의 VINS-RGBD-FAST도 같은 계열이므로 제품화·비공개 배포 단계에서 동일
쟁점이 예상된다.** 조직 차원의 선례 판단이 이미 존재함을 기록해 둔다.

## EDIE 이식 후보 (우선순위)

### 1. 재초기화 정지 대기 게이트 — HANDOFF 미착수 항목 ⑤의 처방

SW2 보호관찰 기준선의 진화가 원형이다:
- 1안 "첫 LC 수락" 기준선 → localization 모드에선 LC가 상시라 **0.8초 만에 열려
  사실상 무방비** (실측, setup 3.17)
- 2안 **reanchor 모드**: map→odom이 **실제로 이동**(2cm/0.01rad)한 것을 확인 +
  최소 지연 20s 경과 → bag 검증에서 실제 pass 판정 산출 (3.64)

패턴 = **"상태 변화의 실증 + 최소 지연" 이중 요건**. 이벤트 발생 여부가 아니라
상태가 실제로 바뀌었음을 조건으로 삼는다. EDIE의 failure 후 재초기화(다리
애니메이션 중 착지로 R0 10° 오염 실증)에 같은 패턴 적용 가능:
"휠 정지 + 다리각 안정이 N초 연속 실증"될 때까지 static init 창 수집을 미룬다.
별도 계보로 ALICE 상태추정기(`~/alice_ws/src/aeirobot_state_estimator`, 로컬
스냅숏)의 초기화 시퀀스(양발 접지 1s 연속 유지 → IMU 100샘플 수집 → init)도
같은 처방이다.

### 2. 리셋 경로 가드 무력화 체크리스트

두 조직이 독립적으로 같은 버그 계보를 반복했다 — 상세는
[GUARD_RESET_PATH_CHECKLIST.md](GUARD_RESET_PATH_CHECKLIST.md).

### 3. soft 노이즈 인플레 선례 (게이팅 증폭 결함의 처방 후보)

ALICE 상태추정기는 접촉이 끊긴 발을 측정에서 제외하지 않고 **process noise를
contact/not-contact 두 값 사이에서 스위칭**한다(z는 별도 채널). EDIE에서 08-03
확정된 게이팅 증폭 결함(0.2s 다리 이벤트가 키프레임 구간 단위 skip 때문에 수 초짜리
휠 앵커를 통째 제거)의 처방 후보 "구간 분리/soft 인플레" 중 후자의 실전 선례.

### 4. SW1-1788 (맵 기반 전역 위치추정) 착수 시 직접 참고 목록

- **VoteEngine** (`persistence_core.py`, 141줄 순수 수학): 특징 3D를 depth에 투영해
  관통=부재표/일치=존재표/차단=기권. 방어 계층 — 경계 불연속 기권(이웃 4px depth
  산포), 프레임 건전성 게이트(존재율 <20% 프레임 통째 폐기), 히스테리시스(존재표가
  부재 카운트 −2), 다시점 요건(0.5m 격자 빈 ≥2), veto 비율 3:1
- **은퇴 = 좀비화**: 특징 전량을 `Feature_archive`로 이관, 노드·그래프 링크는 보존
  (맵 분열 위험 0, 가역). SQL 직접 삭제는 시각 사전이 깨져 금지(실사고 후 확립)
- **LC 확정 파라미터**: `Vis/PnPReprojError` 2, `Kp/MaxDepth` 10(카메라 유효거리
  기준), `RGBD/MaxOdomCacheSize` 30 — 오탐 LC의 진범은 문턱이 아니라 **PnP 관용**
  (base frame 점프 소멸을 사용자 화면으로 확인, 3.69)
- **핵심 실측**: 원시 부재 신호는 구분력 0(변화 bag 22.2% vs 무변화 bag 21.2%) —
  정탐/오탐 11.4배 분리는 전부 누적 필터가 만든다. **"신호가 아니라 필터가 성능"**.
  ground truth는 무변화 통제 bag으로 확보(지워진 것 = 정의상 전부 오탐, 3.65)

## 적용 한계 (자체 검증에서 정정한 것)

- **프레임 건전성 게이트는 EDIE 폭주 국면에 무효**: 이 게이트는 외부 고정
  기준(영속 특징맵) 대비 불일치를 재기 때문에 작동한다. EDIE 폭주의 핵심
  국면(창 전체 게이지 회전)은 창 내부 재투영 잔차가 동결됨을 이미 포렌식으로
  실증했으므로(mean 8.3px 고정), 같은 발상의 내부 게이트는 원리상 아무것도 못
  본다 — 10라운드 교훈⑦의 재확인. 유효 범위는 오염 주입 단계 한정(기존 1선
  StillMotionFactor와 중복 영역).
- SW2 스택 자체의 미결 리스크(도입 시 승계됨): rtabmap 17분 동결, 4dmap 저장
  817MB 스파이크, 은퇴 누적 폭주 미측정, 객체 경로 쿨다운의 공간 키 이관 미완.
- 환경 차이: ROS2 Jazzy + JetPack 7 기준(EDIE와 배포판 상이), Orin 성능 환산계수는
  미측정 가정 위에 있다고 원문 스스로 명시.

## 부수 산출물

- z 래칫 가드 v13 검증 부속 조사(같은 날): 가드 발동의 약 70~75%가 다리 움직임
  구간 내부·인접이며, 대형 z 환원(10mm+)은 다리 구간에 집중 — 다리 이벤트 → z 계단
  인과와 정합. 예외 1건(100.8s, 전 run 재현)은 주행 누적 회수 추정.
- Linear 타 팀 조회 결과: 워크스페이스 전 팀 읽기 가능. SW2 히어로즈컵
  로컬라이제이션(MCL+게이팅+헬프콜, 로컬 매니저 공분산 아비터), SW3 MHE(arrival
  cost ≈ marg prior 동형) 참고 가치.
