# all_image_frame 정지 중 메모리 누수 — 수사 보고서

> 작성: 2026-08-31 · 상태: **수정 구현 + bag A/B 검증 합격, KPI 3-run 관문 대기** (TASK-20260831-vins-still-frame-leak) · 귀책: **업스트림 상속** (jianhengLiu/VINS-RGBD-FAST 원본부터 존재, VINS-Mono 계열 공통)

## 요약

로봇이 **정지해 있는 동안** vins_estimator 힙이 **분당 ~19MB씩 선형 증가**한다.
원인은 `all_image_frame` 맵: 매 프레임 무조건 삽입되지만 삭제는 **키프레임 마지널라이즈(MARGIN_OLD) 분기에만** 있어서,
정지 중(= 시차 미달로 키프레임이 안 생기는 동안)에는 삽입만 15Hz로 무한 누적된다.
움직이기 시작하면 MARGIN_OLD가 돌며 일괄 해제되므로 **주행 위주 사용에서는 드러나지 않고**,
반려로봇처럼 켠 채 세워두는 사용 방식에서만 치명적이다 (시간당 ~1.2GB, 8시간 방치 시 ~9GB).

## 발견 경위

2026-08-31 실기(aeirobot, RAM 31.8GB)에서 풀스택 가동 상태로 노드별 RAM을 측정하던 중,
vins_estimator RSS만 유일하게 선형 증가하는 것을 발견 → 31분 힙 로거로 판별.

## 실측 데이터 (10초 간격 로거, `/dev/shm` 기록)

| 시각 | 힙(Anonymous) | 비고 |
|---|---|---|
| 19:46:18 | 275MB | 로거 시작 (부팅 19:33부터 로봇 정지 지속) |
| 19:55 | 448MB | 기울기 분당 +19MB, 완전 선형 |
| 20:04:24 | **627MB** | 피크 |
| 20:05:25 | 615MB | **-12MB 후 평탄 전환** |

평탄 전환 시각(20:04:45)에 세 사건이 동시 발생:

1. 사용자가 **조이스틱으로 로봇을 이동**시킴 (사용자 증언으로 확인)
2. CUM-GUARD-YAW(정지 창 가드) 로그가 8,220회 연발 끝에 침묵 — 정지 창 종료의 독립 증거
3. 힙 증가 정지 + RSS 12MB 하강

해석: 이동 → 시차 발생 → 키프레임 생성 → MARGIN_OLD → **31분치 누적 일괄 delete**.
해제분은 glibc 할당기가 보유·재사용하므로 RSS는 12MB만 내려가고 평탄해진다 — 관측과 정확히 일치.

증가율 산수: `ImageFrame`(특징점 맵 사본 ~150개 × ~130B + IMU `IntegrationBase` 수 KB) ≈ 프레임당 20~25KB × 15Hz ≈ **분당 18~23MB** — 실측 19MB/분과 일치.

## 코드 근거 (커밋 2b3fc9e 기준)

- **삽입(무조건)**: `vins_estimator/src/estimator/estimator.cpp:444-446`
  ```cpp
  ImageFrame imageframe(image, rclcpp::Time(header.stamp).seconds());
  imageframe.pre_integration = tmp_pre_integration;
  all_image_frame.insert(make_pair(rclcpp::Time(header.stamp).seconds(), imageframe));
  ```
- **삭제(MARGIN_OLD 분기에만)**: `estimator.cpp:3419-3429` — `t_0`(슬라이딩 윈도우 최고령 키프레임 시각) 이전을 일괄 delete+erase
- **SECOND_NEW 분기(3433-3497): 삭제 없음** ← 결함의 몸통. 정지 중엔 모든 프레임이 이 분기로 감
- 참고: 휠 쪽 `tmp_wheel_pre_integration`은 447-454행에서 즉시 delete되어 무관 (IMU `pre_integration`만 ImageFrame에 실려 누적됨)

## 귀책 판정 — 업스트림 상속

원본 [jianhengLiu/VINS-RGBD-FAST](https://github.com/jianhengLiu/VINS-RGBD-FAST) main 브랜치와 대조 (2026-08-31):

| 지점 | 업스트림 | 우리 포트 |
|---|---|---|
| 무조건 삽입 | `estimator.cpp:205` | `:446` — 동일 |
| 삭제 (MARGIN_OLD만) | `:1643-1644` | `:3428-3429` — 동일 |
| SECOND_NEW 분기 erase | **없음** | **없음** — 동일 |

이 slideWindow 구조는 VINS-Mono → VINS-RGBD → VINS-RGBD-FAST로 상속된 것으로, **계열 전체 공통 결함**이다.
업스트림에서 안 알려진 이유: 통상 사용(bag 재생 수 분)에서는 노출 조건("정지 수십 분 연속 가동")이 성립하지 않음.
수정 확정 시 업스트림 issue/PR 기여 후보.

## 영향 평가

정확한 성격: **고전적 누수가 아니라 "정지 구간 한정 무한 누적 + 이동 시 회수"**다.
움직이면 MARGIN_OLD가 누적분을 일괄 해제하므로 주행 위주 사용에선 드러나지 않는다.
그럼에도 문제인 이유:

- **회수는 이동이 있어야만 발생** — 반려로봇의 지배 상태가 정지 대기라, 방치 시 시간당 1.2GB가
  회수 기회 없이 쌓임. 최악치는 평균이 아니라 **최장 정지 구간**이 결정
- **회수돼도 OS로는 반환 안 됨** — 실측: 340MB 누적 후 이동 시 RSS는 12MB만 하강.
  해제분은 glibc 할당기가 보유·재사용할 뿐이라 **프로세스 점유는 역대 최장 정지 구간의
  고수위에 래칫**됨. RAM 사이징은 이 고수위 기준으로 해야 함
- **RAM 사이징(원가 절감) 검토의 선결 차단 요인**: 누적 있는 시스템은 사이징 무의미
- 기능(자세 추정)에는 무영향 — `all_image_frame`은 비선형 단계에서 소비되지 않는 초기화용 데이터

## 처방 (TASK-20260831-vins-still-frame-leak에서 구현)

**SECOND_NEW 분기에서 버려지는 차차신 프레임의 `all_image_frame` 항목을 즉시 제거** —
`solver_flag == NON_LINEAR`일 때만 (사전적분 delete 포함, find 가드).

- ⚠️ 초안이었던 "`Headers[0]` 이전 가지치기"는 **무효**: 정지 중 누적 항목의 스탬프는
  윈도우 시간 범위 *안*(Headers[0]보다 최신)이라 한 개도 안 지워진다. 누적의 본질은
  "오래된 항목 잔존"이 아니라 "버려진 비키프레임 항목의 삭제 경로 부재"다.
- INITIAL 단계는 게이트로 제외 — 초기화는 비키프레임도 IMU 정렬(`visualInitialAlign` 계열)에 쓴다.
- MARGIN_OLD의 `find(t_0)` 무방비 역참조(`:3420-3421`)와는 충돌 없음 — t_0는 항상
  키프레임(SECOND_NEW로 버려진 적 없는 프레임)이라 항목이 보존된다.
- 검증용 진단: slideWindow 말미 `[AIF-SIZE]` DEBUG 스로틀 로그 (평시 무음).
  정상 상한 ≈ WINDOW_SIZE+1 + 키프레임 사이 비키프레임 소수.

## 검증 결과 (08-31 bag A/B — 합격)

`edie_gate_verify_v16_play` 438초 전체 재생 × 2 (A=수정 빌드, B=수정 전), 격리 도메인 + localhost 전용:

| 항목 | A (수정) | B (수정 전) | 판정 |
|---|---|---|---|
| AIF-SIZE 최대 | **10** (윈도우 크기, 내내 고정) | (미계측 — 진단은 수정판에만) | ✅ **직접 증거** — 소거법 판정 승격 |
| 힙 종료값 / 형태 | 50MB, **마지막 2분 평탄**(기울기→0) | 60MB, **종료까지 상승**(2.4MB/분) | ✅ 누적 차단 |
| init / odometry | 성공 2회 / 9.9Hz | 성공 3회 / 9.9Hz | ✅ 무회귀 (횟수 차는 이 bag의 기지 비결정성) |

각주: B의 2.4MB/분이 로봇 실측 19MB/분보다 작은 이유 — 이 bag은 동적 장애물(사람 이동)로
간헐 키프레임·재초기화가 누적을 주기적으로 씻어냄. **순수 정지(반려로봇 방치)가 최악 조건**이며
그 조건에서 수정 후 누적은 원천 차단(상한 10)된다.

## KPI 3-run 회귀 (09-01 — 합격)

dynobs_eval.sh 3-run × 2벌(A=수정, B=수정 전), 같은 bag, 도메인 격리. **무손실 정량 확정**:
궤적 쌍별 전 시점 대응(~4,340점/쌍)에서 빌드 간(교차 9쌍) xy 차이 중앙 **0.042m** ≤ 같은 빌드
재생 노이즈(B내부 0.048m, A내부 0.084m) — 수정 영향이 재생 비결정성 밑에 묻힘. run당 pose 수
동일(4,353). RMS 1.4m급 이상치 쌍은 r2 런들의 기지 폭주 시드 분기(양 빌드 공통)로 빌드 무관.

## 회귀 테스트 + critic 리뷰 (09-01)

- **회귀 테스트 4건** `vins_estimator/test/test_all_image_frame_prune.cpp` (vins_lib 실물 링크):
  ①SECOND_NEW 즉시 제거 ②500회 연속 SECOND_NEW 유계 ③INITIAL 보존 ④find 가드 무해.
  **음성 검증 완료** — 수정 걷어낸 빌드에서 ①② 실패·③④ 통과(설계 그대로). 전체 스위트 151/151.
- **독립 critic 리뷰**: 반증 8항목(t_drop 정확성·소비자 부재·이중 해제·find(t_0) 상호작용·
  INITIAL 경계·clearState 경로·진단 로그·규칙) **전건 반증 실패 = 건전**. minor 주석 정정 반영.

## ⚠️ critic이 발견한 별도 잠복 버그 (이번 수정과 무관, 별도 이슈 후보)

`estimator.cpp:545-551` static init 경로의 `for (auto &frame_it : all_image_frame) { ... Rs[i]; i++ }`
루프는 맵 크기 == 창 크기를 가정하고 i를 무제한 증가시킨다. INITIAL 중 워밍업 홀드가 SECOND_NEW
slideWindow를 반복하면 맵이 창(11)보다 커져 **배열 밖 읽기(UB)** → garbage R/T가
solveGyroscopeBias의 Bg 추정에 유입될 수 있다. 업스트림 상속 결함이며 이번 수정은 INITIAL 동작을
바꾸지 않아 현상 유지 — 후속 이슈로 기록 권고.

## 남은 검증

1. **실기 30분 정지 재측정**: 로봇 배포 후 힙 평탄 확인 (같은 로거 재사용, `/dev/shm` 기록 —
   디스크 I/O 스톨 회피)

## 재현·측정 방법

```bash
# 힙 성분 분리 로거 (RSS만 보면 DDS 공유메모리 착시와 섞임)
for i in $(seq 1 186); do
  awk -v t=$(date +%H:%M:%S) '/^Rss:/{r=$2} /^Anonymous:/{a=$2} END{print t","r","a}' \
    /proc/<PID>/smaps_rollup >> /dev/shm/vins_rss_log.csv
  sleep 10
done
```

조건: 로봇(또는 bag) **완전 정지** 유지. 판별 신호 = 힙(Anonymous) 선형 증가, 이동 시 즉시 평탄.
주의: 노드 RSS ~520MB 기준선은 FastDDS 공유메모리 세그먼트(참여자당 513MB × 상호 매핑) 착시 — 힙 성분으로 판별할 것.
