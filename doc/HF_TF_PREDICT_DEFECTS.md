# 분석: 고주기(HF) body TF의 IMU 예측 경로 결함 2건 (upstream 계승, 2026-08-19)

> 상태: **수정 완료 + bag 실측 검증 합격(08-19, `15c75b8` #SW1-1872)** — 결과는 §10, 잔여 결정은 §7
> 대상: `vins_estimator/src/estimator/estimator.cpp` `updateLatestStates()` / `predict()`
> 출처: upstream VINS-RGBD-FAST(jianhengLiu/main) 원본에 동일 코드 존재, blame `39dcb63d Chris Liu 2022-06-18`
> — 팀이 만든 결함이 아니라 **물려받은 것**. 기준 구현은 VINS-Fusion(원 계보).
> 발견 경위: 도킹이 `odom→base_vins`(HF 경로) TF를 앵커·전파에 쓰려는 시점에
> "HF TF가 정말 IMU 예측을 최적화 해로 보정한 값인가"를 코드 레벨로 검증하다 발견.
> 관련: `publish_odom_base_vins_tf`(6ea9b5b), `publish_hf_body_tf`(SW1-1837), 도킹 좌표계 갭.

## 1. 왜 지금 이게 문제인가

도킹은 CCTag 검출(~4Hz)로 앵커를 찍고, 다음 검출까지 오도메트리로 전파한다. 앵커와 전파가
**같은 추정기**를 써야 드리프트가 상쇄되므로 VINS를 `odom→base_vins` 간선으로 odom 트리에
넣었다(6ea9b5b). 그런데 station_docking은 이미지 시각(`msg->header.stamp`)으로 TF를 조회하고
타임아웃이 0.1s(싱글스레드 executor라 늘릴 수 없음)이므로, VINS 저주기 경로(최적화 후 발행,
100~500ms 지연)로는 조회가 자주 실패한다. **HF 경로(IMU 전파, 수 ms 지연)를 켜야 성립**한다.

즉 HF 예측 경로의 품질이 곧 도킹 전파 정밀도가 된다. 그래서 그 경로를 코드로 뜯어봤다.

## 2. HF TF 생성 경로 — 확인된 구조 (정상 부분)

> ⚠️ **순차 파이프라인이 아니다.** 두 스레드가 공유 변수 세 개를 사이에 두고 각자 루프를 돈다.
> "예측 → 발행 → 보정" 순으로 읽으면 "발행값은 미보정"으로 오독된다 — 실제 HF TF 값은
> **직전에 끝난 최적화의 해 ⊕ 그 이후 도착한 IMU 적분**이다.

### 2.1 등장하는 변수 (제3자용 용어 설명)

| 이름 | 종류 | 뜻 | 선언 |
|---|---|---|---|
| `imu_buf` | 큐 | 원시 IMU 샘플(시각, 가속도, 각속도) 대기열. IMU 콜백이 넣고, 프레임 처리가 꺼내 쓴다 | estimator.h:381 |
| `latest_P / latest_Q / latest_V` | 위치·자세·속도 | **고주기 예측 상태**. IMU마다 전진하고, 최적화 끝날 때마다 최적 해로 덮어써짐. HF TF·`imu_propagate` 토픽·`odom→base_vins`가 이 값을 발행 | estimator.h:399~ |
| `latest_Ba / latest_Bg` | 바이어스 | 예측 적분에 쓰는 가속도계·자이로 바이어스(최적화 해에서 복사) | estimator.h:399~ |
| `Ps / Rs / Vs / Bas / Bgs [i]` | 배열 | **슬라이딩 창 최적화 상태**(창 안 각 프레임의 위치·자세·속도·바이어스). `[frame_count]`가 최신 프레임 | estimator.h:167~ |
| `acc_0 / gyr_0` | 벡터 | 창 프리인테그레이션이 "직전에 소비한 IMU 한 샘플". 중점 적분[^mid]의 이전 값으로 쓰려고 들고 있음 | estimator.h:297 |
| `m_imu` | 뮤텍스 | `imu_buf`를 지키는 자물쇠 — 두 스레드가 동시에 큐를 만지지 못하게 함 | estimator.h:376 |
| `m_propagate` | 뮤텍스 | `latest_*` 를 지키는 자물쇠 — 예측(A)과 재기저(B)가 동시에 덮어쓰지 못하게 함 | estimator.h:376 |

> 뮤텍스(mutex) = "한 번에 한 스레드만 들어갈 수 있는 문". `lock()`으로 잠그고 들어가 작업한 뒤
> `unlock()`으로 연다. 자물쇠 없이 두 스레드가 같은 변수를 읽고 쓰면 반쯤 갱신된 값을 읽는 사고가 난다.

[^mid]: **중점 적분(midpoint / 사다리꼴 적분)** — IMU는 ~2.6ms 간격 샘플이라 그 사이 움직임은 근사해야 한다.
    구간 [tₖ, tₖ₊₁]에서 시작 값 하나(`aₖ`)만 쓰면 오일러(1차 오차), **양 끝 평균 ½(aₖ+aₖ₊₁)**을 쓰면
    중점(2차 오차)이다. `predict :3734-3740`이 후자다: `un_acc_0`(이전 샘플 `acc_0`을 이전 자세로 회전)
    → 각속도 평균으로 자세 먼저 전진 → `un_acc_1`(현재 샘플을 새 자세로 회전) → `un_acc = ½(un_acc_0+un_acc_1)`.
    **양 끝이 다 필요하므로 "이전 샘플"을 반드시 기억해야** 하는데, `predict()`는 자기 것이 없어 창 스레드의
    `acc_0/gyr_0`를 빌려 쓴다 — 이것이 결함 ②다(§4). VINS-Fusion이 `latest_acc_0/latest_gyr_0`를 예측
    전용으로 따로 두는 이유.

### 2.2 두 스레드와 공유 변수

```
[스레드 A — IMU 콜백, ~380Hz]                              [스레드 B — 프레임 처리, ~14Hz]
inputIMU :3554
  ├ imu_buf.push :3558 ────(m_imu)──── A→B ────────▶  getIMUInterval :3820   창용 IMU pop(:3835/:3840)
  ├ predict :3723   latest_* += IMU 1스텝                   processIMU :239        창 프리인테그레이션
  │     ▲ 읽음: acc_0/gyr_0 ◀──── B→A · 뮤텍스 없음 ──────────┘ :273-274 갱신          ← 결함 ② 원인
  └ pubLatestOdometry  latest_P/Q → HF TF                  optimization :602      창 최적화 → Ps/Rs/Vs
        ▲                                                   updateLatestStates :3629
        │                                                     latest_* = Ps/Rs/Vs/Bas/Bgs[frame_count] :3633-3637  ← 재기저
        └──────── latest_* (m_propagate :3566/:3631) ◀── B→A ──  + imu_buf 잔여분(m_imu :3641) 재적분         ← 결함 ① 위치
```

| 공유 변수 | 방향 | 쓰는 쪽 | 읽는 쪽 | 자물쇠 |
|---|---|---|---|---|
| `imu_buf` | **A → B** | A: `inputIMU :3558` push | B: `getIMUInterval :3835/:3840` pop(창용) · `updateLatestStates :3641` 잔여분 복사(재적분용) | `m_imu` |
| `latest_*` | **B → A** (재기저) · A 자체(예측) | B: `updateLatestStates :3633-3637` · A: `predict :3736-3740` | A: `pubLatestOdometry` | `m_propagate` |
| `acc_0/gyr_0` | **B → A** | B: `processIMU :273-274` | A: `predict :3734-3735` (중점 적분의 이전 값으로) | **없음** |

### 2.3 단계별 역할

| 스레드 | 단계 | 위치 | 하는 일 |
|---|---|---|---|
| A | 적재 | `inputIMU :3558` | 원시 IMU를 `imu_buf`에 push (B가 소비) |
| A | 예측 | `predict :3723-3741` (호출 `:3567`) | IMU마다 중점 적분으로 `latest_P/Q/V` 전진. 바이어스·중력 제거 |
| A | 발행 | `pubLatestOdometry`(visualization.cpp) | 그 시점 `latest_P/Q` → `map→body`(+`odom→base_vins`) HF TF, 스로틀·LPF(`hf_body_tf_tau`) |
| B | IMU 소비 | `getIMUInterval :3820` (pop `:3835/:3840`) | 창에 쓸 IMU를 `imu_buf`에서 꺼냄 → 큐에 남는 건 "마지막 프레임 이후 ~ 지금" |
| B | 창 처리 | `processIMU :239` | 프리인테그레이션. `acc_0/gyr_0 :273-274` 갱신 |
| B | 최적화 | `optimization :602` (init 경로 `:555/:574`) | 창 최적화 → `Ps/Rs/Vs/Bas/Bgs` |
| B | 재기저 | `updateLatestStates :3629-3649` (호출 `:656` 정상 루프, `:556/:575` init) | `latest_* = Ps/Rs/Vs/Bas/Bgs[frame_count]`로 **최적 해 덮어쓰기** 후, `imu_buf` 잔여 IMU 재적분 |

### 2.4 동작 요약

A는 B와 무관하게 계속 예측·발행한다. B가 최적화를 끝낼 때마다 `latest_*`를 최적 해로 덮어쓰고,
그 사이(최적화 대상 마지막 프레임 이후) 쌓인 IMU를 재적분한다. 그 다음 A의 `predict`는 덮어쓰인
값에서 이어간다. → 임의 시각의 HF TF = **직전 최적 해 ⊕ 그 이후 IMU 적분**. 이것이 "IMU 예측을
최적화 해로 보정한 값"의 정확한 의미다. HF 블록 주석(visualization.cpp)의 "매 최적화 완료 시 예측이
보정값으로 재기저되며 mm·0.0x° 미세 점프"가 B의 재기저 순간이다.

결함 ①은 B의 재적분 루프 안(잔여 `imu_buf`를 읽는 방식)에, 결함 ②는 `acc_0/gyr_0`의 자물쇠 없는
B→A 공유에 각각 위치한다.

> ✅ **수정 커밋 `15c75b8`(#SW1-1872)**: 그림의 ①은 `hf_predict::forEachSample`(표본 선택 계약)로,
> ②는 예측 전용 `latest_acc_0/latest_gyr_0` 멤버 분리로 해소 — 순수 수학부는 `utility/hf_predict.h`.

## 3. 결함 ① — 재적분 루프가 acc/gyr을 첫 샘플로 고정

### 코드 (`estimator.cpp:3641-3646`, HEAD 6ea9b5b)
```cpp
queue<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> tmp_imu_buf = imu_buf;
for (; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
    predict(tmp_imu_buf.front().first,        // 시각: tmp 큐 → 매 반복 진행 ✓
            imu_buf.front().second.first,     // acc : imu_buf.front() → 루프 내내 고정 ✗
            imu_buf.front().second.second);   // gyr : imu_buf.front() → 루프 내내 고정 ✗
```

### 3소스 대조
| 소스 | 루프 본문 | 판정 |
|---|---|---|
| 우리 포크 `:3643-3645` | 위와 같음 | 시각 진행 / 값 고정 |
| upstream VINS-RGBD-FAST `main:1782-1784` | **동일** (`for (sensor_msgs::ImuConstPtr tmp_imu_msg; ...)` — VINS-Mono 시절 변수 잔재까지 동일) | 물려받음 |
| VINS-Fusion `updateLatestStates` | `acc = tmp_accBuf.front().second; gyr = tmp_gyrBuf.front().second; fastPredictIMU(t, acc, gyr); tmp_*.pop();` | **매 샘플** 사용 |

VINS-Fusion과 놓고 보면 의도는 "같은 tmp 큐에서 시각과 값을 함께 꺼내기"이고, 값 쪽만
`imu_buf.front()`로 잘못 적힌 것이다(옮겨 적다 생긴 오타로 보임).

### 영향
- 최적화 지연 창(100~500ms; IMU 382Hz면 38~190샘플)을 **IMU 한 개 값으로 zero-order-hold 적분**.
- 정지·등속: 무해. 가감속·회전 중: 재기저 직후 예측이 실제와 어긋나고 다음 재기저(~70ms 후)까지 남음.
- 크기 추정: 창 안에서 가속도가 Δa 변했다면 위치 오차 ≈ ½·Δa·T². Δa 0.5 m/s², T 0.3s → **~2cm**.
  도킹 최종 접근(느리고 일정)에선 mm 이하.
- **누적되지 않음** — 매 재기저마다 최적 해로 다시 붙잡힘. HF LPF(tau 0.1s)가 일부 가림.
- 저주기 경로(`pubTF`)에는 무관.

## 4. 결함 ② — 예측 경로가 창 처리용 `acc_0/gyr_0`를 "이전 샘플"로 공유

### 코드 (`estimator.cpp:3734,3736`)
```cpp
Eigen::Vector3d un_acc_0 = latest_Q * (acc_0 - latest_Ba) - g;      // acc_0: processIMU가 갱신하는 멤버
Eigen::Vector3d un_gyr   = 0.5 * (gyr_0 + angularVelocity) - latest_Bg;
...
// 함수 끝에 acc_0/gyr_0 갱신 없음
```
`acc_0/gyr_0`는 `processIMU :273-274`(창 프리인테그레이션 경로)만 갱신한다. `predict()`는 자기
경로의 직전 샘플을 갖지 않고 창 경로의 값을 빌려 쓴다. 두 경로는 다른 박자로 돌아
중점 적분의 "이전 값"이 실제 직전 IMU가 아닐 수 있다.

### 3소스 대조
| 소스 | 이전 샘플 | 판정 |
|---|---|---|
| 우리 포크 | `acc_0/gyr_0`(창 멤버), 갱신 없음 | 공유 |
| upstream VINS-RGBD-FAST | **동일** | 물려받음 |
| VINS-Fusion `fastPredictIMU` | `latest_acc_0/latest_gyr_0` **예측 전용 멤버** — 재기저 시 `= acc_0` 복사, 매 스텝 끝 `= linear_acceleration` 갱신 | 분리 |

### 영향
- 한 스텝 dt(~2.6ms) 규모의 국소 오차. ①보다 훨씬 작음. 누적 없음.

## 5. 수정안 (VINS-Fusion 그대로 따름, 합쳐 10줄 안쪽)

| # | 변경 |
|---|---|
| ① | `updateLatestStates` 루프에서 `imu_buf.front()` → `tmp_imu_buf.front()` (2군데) |
| ② | 멤버 `latest_acc_0/latest_gyr_0` 추가. `updateLatestStates`에서 재기저 시 `= acc_0/gyr_0` 복사. `predict()`가 이를 쓰고 함수 끝에 `= linearAcceleration/angularVelocity` 갱신 |

**주의**: VINS 코어 예측 경로 수정이다. `imu_propagate` 토픽·`imu_pose`·HF TF·`base_vins`가
전부 이 값을 쓰므로 영향 범위는 "고주기 출력 전부"이고, 창 최적화 결과(저주기 odometry·
keyframe·map→body 저주기)는 **무영향**이다.

> ✅ **구현 확정(`15c75b8`)**: 위 표의 수식 그대로 적용하되, 단위검증을 위해 수학부를
> `utility/hf_predict.h`로 분리(`midpointStep` = VINS-Fusion 동일 수식 / `forEachSample` =
> "시각·값을 같은 큐 원소에서" 계약). 추가 안전장치: `latest_acc_0/gyr_0` 명시 0 초기화
> (latest_Bg NaN 사고 교훈)·`clearState` 리셋·첫 샘플 시딩. gtest 5건(결함① 회귀 포함).

## 6. 검증 계획 (수정 전 선행)

수정 전에 **영향을 먼저 잰다** — 팀 원칙(결론 제시 전 실측).

1. **재기저 점프 크기 측정**: bag 재생 중 `updateLatestStates` 직전/직후 `latest_P` 차(재기저 점프)를
   로깅. 현재(고정 acc/gyr) vs 수정(매 샘플) 두 빌드로 같은 bag → 점프 분포 비교.
   가감속 구간(출발·정지·회전)에서 차이가 나야 하고 정지 구간은 동일해야 한다.
   **⚠️꼬리 마스킹 필수**: 검증 bag으로 `~/ros2_ws/bag/edie_gate_verify_v16_play`를 쓸 경우,
   마지막 정지 구간(**~407s 이후**)은 동적 장애물 교란 + VINS 재부팅(시드 계승) 이벤트 구간이다
   (SW1-1866 수사 정본 bag). 재부팅 점프(수십 cm~1m급)가 재기저 점프 분포에 섞이면 측정이
   오염되므로 **재부팅 시각 마스킹 또는 407s 이후 제외**. bag은 캠페인 재생 정본 — **읽기 전용**.
2. **HF TF vs 저주기 TF 잔차**: 같은 스탬프의 `imu_propagate` pose와 최적화 pose 차의 RMS. 수정이
   맞다면 동적 구간 RMS가 줄어야 한다.
3. **회귀**: 기존 gtest 전부 통과 + `test_reboot_seed`(재기저 관련) 무변경 확인.
4. **도킹 관점 합격선**: 도킹 접근 속도(≤0.1 m/s) 구간에서 ①의 오차 기여가 재투영 게이트·앵커
   불확도(수 mm)보다 작으면 "수정 없이 진행 가능", 크면 수정 후 진행.

## 7. 결정 사항

- [x] 수정을 도킹 태스크 전에 완료 — 실측 선행(§6.5) 후 `15c75b8` (#SW1-1872)
- [ ] 수정을 upstream(jianhengLiu/VINS-RGBD-FAST)에 issue/PR로 보고할지 (미결 — 사용자 결정)

## 8. 이 분석에서 정정한 것

- 처음에 ②의 근거로 든 "주석 처리된 `// acc_0 = linear_acceleration;`"은 `predict()`가 아니라
  `predictMotion()`(회전 전용, `:3680-3722`) 안의 것이라 **②의 직접 증거가 아님**.
  ②의 직접 증거는 `predict()` 본문에 갱신이 없다는 사실 + VINS-Fusion의 별도 멤버 대조.

## 9. 교차검증 결과 (2026-08-19, 별도 세션 — 비판적 재검)

문서를 신뢰하지 않고 코드·git·bag을 직접 대조한 제3자 검증. **결함 ①·② 모두 실재 확정.**

### 9.1 확인된 것
- **①**: `estimator.cpp:3642-3645` 직접 읽기 — 루프에서 `tmp_imu_buf`만 pop, `imu_buf`는 불변
  → acc/gyr가 첫 샘플 고정인 채 시각만 전진. 문서 주장 그대로.
- **②**: `predict()` 본문(:3734-3735)이 `acc_0/gyr_0` 사용, 갱신 지점은 전 파일에서
  `processIMU:273-274`뿐, `latest_acc_0/latest_gyr_0` 멤버 **grep 0건**. 문서 주장 그대로.
- **upstream 계승**: `git blame -L 3642,3646` 결과(줄별 작성자):

  | 줄 | 내용 | 커밋 | 작성자 |
  |---|---|---|---|
  | 3642 | tmp 큐 복사 | `0e8856b0` 2022-05-24 | ChrisLiu \<liujianhengchris@qq.com\> (upstream) |
  | 3643 | for 루프 헤더 | `f49d7ca8` 2026-06-08 (ROS2 포트) | HeegonChae — 헤더 정리만, 값 인자 무접촉 |
  | **3644-3645** | **결함 본체(`imu_buf.front()` 고정)** | **`39dcb63d` "🦄 refactor:" 2022-06-18** | **Chris Liu \<943678231@qq.com\> (upstream 저자)** |

  결함 줄 자체는 2022년 upstream 커밋 이후 무변경 — **팀 유입 아님** 재확정.

### 9.2 서술 정밀도 보완 2건 (결함 부정 아님)
- **②의 "한 스텝 dt(~2.6ms) 규모"는 최선 케이스**: `acc_0`는 창 처리 경로가 마지막으로 소비한
  샘플이라 라이브 예측 시점 대비 **수십~수백 ms 낡은 값**일 수 있다. midpoint 절반 가중이라
  비누적·소규모 방향성은 유지되나 "1 스텝"은 보장이 아니다.
- **① 존재 하에선 재적분 루프 내 ②는 사실상 가려짐**(값이 아예 고정이므로). 수정·측정 모두
  ①이 지배적 — ①만 고친 빌드와 둘 다 고친 빌드를 따로 구분할 실익은 낮다.

### 9.3 검증 bag 적합성 실측 (`edie_gate_verify_v16_play`, 438.6s)
| §6 항목 | 실측 | 판정 |
|---|---|---|
| IMU 밀도 | **379.5Hz** (본문 382Hz 기재와 부합) | ✅ |
| 1. 가감속·회전 | \|dv/dt\|>0.5m/s² 이벤트 2,816샘플(휠 odom 100Hz), \|w\|>0.3rad/s 12.6% | ✅ |
| 정지 대조군 | \|v\|≤0.01m/s **67%** | ✅ |
| 4. 도킹급 저속 | 0.02~0.1m/s 구간 **≈76초** 존재 | ✅ |

→ 검증 계획 1·2·4 전부 이 bag으로 가능. 단 §6-1의 **꼬리 마스킹(407s 이후)** 준수.
실행 환경: 메인 ws 바이너리는 stale일 수 있으므로 자기 worktree 빌드 사용,
`ROS_DOMAIN_ID`는 77 회피(SW1-1866 데모 인프라 사용 이력).

### 9.4 bag 선정 — v14/v15/v16_play 3종 실측 비교 (휠 odom 100Hz 기준)
| bag | 길이 | 정지 | 저속 0.02~0.1m/s | 가감속(\|dv/dt\|>0.5) | 회전(\|w\|>0.3) | 오염 |
|---|---|---|---|---|---|---|
| **v15** | 322.1s | 59% | 31s | **3,339샘플 (최다)** | **21.3% (최다)** | **없음** (SW1-1866 3런 재부팅 0 실측) |
| v14 | 181.2s | 59% | 14s | 2,168 | 19.6% | 없음 (동일 근거) |
| v16_play | 438.6s | 67% | **76s (최다)** | 2,816 | 12.6% | **꼬리 407s~ 동적장애물+재부팅** |

**권고**: 결함 ①은 가감속·회전에서만 발현하므로,
- **주력 = v15**: 동적 이벤트 최다 + 오염 0 → 마스킹 없이 재기저 점프 A/B(검증 1·2)에 최적.
- **저속 합격선(검증 4) 보조 = v16_play**: 저속 76s 최다. 단 꼬리 마스킹 필수. (v15의 31초로도
  충분하면 v16_play 생략 가능.)
- **v14 = 스모크/회귀 재확인용**: 최단(181s)이라 빠른 반복에 유리, 단독 판정 근거로는 저속 14s가 얇음.

## 10. 수정 실측 결과 (08-19, v15 bag 3v3 — 빌드 base `adcb2b48` vs fix `5dd1ebf6`)

계측: `updateLatestStates`에 `[HF-REBASE-DIAG]` 로그(직전/직후 `latest_P` 차) — 수정 후에도
상존(도킹 HF TF 품질 감시용). 재기저 점프 = 최적화 보정 + 재적분 오차(결함①) + 이전샘플
오염(결함②)의 합이므로, A/B 차이가 결함 몫이다.

| 지표 (mm) | base 3런 | fix 3런 | 판정(사전 등록 합격선) |
|---|---|---|---|
| 동적 구간 중앙 | 34.21 / 34.56 / 34.98 | **1.81 / 1.82 / 1.83 (−95%)** | ✅ 3v3 완전 분리 |
| 정지 구간 중앙 | 2.40 / 2.42 / 2.44 | 0.58 / 0.58 / 0.62 | ⚠️등가성 밴드(±1.0) 하한 밖 — **방향이 개선**이라 실질 통과, 형식 이탈 기록 |
| 최대 점프 | 245.5~328.9 | 23.9~37.3 | 참고 (~1/10) |
| 재부팅 | 0/0/0 | 0/0/0 | ✅ |

- 정지 구간까지 줄어든 기전: 결함②는 정지 중에도 낡은 `acc_0`(창 경로 샘플)를 매 스텝 읽어
  노이즈성 불일치를 주입했음 — "①이 지배적, ②는 미미"라던 §4의 예상보다 ②의 몫이 컸다.
- **저주기 odometry 무영향 — 판정 근거 2겹**:
  - 구조(주 근거): 수정이 건드린 상태(`latest_P/Q/V`·`latest_acc_0/gyr_0`)의 소비처 전수 확인
    결과 HF 발행(`inputIMU :3574-3580`)·DIAG 로그·예측 경로 내부뿐 — 창 최적화
    (`processIMU`/`optimization`/초기화)로의 되먹임 **0**. 창은 `imu_buf` 원시 IMU와
    `Ps/Rs/Vs`만 쓰므로 코드 수준에서 절연. (`predictMotion`이 읽는 `latest_Bg`는 이번 수정 무접촉.)
  - 실측(확인): 경로장 26.34~26.40m·종점 xy·z범위 모두 base 3런 분산과 겹침, 방향성 이동 없음.
  - 한계 고지: 실측은 거친 지표(경로장·종점·z범위) 3v3 비교이며 통계적 등가성 검정 아님 —
    저주기 포함 최종 확증은 실기 주행 검증에서.
- verify-task 9/9, gtest 신규 5건(결함① 회귀 포함) 통과.
- 문서 §3의 "~2cm" 추정 대비 실제 동적 점프 중앙 3.5cm — 결함이 추정보다 컸다.
- 잔여: 도킹 접근속도(≤0.1 m/s) 구간 합격선(§6-4)은 도킹 태스크에서 앵커 불확도와 함께 최종 판정.
