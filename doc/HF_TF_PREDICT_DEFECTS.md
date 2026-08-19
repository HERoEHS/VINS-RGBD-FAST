# 분석: 고주기(HF) body TF의 IMU 예측 경로 결함 2건 (upstream 계승, 2026-08-19)

> 상태: **발견·근거 확정 + 별도 세션 교차검증 완료(08-19, §9) — 수정 미착수(사용자 결정 대기)**
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

| 단계 | 위치 | 하는 일 |
|---|---|---|
| 예측 | `inputIMU :3554-3578` (predict 호출 `:3567`) → `predict :3723-3741` | IMU마다 중점 적분으로 `latest_P/Q/V` 전진. 바이어스·중력 제거 |
| 발행 | `pubLatestOdometry`(visualization.cpp) | `latest_P/Q` → `map→body`(+`odom→base_vins`) HF TF, 스로틀·LPF(`hf_body_tf_tau`) |
| 보정 | `updateLatestStates :3629-3649` (호출 `:656` 정상 루프, `:556/:575` init) | `latest_* = Ps/Rs/Vs/Bas/Bgs[frame_count]`로 **최적 해 재기저** 후, `imu_buf`에 남은(창 이후 도착한) IMU를 재적분 |
| IMU 소비 | `getIMUInterval :3835-3840` | 창에 쓴 IMU를 `imu_buf`에서 pop → 남는 건 "마지막 프레임 이후 ~ 지금" |

결론: **"IMU 적분 예측 → 매 최적화마다 최적 해로 재기저 → 잔여 IMU 재적분 → 계속 예측"**
구조는 맞다. HF 블록 주석(visualization.cpp)의 "매 최적화 완료 시 예측이 보정값으로 재기저되며
mm·0.0x° 미세 점프"가 이 재기저 순간이다.

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

## 7. 결정 사항 (미결)

- [ ] 수정을 도킹 태스크 전에 할지 / 실측 후 판단할지
- [ ] 수정 시 upstream(jianhengLiu/VINS-RGBD-FAST)에 issue/PR로 보고할지

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
