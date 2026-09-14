# depth 채택 프로브 (VINS_DEPTH_ADOPT_LOG) — 2026-09-14, SW1-1889

## 왜
depth 백엔드 A/B(CPU SGBM vs Mali OpenCL BM, edie_vision)에서 BM은 복도 저텍스처 구간의 신선 유효 depth가 5~8 %뿐인데도
VINS 닫힌루프 오차는 SGBM과 구분되지 않았다(0.91 vs 0.89 %, edie_vision 측 측정). 발행 depth는 EMA(α 0.4)가 새값이 없으면
옛값을 그대로 유지하므로, VINS가 **몇 개의 특징점에 depth를 고정 채택**했는지와 그 값이 **몇 프레임 묵었는지**를 알아야
"BM이라서 괜찮은지, 묵은 값 덕인지"를 판정할 수 있다. 기존 코드에는 이 로그가 없었다.

## 무엇
환경변수 `VINS_DEPTH_ADOPT_LOG=<파일경로>` 가 있으면 최적화(`Estimator::optimization`)마다 파일에 기록한다. 없으면 완전 no-op(기본).
로직 변경은 없고 기록만 한다. 구현: `vins_estimator/src/utility/depth_adopt_probe.h`(순수 함수 + `Logger`),
훅은 `estimator.cpp` 재투영 잔차 루프(`estimate_flag == 1 && FIX_DEPTH` 가 파라미터 블록을 상수로 고정하는 바로 그 조건),
`FeaturePerFrame::depth_verified`(triangulateWithDepth 가 교차검증 통과 관측에 표시, 프로브만 읽음).

| 행 | 형식 | 뜻 |
|---|---|---|
| `S` | `S <t> <fixed> <tri> <rough> <total> <flag1>` | 최적화 1회 요약. 잔차에 참여한 특징 중 depth 고정(flag 1 && FIX_DEPTH) / 삼각측량(flag 2, 상한만) / 그 외 수. `flag1` 은 `fix_depth` 설정과 무관한 flag 1 수 |
| `A` | `A <t> <id> <depth_m>` | 처음 고정 채택된 특징(창에서 사라질 때까지 flag가 유지되므로 한 번만). `depth_m` 은 원시 화소 depth 가 아니라 **앵커 프레임으로 옮긴 verified 관측의 평균**(`feature_manager.cpp` triangulateWithDepth) |
| `O` | `O <t> <id> <frame_stamp> <u> <v> <depth_m> <verified>` | 그 특징의 관측 중 `0 < depth ≤ depth_max_dist` 인 것. `verified=1` 이면 다른 프레임과의 재투영 교차검증(residual < 10/460)을 통과해 채택값에 실제로 기여한 관측. **나이는 verified=1 만으로 계산**한다(0 은 후보였을 뿐) |

- `t` = 창 최신 프레임 스탬프(`Headers[WINDOW_SIZE]`), `frame_stamp` = 관측 프레임 스탬프(`Headers[start_frame + k]`).
- `(u, v)` = 왜곡 보정 전 좌영상 화소(= depth 이미지 화소). VINS 는 `(int)` **절사**로 depth 를 샘플링하므로(`feature_manager.cpp` `depth_img.at(v, u)`)
  오프라인 age 맵 조회도 절사를 쓴다(반올림 아님).
- 추정기 `clearState()` 에서 중복 억제 집합을 비운다. 특징 id 는 프로세스 안에서 재사용되지 않지만(`n_id` 는 tracker 생성자에서만 0),
  재시작 후에도 추적이 이어진 특징의 재채택을 새 이벤트로 남기기 위해서다.

## 어떻게 쓰나
```
VINS_DEPTH_ADOPT_LOG=/dev/shm/run1.adopt ros2 launch vins_estimator edie_vslam.launch.py use_sim_time:=true
```
- 로봇에서는 `/dev/shm` 아래를 쓴다(디스크 기록이 제어 루프를 0.8~1.5 s 멈춘 사례가 있음). 로컬 재생은 아무 경로나 된다.
- 채택률 = Σfixed / Σtotal (S 행). 나이 = O 행(verified=1)의 `(frame_stamp, u, v)` 를 edie_vision `scripts/depth_ab` 계열이 만든 age 맵
  (프레임별 uint16 PNG, 값 = 마지막 신선 유효 이후 프레임 수)에서 조회.
- 경로를 열 수 없으면 stderr 에 `[depth_adopt_probe] cannot open …` 한 줄을 내고 비활성으로 떨어진다.

## 한계·주의
- `fix_depth: 0` 이면 flag 1 이어도 고정하지 않으므로 `fixed` 는 0, 대신 `flag1` 로 센다(edie 설정은 1).
- `use_yaw_gating: 1` 이면 모든 관측이 게이트된 특징은 problem 에 들어가지 않는데도 `fixed` 로 세어 과대 계수될 수 있다(edie 설정은 0).
- 중복 억제 집합(`std::set<int>`)은 채택 id 수만큼 프로세스 수명 동안 단조 증가한다. 프로브 전용·기본 off 라 허용하되 장시간 운용 로그 용도로는 쓰지 않는다.
- S 행마다 flush 한 번(최적화당 1회). 관측 행은 특징당 한 번만 나오므로 정상 상태에서 부하는 작다.

## 검증
- gtest `test_depth_adopt_probe` 6건: 분류(flag×FIX_DEPTH, flag1), 관측 범위(0 제외·상한 포함), 행 형식(verified 0/1), id당 1회·reset, 비활성 no-op, 열기 실패 무크래시.
- 로직 무변경: 환경변수 없는 기본 경로는 `enabled()` 검사와 `depth_verified` bool 쓰기 외 추가 연산 없음. 추정 결과에 쓰이는 값은 읽기만 한다.
