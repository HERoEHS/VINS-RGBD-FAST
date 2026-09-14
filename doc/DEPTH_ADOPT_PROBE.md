# depth 채택 프로브 (VINS_DEPTH_ADOPT_LOG) — 2026-09-14, SW1-1889

## 왜
depth 백엔드 A/B(CPU SGBM vs Mali OpenCL BM, edie_vision)에서 BM은 복도 저텍스처 구간의 신선 유효 depth가 5~8 %뿐인데도
VINS 닫힌루프 오차는 SGBM과 구분되지 않았다(0.91 vs 0.89 %). 발행 depth는 EMA(α 0.4)가 새값이 없으면 옛값을 그대로
유지하므로, VINS가 **몇 개의 특징점에 depth를 고정 채택**했는지와 그 값이 **몇 프레임 묵었는지**를 알아야 "BM이라서
괜찮은지, 묵은 값 덕인지"를 판정할 수 있다. 기존 코드에는 이 로그가 없었다.

## 무엇
환경변수 `VINS_DEPTH_ADOPT_LOG=<파일경로>` 가 있으면 최적화(`Estimator::optimization`)마다 파일에 기록한다. 없으면 완전 no-op(기본).
로직 변경은 없고 기록만 한다. 구현: `vins_estimator/src/utility/depth_adopt_probe.h`(순수 함수 + `Logger`),
훅은 `estimator.cpp` 재투영 잔차 루프(`estimate_flag == 1 && FIX_DEPTH` 가 파라미터 블록을 상수로 고정하는 바로 그 조건).

| 행 | 형식 | 뜻 |
|---|---|---|
| `S` | `S <t> <fixed> <tri> <rough> <total>` | 최적화 1회 요약. 잔차에 참여한 특징 중 depth 고정(flag 1 && FIX_DEPTH) / 삼각측량(flag 2, 상한만) / 초기값·평균(그 외) 수 |
| `A` | `A <t> <id> <depth_m>` | 처음 고정 채택된 특징(창에서 사라질 때까지 flag가 유지되므로 한 번만) |
| `O` | `O <t> <id> <frame_stamp> <u> <v> <depth_m>` | 그 특징의 관측 중 `0 < depth ≤ depth_max_dist` 인 것. `frame_stamp`·`(u, v)`로 오프라인 age 맵을 조회한다 |

`t`는 창 최신 프레임 스탬프(`Headers[WINDOW_SIZE]`), `frame_stamp`는 관측 프레임 스탬프(`Headers[start_frame + k]`), `(u, v)`는
좌영상 화소(= depth 이미지 화소, `feature_manager.cpp` `depth_img.at(v, u)` 와 동일). 추정기 `clearState()`에서 중복 억제 집합을 비운다.

## 어떻게 쓰나
```
VINS_DEPTH_ADOPT_LOG=/tmp/run1.adopt ros2 launch vins_estimator edie_vslam.launch.py use_sim_time:=true
```
- 채택률 = Σfixed / Σtotal (S 행). 나이 = O 행의 `(frame_stamp, u, v)` 를 edie_vision `scripts/depth_ab` 계열이 만든 age 맵
  (프레임별 uint16 PNG, 값 = 마지막 신선 유효 이후 프레임 수)에서 조회.
- 주의: `fix_depth: 0` 이면 flag 1 이어도 고정하지 않으므로 `fixed` 는 0 이 된다(분류 규칙이 factor 분기와 같음).

## 검증
- gtest `test_depth_adopt_probe` 5건: 분류(flag×FIX_DEPTH), 관측 범위(0 제외·상한 포함), 행 형식, id당 1회·reset, 비활성 no-op.
- 로직 무변경: 환경변수 없는 기본 경로는 `enabled()` 검사 한 번 외에 추가 연산 없음.
