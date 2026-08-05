# AprilTag GT 도구 — 평가 세션 표준 실행 규약 (SW1-1866)

## 왜 표준화하나

`gt_apriltag_4board.py`의 옵션이 세션마다 다르면 **평가 좌표계가 세션마다 달라진다**.
특히 `--pin-odom-frame odom`이 빠지면 `map→odom` 1회 핀이 발행되지 않아:

- 휠 `base_footprint`가 map에 앵커되지 않고,
- VINS 출력 핀(`use_output_map_anchor: 1`)도 받을 핀이 없어 세션 프레임 발행으로
  퇴화한다(우아한 퇴화 — 죽지는 않지만 KPI 측정 좌표계가 달라짐).

**GT 평가 세션 = 항상 아래 표준 명령**으로 실행한다.

## 표준 명령 (평가 세션)

```bash
python3 "$HOME/ros2_ws/src/edie9/edie_localization/VINS-RGBD-FAST/scripts/april_gt/gt_apriltag_4board.py" \
   --K 397.64,397.64,339.36,270.85 \
   --tag-size 0.024 \
   --allowed-boards AR1,AR2,AR3 \
   --frame-id map \
   --child-frame-id apriltag_gt_camera \
   --gt-odom-topic /apriltag_gt/odom \
   --gt-path-topic /apriltag_gt/path \
   --publish-tf \
   --publish-board-tf \
   --pin-odom-frame odom \
   --output "$HOME/ros2_ws/bag/live_apriltag_gt.tum"
```

핵심 = `--pin-odom-frame odom` **누락 금지**. 나머지 옵션은 캘리브 값이라 고정.

## 핀의 의미와 알려진 한계 (08-04 실증)

- 핀 = **첫 유효 GT 관측에서 map→odom을 1회 계산해 static 발행 후 고정**
  (`try_pin_odom`). 연속 보정이 아니다 — 평가에서 드리프트가 보여야 하므로 의도된
  설계.
- 실측 핀 잔차(v13): xy 37mm / yaw −0.95° (yaw는 3m 거리서 ~5cm로 번짐 — 모든
  평가 수치의 계통 성분). 원인 분해(핀 계산 지연 중 이동 vs GT 단일 프레임 노이즈)
  는 미완 — 개선(정지 중 N장 평균 등)은 분해 측정 후 별도 태스크.
- v13 실측: 핀 발행 시각 = 첫 GT 관측 +0.26s.

## 소비 관계

| 소비자 | 사용 방식 |
|---|---|
| rviz 휠 `base_footprint` | `map→odom`(핀) ∘ `odom→base_footprint`(휠 odom) |
| VINS 출력 핀 (`use_output_map_anchor: 1`) | `map→odom` static 구독 ∘ init 순간 휠 pose 스냅샷 — 상세는 vio_edie.yaml 주석 |
| 오프라인 평가 | 핀 규약(pin convention) — 전역 최적 맞춤(evo -a)과 규약이 다름에 주의 |
