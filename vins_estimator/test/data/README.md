# gtest 발췌 데이터

| 파일 | 쓰는 테스트 | 출처·추출 |
|---|---|---|
| `vpr4_anchor_to_capture_imu.csv` | `test_reboot_seed` `BridgeAlign.Vpr4FixtureReproducesLiftRotation` (SW1-1936) | 실기 bag `~/ros2_ws/bag/vpr4_0928_1744` 의 `/edie/sensor/offset_imu` 원시 자이로 + 휠 odom 이동 플래그, 앵커 pose(1790586259.342) → 시드 캡처(1790586280.870). `extract_vpr4_fixture.py <출력.csv>` 로 재생성(sqlite 읽기 전용, `source /opt/ros/humble/setup.bash` 필요). |

이 구간에서 로봇은 앵커를 잡은 뒤 들려서 약 −79° 돌고 바닥에 내려놓였다. 운동 구간만 적분하는 다리 누적기가
−79° 를 재현하고, 들리기 전 유휴 15.66 s 의 기여가 거의 0 인지를 고정한다. 근거 분석:
`~/ros2_ws/bag/analysis/reboot_seed_stale_anchor_20260929/CLAIMS.md`.
