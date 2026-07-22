#pragma once
// [SW1-1837] Bg_z 잠금 — 발동 판정 로직 (헤더 단독, gtest 가능)
//
// [왜]
//   최적화기가 yaw 불일치(휠 타이밍·저품질 장면 등)를 gyro z-bias(Bg_z)로 도피시켜
//   참값(~1.5e-4 rad/s)의 15~40배로 과대추정하는 것이 yaw 드리프트의 단일 지배 원인
//   (인과 봉인 probe: 수렴 후 Bg_z만 고정 → v7 yaw −24°→−2.7°, xy 개선, 무비용).
//   참 gyro bias는 부팅 캘리브 + DLPF + Allan 실측상 작고 안정하므로, 수렴 후
//   상수로 고정하는 것이 물리적으로 정당하다.
//
// [안전 가드]
//   발동 시점의 |Bg_z|가 한계(max_radps)를 넘으면 잠그지 않는다 — 이미 부풀어버린
//   나쁜 값을 고정하는 사고 방지(6월 ZUPT 과제약 사고의 교훈: 맹목적 강제 금지).
#include <cmath>

namespace bgz_lock
{

// 잠금을 발동해도 되는가.
//   elapsed_sec: 첫 최적화 이후 경과 시간, bgz: 현재 Bg_z 추정치 [rad/s]
//   delay_sec: 수렴 대기 시간, max_radps: 발동 허용 |Bg_z| 상한 (<=0이면 항상 거부, 안전)
inline bool shouldLock(double elapsed_sec, double bgz, double delay_sec, double max_radps)
{
    if (max_radps <= 0.0)
        return false;
    if (elapsed_sec < delay_sec)
        return false;
    return std::fabs(bgz) < max_radps;
}

}  // namespace bgz_lock
