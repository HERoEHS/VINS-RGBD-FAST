#pragma once
// [SW1-1883 후속] 사전적분 슬롯 상한 강제 키프레임 — 순수 판정 함수(단위 테스트 대상).
//
// 배경: 정지 중 SECOND_NEW가 반복되면 버려지는 프레임의 IMU·휠 적분이 슬롯 WINDOW_SIZE-1에 계속
//   이어 붙어(slideWindow) 무한히 길어지고, 길이가 preint_max_dt_s(게이트)를 넘는 순간 그 구간의
//   IMU·휠 factor가 최적화·marg에서 빠져 창이 앞/뒤로 끊긴다(분단). 09-04 분단 실험: 분단만 없애면
//   v16 bag의 가드 발동·재부팅 3/3→0/3, 정지 중 Bg 요동 10~30배 감소, 궤적 무손실.
// 처방: 슬롯 WINDOW_SIZE-1 누적이 상한(cap)을 넘은 것이 확인되는 즉시(=그 다음 프레임) 키프레임(MARGIN_OLD)으로
//   강제해 누적 구간이 게이트에 닿을 일이 없게 한다(사슬 상시 유지 + factor 길이 유계 ≈ cap + 1프레임). cap<=0 이면 끔.
//   ※ 판정은 processImage에서 이번 프레임의 IMU·휠 적분 '전'에 이뤄지므로 '이번 구간'은 더하지 않는다(critic 지적 09-04).
//   비용 후보(critic): 정지 중 창이 cap마다 한 칸 회전 → 가드류의 '정지 창 동결' 전제 흔들림 → v16 A/B로 판정.
namespace preint_gate
{
// slot_prev_dt: 슬롯 WINDOW_SIZE-1 에 이미 누적된 적분 길이[s]
// cap_s       : 상한[s], <=0 = 비활성
inline bool shouldForceKeyframe(double slot_prev_dt, double cap_s)
{
    if (cap_s <= 0.0)
        return false;
    return slot_prev_dt > cap_s;
}
}  // namespace preint_gate
