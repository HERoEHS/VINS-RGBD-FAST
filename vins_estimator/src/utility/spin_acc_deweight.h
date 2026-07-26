#pragma once
// [SW1-1837] 스핀 중 accel 신뢰 강등 — 판정 로직 (헤더 단독, gtest 가능)
//
// [왜]
//   v10 포렌식(07-26) 확정: 제자리 스핀 중 IMU는 레버암(축중점에서 0.1056m) 탓에
//   원심가속 ω²r을 전방으로 받는데, 스핀 중엔 수직 참조가 결핍돼 옵티마이저가 이를
//   중력 기울기 δθ≈ω²r/g(95°/s에서 ~1.9°)로 오해한다 → 실측 자세 오차 1.1~1.8°
//   (풀 쿼터니언 gyro 적분 대비) → 중력 누설이 회전과 함께 돌며 적분돼
//   유령 레버 f≈r의 (I−R)·f 전방 활주(run당 0.1~0.26m, 닫힌루프 잔여 오차의
//   단독 지배 원인). 비전 게이팅·plane off·marg 토글 개입 3연속 무반응으로
//   "모호성은 IMU 관측 내재"가 확정됨.
//
// [처방]
//   고속 회전 샘플의 acc 노이즈만 키워(가중 강등) 그 구간의 자세는 gyro(스핀 중
//   참값 실측 확인, 3s 적분 오차 <0.05°), 위치는 휠 factor에 맡긴다.
//   gyro 노이즈는 건드리지 않는다(회전 정보는 온전히 유지).
//
// [가중치 근거]
//   ω=1.66 rad/s(95°/s)에서 교란 ω²r = 0.29 m/s² ≈ 3.6×ACC_N(0.08).
//   factor 10이면 유효 σ가 0.8 m/s²로 커져 교란이 0.36σ 수준 → 자세를 끌 힘 상실.
namespace spin_acc_deweight
{

// 이 샘플의 acc 노이즈 표준편차 배율.
//   gyr_norm_biascorr: bias 보정된 각속도 크기 |ω − Bg| [rad/s]
//   thresh_radps: 발동 임계 (<=0 이면 항상 1.0 = 비활성, 안전)
//   factor: 초과 시 배율 (<=1 이면 항상 1.0 = 비활성, 안전)
inline double accNoiseScale(double gyr_norm_biascorr, double thresh_radps, double factor)
{
    if (thresh_radps <= 0.0 || factor <= 1.0)
        return 1.0;
    return (gyr_norm_biascorr > thresh_radps) ? factor : 1.0;
}

}  // namespace spin_acc_deweight
