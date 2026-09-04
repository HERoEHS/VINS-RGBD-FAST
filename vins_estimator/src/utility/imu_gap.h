#pragma once
// [SW1-1883 후속] IMU/휠 스탬프 간극 판정 — 순수 함수(단위 테스트 대상).
//
// 배경(09-04, 로봇 251): IMU 리더가 시계 점프를 따라가도록 고치거나, VINS를 켠 채 IMU 노드를 재시작하면
//   IMU 스탬프가 수십 초 점프한다. VINS에는 이미지 불연속 가드(1 s 초과 → 재시작)는 있으나 IMU 간극 가드가 없어
//   전방 점프면 getIMUInterval이 점프 구간을 단일 dt로 적분해 위치가 폭주하고, 후방 점프면 disorder 가드가
//   점프 구간 동안 IMU를 전량 폐기해 이미지 처리가 막힌다(critic-imu-clock 코드 확인).
// 판정: max_gap_s<=0 이면 간극 검사 끔(기존 동작 = 역행만 폐기). 첫 샘플(prev<=0)은 항상 통과.
namespace imu_gap
{
enum class Verdict
{
    kFirst,         // 첫 샘플(prev<=0: 기동 직후·이미지 불연속 리셋 직후) — 통과
    kOk,            // 정상 전진
    kDisorder,      // 소폭 역행/중복(|Δt| <= max) — 기존처럼 이 샘플만 폐기
    kForwardGap,    // Δt > max — 스탬프 전방 점프 → 재시작
    kBackwardJump,  // Δt < -max — 스탬프 후방 점프 → 재시작
};

inline Verdict classify(double prev_t, double now_t, double max_gap_s)
{
    if (prev_t <= 0.0)
        return Verdict::kFirst;
    if (now_t <= 0.0)
        return Verdict::kDisorder;  // stamp 0 글리치(드라이버 초기)는 폐기 — 후방 점프로 오판해 재시작하지 않는다
    const double dt = now_t - prev_t;
    if (max_gap_s > 0.0)
    {
        if (dt > max_gap_s)
            return Verdict::kForwardGap;
        if (dt < -max_gap_s)
            return Verdict::kBackwardJump;
    }
    if (dt <= 0.0)
        return Verdict::kDisorder;
    return Verdict::kOk;
}

inline bool needsRestart(Verdict v)
{
    return v == Verdict::kForwardGap || v == Verdict::kBackwardJump;
}
}  // namespace imu_gap
