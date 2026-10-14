#pragma once
#include <atomic>
#include <cmath>

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

// [SW1-1964 V1] 휠 기준 다시 잡기 판정 — 순수 함수(단위 테스트 대상).
//
// 배경: 벽시계가 N s 뒤로 뛰면 IMU 가드·이미지 불연속이 추정기를 재시작하지만, 휠 콜백의 last_wheel_t 는
//   옛 시각에 남는다. wheel_gap_max_s 가 꺼져 있으면(기본 0) 그 뒤 오는 과거 스탬프 휠 표본이 전부 kDisorder 로
//   버려져, 재시작한 추정기가 휠을 기다리며 약 N s 멈춘다(바퀴 컨트롤러가 계단 직후 바로 발행할 때, SW1-1964 A).
// 판정: 재시작 요청이 있었고(rebase_requested) 표본이 기준보다 back_jump_s 넘게 과거일 때만 참.
//   - 앞쪽 표본(계단 직전 늦게 온 옛 odom)이나 소폭 역행(중복·지터)은 깃발을 먹지 않는다.
//   - 첫 표본(prev<=0)·stamp 0 글리치(now<=0)는 대상이 아니다(classify 가 처리).
inline bool shouldRebaseWheel(bool rebase_requested, double prev_t, double now_t, double back_jump_s)
{
    if (!rebase_requested || prev_t <= 0.0 || now_t <= 0.0)
        return false;
    return now_t < prev_t - back_jump_s;
}
// [SW1-1964 V1] 휠 스탬프 문 — 휠 콜백이 실제로 쓰는 판정(기준 시각·다시 잡기 깃발)을 한곳에 모은다.
//   단위 테스트가 노드와 같은 코드를 지나가게 하려는 것(critic V1 R3).
//   스레드: requestRebase() 는 어느 스레드에서 불러도 된다(원자). check()/accept() 는 휠 콜백 스레드 전용.
class WheelStampGate
{
  public:
    void   requestRebase() { rebase_request_.store(true); }
    bool   rebaseRequested() const { return rebase_request_.load(); }
    double last() const { return last_t_; }

    // 표본 하나를 판정한다. *rebased=true 면 기준을 지우고 첫 표본(kFirst)으로 받았다는 뜻.
    //   imu_t: 같은 콜백 스레드가 마지막으로 받은 IMU 시각(0 이하면 모름).
    //   IMU 도 같은 계단을 탔을 때만(|now−imu| < 문턱) 다시 잡는다 — 휠만 따로 뒤로 간 경우(재시작 없음)에
    //   묵은 깃발을 잘못 먹고 추정기 휠 줄에 순서가 뒤집힌 표본을 넣는 것을 막는다(critic V1 R1).
    Verdict check(double now_t, double imu_t, double max_gap_s, double back_jump_s, bool *rebased)
    {
        *rebased              = false;
        const bool imu_agrees = imu_t <= 0.0 || std::abs(now_t - imu_t) < back_jump_s;
        if (imu_agrees && shouldRebaseWheel(rebase_request_.load(), last_t_, now_t, back_jump_s))
        {
            // 깃발을 내리는 곳은 이 스레드뿐이라 비교-교환은 store(false) 와 같다. 소비 직전 µs 안에 온
            //   두 번째 요청은 이번 소비에 합쳐진다(실해 사실상 0) — 받아들임.
            bool expected = true;
            if (rebase_request_.compare_exchange_strong(expected, false))
            {
                last_t_  = 0.0;
                *rebased = true;
            }
        }
        return classify(last_t_, now_t, max_gap_s);
    }
    void accept(double now_t) { last_t_ = now_t; }

  private:
    std::atomic<bool> rebase_request_{false};
    double            last_t_{0.0};
};
}  // namespace imu_gap
