#pragma once
// [SW1-1959 도킹 위치 나이] 카메라가 끊겼을 때 HF(고주기) 위치 발행을 멈출지 판정 — 순수 함수부.
//
// 배경: 카메라가 끊겨도 VINS 는 IMU 만으로 계산한 위치를 "새 시각"을 찍어 계속 내보냈다. 도킹 컨트롤러는
//   시각만 보고 위치를 믿으므로, 눈 감은 위치로 달릴 수 있었다.
// 왜 수신 시각(steady_clock)인가: 영상 스탬프로 재면 시계 점프·재스탬프 때 영구 차단이 날 수 있다.
//   "지금 프레임이 실제로 도착했는가" 만 보면 프레임이 흐르는 한 절대 차단되지 않는다.
//   대가로 bag 재생은 1배속이어야 한다(빠르거나 느리면 공백 길이가 달라진다).

#include <chrono>

namespace hf_image_gate
{

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

// "영상을 아직 한 번도 못 받음" 표시. steady_clock 은 부팅 뒤 시간이라 실제 now 가 이 값일 일은 없다.
constexpr TimePoint kNoImageYet{};
// 차단이 이어지는 동안 공백 경고를 다시 찍는 주기 [s]
constexpr double kCutWarnPeriodS = 10.0;

// 발행해도 되면 true. now − last_arrival > timeout_s 일 때만 false.
//   timeout_s ≤ 0 = 끔, 영상을 아직 못 받았으면 막지 않는다(부팅 중엔 INITIAL 이 이미 막는다).
//   IMU 스레드의 now 가 트래커 스레드의 갱신보다 먼저 잡혀 나이가 음수여도 신선으로 본다.
inline bool hfImageFresh(TimePoint now, TimePoint last_arrival, double timeout_s)
{
    if (timeout_s <= 0.0 || last_arrival == kNoImageYet)
        return true;
    return !(std::chrono::duration<double>(now - last_arrival).count() > timeout_s);
}

enum class CutEvent
{
    kNone,       // 할 말 없음
    kCut,        // 막 끊김 — 1회 경고
    kStillCut,   // 끊긴 채 경고 주기가 지남
    kRecovered,  // 막 복귀 — 1회 경고
};

// 끊김·복귀 전이와 주기 경고 시점만 판정한다(로그 출력·발행은 호출자 몫).
class CutTracker
{
  public:
    CutEvent update(bool fresh, TimePoint now)
    {
        if (!fresh)
            ++blocked_samples_;
        if (!cut_)
        {
            if (fresh)
                return CutEvent::kNone;
            cut_       = true;
            cut_since_ = now;
            last_warn_ = now;
            ++cut_count_;
            return CutEvent::kCut;
        }
        if (fresh)
        {
            cut_ = false;
            return CutEvent::kRecovered;
        }
        if (std::chrono::duration<double>(now - last_warn_).count() >= kCutWarnPeriodS)
        {
            last_warn_ = now;
            return CutEvent::kStillCut;
        }
        return CutEvent::kNone;
    }

    bool isCut() const { return cut_; }
    long cutCount() const { return cut_count_; }              // 프로세스 시작 뒤 끊김 횟수
    long blockedSamples() const { return blocked_samples_; }  // 프로세스 시작 뒤 건너뛴 발행 수
    // 이번 끊김이 시작된 뒤 지난 시간 [s] — kRecovered 때 중단 길이를 찍는 용도
    double cutDurationS(TimePoint now) const
    {
        return std::chrono::duration<double>(now - cut_since_).count();
    }

  private:
    bool      cut_             = false;
    TimePoint cut_since_       = kNoImageYet;
    TimePoint last_warn_       = kNoImageYet;
    long      cut_count_       = 0;
    long      blocked_samples_ = 0;
};

}  // namespace hf_image_gate
