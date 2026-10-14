#pragma once
#include <cmath>

// [SW1-1964] 영상 속도 제한기 — 시간 기준(first_image_time)과 받은 사진 수(input_count)를 한 몸으로 묶는다.
//
// 배경: 제한기는 "받은 사진 수 ÷ 기준 시각부터 지난 시간"이 frontend_freq 를 넘으면 프레임을 건너뛴다.
//   예전 코드는 두 값을 따로 들고 있어, 트래커 재시작(영상 불연속·IMU/휠 간극) 때 기준 시각만 새로 잡고
//   받은 사진 수는 남겼다. 그러면 재시작 직후 "963 장 ÷ 0.1 s" 처럼 계산돼 모든 프레임을 건너뛰고,
//   1 s 뒤 image discontinue 로 또 재시작하는 고리에 빠졌다(재생 109 회). upstream 0e8856b(제한기 도입)부터의 누락.
//   기준 시각을 바꾸는 길을 restart() 하나로 모아 두 값이 따로 움직일 수 없게 한다.
//
// 동작은 예전 식과 같다: round(input_count / (t − first_time)) > max_hz 면 건너뜀, 아니면 받은 수 +1.
class FrontendRateLimiter
{
  public:
    // 기준 시각을 t 로 새로 잡고 받은 사진 수를 0 으로 (둘은 항상 함께 바뀐다).
    void restart(double t)
    {
        first_time_  = t;
        input_count_ = 0;
    }

    // 이 프레임을 받을지. false = 건너뜀(받은 수 그대로), true = 받음(받은 수 +1).
    bool admit(double t, int max_hz)
    {
        if (std::round(rate(t)) > max_hz)
            return false;
        ++input_count_;
        return true;
    }

    double firstTime() const { return first_time_; }
    int    inputCount() const { return input_count_; }
    // 기준 시각 이후 받은 사진의 평균 속도 [장/s] — 로그·판정용.
    double rate(double t) const { return 1.0 * input_count_ / (t - first_time_); }

  private:
    double first_time_  = 0.0;
    int    input_count_ = 0;
};
