// [SW1-1964] 영상 속도 제한기(frontend_rate_limiter.h) 회귀 테스트 — 재시작 뒤 영구 정지 재발 방지
#include <gtest/gtest.h>
#include <opencv2/core/persistence.hpp>

#include <string>

#include "../src/utility/frontend_rate_limiter.h"

namespace
{
// 배포 설정의 frontend_freq 를 그대로 쓴다(값이 바뀌어도 시험이 같은 판정을 본다).
int deployedFrontendFreq()
{
    const std::string path = std::string(VINS_SOURCE_ROOT) + "/config/elp_stereo_edie/vio_edie.yaml";
    cv::FileStorage   fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
        return -1;
    return static_cast<int>(fs["frontend_freq"]);
}

// 카메라 hz 로 n 장을 흘리고 받은 장수를 센다.
int feed(FrontendRateLimiter &lim, double t0, double hz, int n, int max_hz)
{
    int admitted = 0;
    for (int i = 0; i < n; ++i)
        admitted += lim.admit(t0 + i / hz, max_hz) ? 1 : 0;
    return admitted;
}
}  // namespace

class FrontendRateLimiterTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        max_hz_ = deployedFrontendFreq();
        ASSERT_GT(max_hz_, 0) << "vio_edie.yaml frontend_freq 를 못 읽음";
    }
    int max_hz_ = 0;
};

TEST_F(FrontendRateLimiterTest, SteadyCameraRateIsNeverBlocked)
{
    // .219 실측 카메라 약 9.58 Hz(freq 10 보다 느려 1% 맞춤 리셋이 안 걸림) — 오래 흘려도 막히지 않아야 한다.
    FrontendRateLimiter lim;
    lim.restart(100.0);
    const int n = 2200;  // 약 230 s
    EXPECT_EQ(feed(lim, 100.0 + 1 / 9.58, 9.58, n, max_hz_), n);
}

TEST_F(FrontendRateLimiterTest, OverRateBurstIsStillSkipped)
{
    // 제한기 본래 일: frontend_freq 보다 훨씬 빠른 입력은 일부를 건너뛴다.
    FrontendRateLimiter lim;
    lim.restart(100.0);
    const int n        = 300;
    const int admitted = feed(lim, 100.0 + 0.01, 100.0, n, max_hz_);
    EXPECT_LT(admitted, n);
    EXPECT_LE(lim.rate(100.0 + n / 100.0), max_hz_ + 1.0);
}

TEST_F(FrontendRateLimiterTest, RestartAfterLongRunAdmitsNextFrame)
{
    // 재생 B47 실측: 계단 순간 받은 수 963. 재시작 뒤 다음 프레임(0.1 s 뒤)은 받아야 한다.
    FrontendRateLimiter lim;
    lim.restart(100.0);
    feed(lim, 100.0 + 1 / 9.58, 9.58, 963, max_hz_);
    ASSERT_EQ(lim.inputCount(), 963);

    lim.restart(73.0);  // −27 s 계단 뒤 트래커 재시작(first_image_flag 블록)
    EXPECT_TRUE(lim.admit(73.1, max_hz_)) << "재시작 뒤 첫 프레임을 건너뛰면 1 s 뒤 또 재시작하는 고리";
    EXPECT_EQ(feed(lim, 73.2, 9.58, 50, max_hz_), 50);
}

TEST_F(FrontendRateLimiterTest, RestartClearsCountTogetherWithTimeBase)
{
    // 예전 결함 모양: 기준 시각만 옮기고 받은 수를 남기면 다음 프레임을 막는다 — restart 는 둘을 함께 바꿔야 한다.
    FrontendRateLimiter lim;
    lim.restart(100.0);
    feed(lim, 100.1, 9.58, 963, max_hz_);
    lim.restart(73.0);
    EXPECT_EQ(lim.inputCount(), 0);
    EXPECT_DOUBLE_EQ(lim.firstTime(), 73.0);

    // 대조: 받은 수 963 이 남은 채 기준만 0.1 s 전이면 속도 9630 장/s → 건너뜀(예전 결함 재현)
    const double stale_rate = 963.0 / 0.1;
    EXPECT_GT(stale_rate, max_hz_);
}
