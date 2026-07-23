#pragma once
// [SW1-1837] Bg_z 잠금 — 상태 기반 발동 + 정지 실측 기준 가드 + 재잠금 (헤더 단독, gtest 가능)
//
// [왜]
//   최적화기가 yaw 불일치(휠 타이밍·저품질 장면 등)를 gyro z-bias(Bg_z)로 도피시켜
//   참값의 15~40배로 과대추정하는 것이 yaw 드리프트의 단일 지배 원인
//   (인과 봉인 probe: 수렴 후 Bg_z만 고정 → v7 yaw −24°→−2.7°, xy 개선, 무비용).
//
// [왜 상태 기반인가 — 시간 기반(고정 delay)의 한계]
//   "init 후 10초"는 검증 bag 안무 의존 상수로 B2C(시동 직후 조작)에 부적합.
//   실측: init 직후 Bg_z≈0(미수렴) → 1s 과도 스파이크 → 3~5s 참값 수렴.
//
// [왜 '정지 실측 기준' 가드인가 — 절대 크기 가드의 한계 (07-23 실기 확정)]
//   gyro bias는 워밍업으로 세션 중 표류한다(실측: 부팅 직후 −0.13e-3 → 56분 후
//   −1.9e-3). warm 상태의 '진짜' bias(1.9e-3)는 인플레(2.4e-3+)와 크기만으로 구분
//   불가 → 절대 상한(1e-3) 가드는 warm 재시작 세션서 잠금을 영구 보류시킴.
//   해법: 정지 중엔 원시 gyro 출력이 곧 bias(직접 물리 관측)이므로, 그 중앙값을
//   기준으로 "최적화기 추정이 물리 실측과 가까운가"로 가드한다 — 인플레는 물리
//   실측과 괴리되므로 여전히 차단되고, warm 참값은 실측과 일치하므로 통과한다.
//
// [재잠금 — 온도 표류 추종]
//   잠금 후에도 정지 창마다 실측 중앙값을 갱신하고, 잠긴 값과 relock_delta 이상
//   벌어지면 잠금값을 실측값으로 교체한다("부팅 1회 고정" → "정지마다 온도 추종").
//   교체는 실측값 주입(Bg_z 한 축, 1차 bias 보정 범위)이라 marg 선형화와 정합.
//
// [조건이 각각 막는 실패]
//   정지 지속   : 움직임 중 발동 — 인플레 진행 중인 값을 잠그는 사고
//   추정 안정   : 1s 과도 스파이크·미수렴 값 잠금 (빠른 경로)
//   실측 거리 가드: 인플레 값 잠금 — 정지 실측과 괴리된 추정 차단 (6월 ZUPT 교훈).
//                 spin-first 세션은 정지 창 자체가 없어 미발동 = 기본 동작(우아한 실패).
#include <algorithm>
#include <cmath>
#include <deque>
#include <utility>
#include <vector>

namespace bgz_lock
{

struct Params
{
    double min_wait_sec;    // 첫 최적화 후 최소 대기 (1s 과도 스파이크 회피 벨트)
    double still_sec;       // 정지 지속 요구 시간 = 안정성·실측 판정 창 길이
    double stab_max_radps;  // 창 내 Bg_z 변동폭(max-min) 허용 상한 (빠른 경로)
    double fallback_sec;    // 정지 누적이 이 시간을 넘으면 중앙값 폴백 발동 허용
    double max_radps;       // |추정 − 정지 실측| 허용 상한 (<=0이면 항상 거부 = 비활성)
    double relock_delta;    // 재잠금 발동 문턱: |실측 − 잠금값| 초과 시 갱신 (<=0=재잠금 off)
};

// 폴백에서 "중앙값 근방 프레임" 판정 허용 오차 (bgz_lock 설계 문서 참조)
constexpr double kSnapTolRadps = 1e-4;
// 정지 판정 문턱 — 정지 시 gyro 노이즈·Vs 오차 ≪ 0.05인 반면 주행/회전은 ≥0.1,
// 두 분포가 겹치지 않는 보수적 경계값 (processIMU·optimization 공용)
constexpr double kStillGyrRadps = 0.05;  // ≈ 2.9 °/s
constexpr double kStillVelMps   = 0.05;
// 휠 정지 문턱 — gyro 문턱(0.05)을 통과하는 '준정지'(스핀 사이 느린 잔여 회전)가
// 실측을 오염시켜 재잠금을 오발동시킨 회귀 실측(v7 6회/run)의 처방. 엔코더가
// 돌고 있으면 정지가 아니라는 직접 판별자.
//   값 근거: 준정지 주행의 휠 twist ≥0.088(차단 대상) vs 정지 중 twist 노이즈
//   스파이크 ≤0.015(허용해야 함 — 0.01로 잡았더니 정지 리셋 반복으로 v5 잠금
//   미발동 회귀 실측). 0.05 = 노이즈의 3배 여유 + 준정지와 명확 분리.
constexpr double kStillWheelMax = 0.05;  // [m/s | rad/s]
// 재잠금 관찰 창 — 온도 표류는 분 단위 현상이라 "연속 정지 10s + 10s 중앙값"으로
// 한정(초기 잠금 창 2s와 별도). 주행 중 준정지(1~2s)를 구조적으로 배제하고
// 저주파 요동(v5 ±0.35e-3 실측)도 평균화.
constexpr double kRelockWinSec = 10.0;
// 실측 중앙값의 물리 상한 — 이 이상이면 센서 이상/미정지 의심, 발동·재잠금 모두 보류
constexpr double kRestPhysMaxRadps = 0.02;
// 재잠금 최소 간격 [s] — 표류는 분 단위 현상, 잦은 갱신 방지
constexpr double kRelockCooldownSec = 10.0;

// 정지 중 원시 gyro z 중앙값 추적 — bias의 직접 물리 관측(최적화기·marg 무관).
// 현재 정지 구간 한정(움직이면 리셋), 창 길이 win_sec 유지.
class RestBias
{
public:
    // keep_sec: 이력 보관 창(재잠금 창 이상으로) — 판정 창은 ready/median의 win_sec
    void update(double t, double gyr_z, bool still_now, double keep_sec)
    {
        if (!still_now)
        {
            hist_.clear();
            return;
        }
        hist_.emplace_back(t, gyr_z);
        while (!hist_.empty() && t - hist_.front().first > keep_sec)
            hist_.pop_front();
    }

    // win_sec 길이의 연속 정지 관찰이 쌓였는가(80% 이상 스팬 + 최소 표본).
    // 움직이면 이력이 리셋되므로 스팬 = 현재 정지 구간의 연속 지속 시간.
    bool ready(double win_sec) const
    {
        return hist_.size() >= 10 &&
               hist_.back().first - hist_.front().first >= 0.8 * win_sec;
    }

    // 최근 win_sec 구간의 중앙값
    double median(double win_sec) const
    {
        std::vector<double> v;
        v.reserve(hist_.size());
        const double t_end = hist_.back().first;
        for (auto it = hist_.rbegin(); it != hist_.rend(); ++it)
        {
            if (t_end - it->first > win_sec)
                break;
            v.push_back(it->second);
        }
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    }

private:
    std::deque<std::pair<double, double>> hist_;
};

// 프레임마다 update()를 호출해 상태를 누적하고, 발동 조건(빠른 경로 또는 폴백)이
// 성립하는 첫 순간에 true를 반환한다. 발동 이후의 래치는 호출측(estimator) 책임.
//   rest_ready/rest_median: 정지 실측(RestBias) 상태 — 가드의 기준값.
class Tracker
{
public:
    bool update(double t, double bgz, bool still_now, bool rest_ready, double rest_median,
                const Params &p)
    {
        if (p.max_radps <= 0.0)
            return false;

        if (first_t_ < 0.0)
            first_t_ = t;

        if (still_now)
        {
            if (still_since_ < 0.0)
                still_since_ = t;
        }
        else
        {
            still_since_ = -1.0;
            hist_.clear();
            return false;
        }

        hist_.emplace_back(t, bgz);
        while (hist_.size() > kMaxHist)
            hist_.pop_front();

        if (t - first_t_ < p.min_wait_sec)
            return false;
        if (t - still_since_ < p.still_sec)
            return false;
        // 가드 기준인 실측이 준비돼야 발동 — 정지 still_sec 지속이면 통상 함께 준비됨
        if (!rest_ready || std::fabs(rest_median) > kRestPhysMaxRadps)
            return false;

        // ── 빠른 경로: 최근 still_sec 창이 충분히 찼고 변동폭이 문턱 이하 ──
        double lo = bgz, hi = bgz, span_start = t;
        for (auto it = hist_.rbegin(); it != hist_.rend(); ++it)
        {
            if (t - it->first > p.still_sec)
                break;
            lo         = std::min(lo, it->second);
            hi         = std::max(hi, it->second);
            span_start = it->first;
        }
        if (t - span_start >= 0.8 * p.still_sec && hi - lo <= p.stab_max_radps)
            return std::fabs(bgz - rest_median) < p.max_radps;

        // ── 폴백: 정지 관찰 누적 + 추정 요동의 중앙값 근방 프레임 선택 ──
        if (p.fallback_sec > 0.0 && t - still_since_ >= p.fallback_sec)
        {
            std::vector<double> v;
            v.reserve(hist_.size());
            for (const auto &s : hist_)
                v.push_back(s.second);
            std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
            const double med = v[v.size() / 2];
            if (std::fabs(med - rest_median) < p.max_radps &&
                std::fabs(bgz - med) < kSnapTolRadps)
                return std::fabs(bgz - rest_median) < p.max_radps;
        }
        return false;
    }

private:
    static constexpr size_t kMaxHist = 4096;
    double first_t_{-1.0};
    double still_since_{-1.0};
    std::deque<std::pair<double, double>> hist_;
};

// 재잠금 판정 — 잠금 후 정지 실측이 잠긴 값에서 relock_delta 이상 벌어졌는가.
inline bool shouldRelock(double rest_median, double locked_val, double t, double last_relock_t,
                         const Params &p)
{
    if (p.relock_delta <= 0.0)
        return false;
    if (std::fabs(rest_median) > kRestPhysMaxRadps)
        return false;
    if (t - last_relock_t < kRelockCooldownSec)
        return false;
    return std::fabs(rest_median - locked_val) > p.relock_delta;
}

}  // namespace bgz_lock
