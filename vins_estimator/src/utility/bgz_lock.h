#pragma once
// [SW1-1837] Bg_z 잠금 — 상태 기반 발동 판정 로직 (헤더 단독, gtest 가능)
//
// [왜]
//   최적화기가 yaw 불일치(휠 타이밍·저품질 장면 등)를 gyro z-bias(Bg_z)로 도피시켜
//   참값(~1.5e-4 rad/s)의 15~40배로 과대추정하는 것이 yaw 드리프트의 단일 지배 원인
//   (인과 봉인 probe: 수렴 후 Bg_z만 고정 → v7 yaw −24°→−2.7°, xy 개선, 무비용).
//   참 gyro bias는 부팅 캘리브 + DLPF + Allan 실측상 작고 안정하므로, 수렴 후
//   상수로 고정하는 것이 물리적으로 정당하다.
//
// [왜 상태 기반인가 — 시간 기반(고정 delay)의 한계]
//   초기 설계는 "init 후 10초 대기"였으나, 10초는 검증 bag들의 주행 안무(첫 스핀이
//   ~20s)에 의존한 상수였다. B2C 실사용에선 시동 직후 바로 조작할 수 있으므로
//   "정지 + 추정 안정"이라는 상태 조건으로 발동해야 안무와 무관해진다.
//   실측 근거(무잠금 BG 로그): init 직후 Bg_z≈0(미수렴) → 1s에 +8e-3 과도 스파이크
//   → 3~5s에 참값 수렴. 따라서 ①정지 지속 ②추정 안정(스파이크 거부) ③크기 가드의
//   3중 조건이 필요하다.
//
// [조건이 각각 막는 실패]
//   정지 지속   : 움직임(회전) 중 발동 — 인플레 진행 중인 값을 잠그는 사고
//   추정 안정   : 1s 과도 스파이크·미수렴 값 — 가드 아래로 통과하는 작은 오값 잠금
//   크기 가드   : 이미 부풀어버린 값(인플레 2e-3+) 잠금 — 6월 ZUPT 과제약 사고 교훈.
//                 spin-first 세션(init 직후 즉시 회전→인플레→marg 고착)은 이 가드가
//                 계속 보류시켜 세션 내내 미발동 = 기본 동작과 동일(우아한 실패, 수용).
//
// [폴백 경로 — 오염 세션(참 bias 큰 세션) 대응, 07-22 v5 회귀 실패로 추가]
//   v5류 세션은 완전 정지 중에도 Bg_z가 프레임마다 ±0.5e-3 요동(2s 창 변동폭이 문턱의
//   10~46배)해 안정 조건이 영원히 불통과 → 잠금 실패 = 무잠금 인플레 복귀. 처방:
//   정지 관찰이 충분히(fallback_sec) 쌓이면 요동의 '중앙값 근방을 지나는 프레임'에서
//   잠근다 — 상태를 덮어쓰지 않고 좋은 프레임을 고르는 방식이라 marg 일관성 위험 없음,
//   시간 기반(요동에서 무작위 추첨)보다 결정론적. 중앙값은 스파이크에 강건.
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
    double still_sec;       // 정지 지속 요구 시간 = 안정성 판정 창 길이
    double stab_max_radps;  // 창 내 Bg_z 변동폭(max-min) 허용 상한 (빠른 경로)
    double fallback_sec;    // 정지 누적이 이 시간을 넘으면 중앙값 폴백 발동 허용
    double max_radps;       // 발동 시점 |Bg_z| 허용 상한 (<=0이면 항상 거부 = 비활성)
};

// 폴백에서 "중앙값 근방 프레임" 판정 허용 오차. 요동 세션(±5e-4)에서 수 프레임 내
// 충족되는 크기이면서, 잠금값-중앙값 차이를 참값 잔차 대비 무시 가능하게 유지.
constexpr double kSnapTolRadps = 1e-4;

// 프레임마다 update()를 호출해 상태를 누적하고, 발동 조건(빠른 경로 또는 폴백)이
// 성립하는 첫 순간에 true를 반환한다. 발동 이후의 래치는 호출측(estimator) 책임.
class Tracker
{
public:
    // t: 현재 프레임 시각 [s], bgz: 현재 Bg_z 추정치 [rad/s]
    // still_now: 이 순간의 정지 판정(회전·병진 모두 문턱 이하) — 판정 기준은 호출측
    bool update(double t, double bgz, bool still_now, const Params &p)
    {
        if (p.max_radps <= 0.0)
            return false;

        if (first_t_ < 0.0)
            first_t_ = t;

        // 정지 연속 구간 추적 — 움직이면 리셋(부분 정지 이어붙이기 금지)
        if (still_now)
        {
            if (still_since_ < 0.0)
                still_since_ = t;
        }
        else
        {
            still_since_ = -1.0;
            hist_.clear();  // 움직인 순간 이전 이력은 안정성·중앙값 근거로 무효
            return false;
        }

        // Bg_z 이력 — 현재 정지 구간 전체 보존(중앙값용). 메모리 상한으로 앞을 자름.
        hist_.emplace_back(t, bgz);
        while (hist_.size() > kMaxHist)
            hist_.pop_front();

        if (t - first_t_ < p.min_wait_sec)
            return false;
        if (t - still_since_ < p.still_sec)
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
            return std::fabs(bgz) < p.max_radps;

        // ── 폴백: 정지 관찰 누적 + 중앙값 근방 프레임 + 중앙값 자체도 가드 통과 ──
        if (p.fallback_sec > 0.0 && t - still_since_ >= p.fallback_sec)
        {
            std::vector<double> v;
            v.reserve(hist_.size());
            for (const auto &s : hist_)
                v.push_back(s.second);
            std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
            const double med = v[v.size() / 2];
            if (std::fabs(med) < p.max_radps && std::fabs(bgz - med) < kSnapTolRadps)
                return std::fabs(bgz) < p.max_radps;
        }
        return false;
    }

private:
    static constexpr size_t kMaxHist = 4096;  // ~15Hz 기준 4분+ — 정지 구간 커버 충분
    double first_t_{-1.0};                    // 첫 update 시각 (min_wait 기준)
    double still_since_{-1.0};                // 현재 정지 연속 구간의 시작 시각
    std::deque<std::pair<double, double>> hist_;  // (t, bgz) — 현재 정지 구간 한정
};

}  // namespace bgz_lock
