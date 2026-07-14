#pragma once

#include <cmath>
#include <deque>
#include <utility>

// [SW1-1837] 다리 이벤트 게이팅 detector (Phase 1)
//
// 근거(07-13 실측, odom_fix_check bag): 다리 인출입(t=40~45s, 0.33rad 스윙) 순간
// VINS 자세 오차가 0.04°→1.6~3.4°로 일괄 주입된 뒤 끝까지 유지됨(3-run 재현).
// 다리가 움직이는 동안 휠(v=w=0 주장)·plane(수평 강제)·vert(vz=0 강제)가
// 전부 '틀린 제약'이 되므로, 그 시각 구간의 factor를 skip하기 위한 구간 마킹을 담당.
//
// ★시작 판정은 '정착 기준점 대비 변위'(pos_min) — 순간 변화율 기반은 다리 엔코더
//   1-LSB(0.017rad) 플리커가 100Hz에서 1.7rad/s로 보여 오발동함(07-13 A/B 1차 실측:
//   가짜 이벤트 6건 → z범위 악화). pos_min을 2 LSB 이상으로 두면 플리커 면역.
//   종료(정착) 판정만 순간 변화율(rate_min)을 사용.
//
// 순수 로직(ROS/Eigen 미의존) — gtest 단위검증 대상. 스레드 안전 아님(호출측 mutex).
class LegEventDetector
{
public:
    struct Params
    {
        double pos_min      = 0.03;  // [rad] 정착 기준점 대비 변위 시작 임계 (엔코더 LSB 0.017의 ~2배)
        double rate_min     = 0.05;  // [rad/s] 진행 중 '아직 움직임' 판정 임계(종료·정착용)
        double cmd_pos_min  = 0.02;  // [rad] 명령-실측 차 시작 임계(선행 트리거)
        double pre_margin   = 0.3;   // [s] 시작 소급 마진(변위 임계 도달 지연 보상)
        double post_margin  = 0.5;   // [s] 종료 후 유지 마진(정착 진동 흡수)
        double max_duration = 2.0;   // [s] 연속 게이팅 상한 — 휠 앵커 상실 발산 방지
        double history      = 30.0;  // [s] 종료 구간 보존 창(슬라이딩 윈도우보다 충분히 길게)
    };

    void setParams(const Params &p) { p_ = p; }

    // 실측 다리각 입력 (joint_states, t 단조 증가 가정)
    void onMeasurement(double t, double theta_l, double theta_r)
    {
        if (!has_prev_)
        {
            prev_t_ = last_rate_move_t_ = t;
            prev_l_ = ref_l_ = theta_l;
            prev_r_ = ref_r_ = theta_r;
            has_prev_ = true;
            return;
        }
        const double dt = t - prev_t_;
        if (dt <= 1e-4)
            return;  // dt 극소 → 변화율 판정 보류
        const double rate = std::max(std::fabs(theta_l - prev_l_),
                                     std::fabs(theta_r - prev_r_)) / dt;
        prev_t_ = t;
        prev_l_ = theta_l;
        prev_r_ = theta_r;
        if (rate > p_.rate_min)
            last_rate_move_t_ = t;

        if (active_)
        {
            if (rate > p_.rate_min)
                last_move_t_ = t;
            if (t - open_t_ > p_.max_duration)
            {
                // 상한 초과 강제 종료 — 휠 factor를 무한정 빼면 스케일 앵커 상실로 발산(07-01 실측 70m).
                // 기준점을 현재 위치로 재고정해 잔여 변위가 즉시 재트리거하는 것을 막고,
                // 정착(rate 조용 post_margin) 관측 전엔 재무장하지 않는다.
                close(open_t_ - p_.pre_margin, open_t_ + p_.max_duration);
                active_ = false;
                force_closed_ = true;
                cooldown_ = true;
                ref_l_ = theta_l;
                ref_r_ = theta_r;
            }
            else if (t - last_move_t_ > p_.post_margin)
            {
                // 정착 → 종료. 새 자세를 기준점으로 고정.
                close(open_t_ - p_.pre_margin, last_move_t_ + p_.post_margin);
                active_ = false;
                ref_l_ = theta_l;
                ref_r_ = theta_r;
            }
        }
        else
        {
            // 정착 상태가 post_margin 이상 유지되면 기준점 재고정 + 재무장
            if (t - last_rate_move_t_ > p_.post_margin)
            {
                ref_l_ = theta_l;
                ref_r_ = theta_r;
                cooldown_ = false;
            }
            const bool displaced = std::fabs(theta_l - ref_l_) > p_.pos_min ||
                                   std::fabs(theta_r - ref_r_) > p_.pos_min;
            if (displaced && !cooldown_)
            {
                active_ = true;
                open_t_ = t;
                last_move_t_ = t;
                ++activations_;
            }
        }
        prune(t);
    }

    // 다리 위치 '명령' 입력 — 목표가 현재 실측과 다르면 실측 반응 전에 게이트 개시
    void onCommand(double t, double target, bool left)
    {
        if (!has_prev_ || active_ || cooldown_)
            return;
        const double cur = left ? prev_l_ : prev_r_;
        if (std::fabs(target - cur) > p_.cmd_pos_min)
        {
            active_ = true;
            open_t_ = t;
            last_move_t_ = t;
            ++activations_;
        }
    }

    // [t0, t1] 구간이 게이팅 구간(마진 포함)과 겹치는가 — factor skip 판정용
    bool overlaps(double t0, double t1) const
    {
        if (active_ && t1 >= open_t_ - p_.pre_margin)
            return true;  // 진행 중 이벤트: [open-pre, 현재진행형)
        for (const auto &iv : closed_)
            if (t1 >= iv.first && t0 <= iv.second)
                return true;
        return false;
    }

    int  activationCount() const { return activations_; }
    bool everForceClosed() const { return force_closed_; }

private:
    void close(double a, double b) { closed_.emplace_back(a, b); }

    void prune(double t)
    {
        while (!closed_.empty() && closed_.front().second < t - p_.history)
            closed_.pop_front();
    }

    Params p_;
    bool   has_prev_ = false, active_ = false, cooldown_ = false, force_closed_ = false;
    double prev_t_ = 0.0, prev_l_ = 0.0, prev_r_ = 0.0;
    double ref_l_ = 0.0, ref_r_ = 0.0;      // 정착 기준점(변위 시작 판정의 원점)
    double last_rate_move_t_ = 0.0;         // 마지막으로 순간 변화율이 임계를 넘은 시각
    double open_t_ = 0.0, last_move_t_ = 0.0;
    std::deque<std::pair<double, double>> closed_;  // 종료된 [시작-pre, 종료+post] 구간들
    int activations_ = 0;
};
