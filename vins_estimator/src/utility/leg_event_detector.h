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
// ★종료(정착) 판정의 실체(07-14): 다리각은 펌웨어가 정수 도(°) 단위로 보고하므로
//   (fifo_comms.hpp int32_t → edie_hardware.cpp:381 ×π/180 = 구조적 1°(0.0175rad) 격자,
//   bag 관측이 아닌 타입 보장) 순간 변화율은 '0 또는 스파이크(1°/dt)'뿐 — 실측으로도
//   스윙 중 0.05~1.0rad/s 구간 샘플 0개. 스파이크 최저값은
//   1-LSB/최대샘플간격 = 0.0175/0.02 = 0.87rad/s(실측 dt 중앙값 10ms·최대 20ms) →
//   rate_min은 (0, 0.87) 구간에서 불감(연속값 엔코더 대비용 일반화 파라미터)이고,
//   정착의 실질 판정은 "스파이크가 post_margin 동안 부재"다(실질 튜닝 노브=post_margin).
//   스윙 중 스파이크 간격 실측 = 중앙값 30ms·최대 50ms → post_margin(0.5s)이 움직임
//   도중 조기 닫힘을 일으킬 여지는 10배 여유로 없음.
//   정착 중 플리커가 튀면 게이트가 버스트당 최대 ~post_margin 연장될 수 있으나
//   (실측 124s 전수: 버스트 ≤0.05s·~20s당 1회, 0.5s-연쇄 최장 0.05s = 연쇄 연장 불가)
//   하드 상한은 max_duration이 보장한다.
//
// 순수 로직(ROS/Eigen 미의존) — gtest 단위검증 대상. 스레드 안전 아님(호출측 mutex).
class LegEventDetector
{
public:
    struct Params
    {
        double pos_min      = 0.03;  // [rad] 정착 기준점 대비 변위 시작 임계 (엔코더 LSB 0.017의 ~2배)
        double rate_min     = 0.05;  // [rad/s] '아직 움직임' 판정(종료용). 최저 스파이크(1-LSB/최대dt≈0.87)보다 낮으면 충분(17배 여유)
        double cmd_pos_min  = 0.02;  // [rad] 명령 '목표 변경량' 시작 임계(선행 트리거) — 직전 목표 대비
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

    // 다리 위치 '명령' 입력 — 목표값이 '직전 목표 대비' 변경됐을 때만 선행 개시.
    // ★07-14 edie_gate_verify bag 실증 수정: 명령은 13.6Hz 연속 스트림이고 다리는
    //   목표에 2°(0.0349rad) 어긋난 채 정착할 수 있다(기계적 스탠드오프). 이전 구현
    //   (|목표-실측| 비교)은 이 지속 오차를 매 메시지 '새 이벤트'로 오인해 무한 재점화
    //   (150s 중 45% 과게이팅 실측). '목표 변경'만이 "새 명령이 떨어졌다"의 올바른 신호.
    void onCommand(double t, double target, bool left)
    {
        double     &prev_cmd = left ? prev_cmd_l_ : prev_cmd_r_;
        bool       &has_cmd  = left ? has_cmd_l_ : has_cmd_r_;
        const bool  first    = !has_cmd;
        const double before  = prev_cmd;
        prev_cmd = target;
        has_cmd  = true;
        if (first)
            return;  // 첫 수신은 기준만 설정
        if (std::fabs(target - before) <= p_.cmd_pos_min)
            return;  // 목표 변경 없음(반복 스트림) → 무시
        if (active_ || cooldown_)
            return;
        active_ = true;
        open_t_ = t;
        last_move_t_ = t;
        ++activations_;
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
    bool   has_cmd_l_ = false, has_cmd_r_ = false;       // 명령 기준값 수신 여부
    double prev_cmd_l_ = 0.0, prev_cmd_r_ = 0.0;         // 직전 명령 목표(변경 감지 기준)
    double prev_t_ = 0.0, prev_l_ = 0.0, prev_r_ = 0.0;
    double ref_l_ = 0.0, ref_r_ = 0.0;      // 정착 기준점(변위 시작 판정의 원점)
    double last_rate_move_t_ = 0.0;         // 마지막으로 순간 변화율이 임계를 넘은 시각
    double open_t_ = 0.0, last_move_t_ = 0.0;
    std::deque<std::pair<double, double>> closed_;  // 종료된 [시작-pre, 종료+post] 구간들
    int activations_ = 0;
};
