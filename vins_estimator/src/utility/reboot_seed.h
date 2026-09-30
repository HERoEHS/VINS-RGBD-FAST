#pragma once
// [SW1-1866 TASK-20260804-reboot-pose-seed] 재초기화 pose 시드 계승 — 순수 수학부.
//
// 배경: 4선 방어의 최종 계단(조기 재초기화)이 발동하면 clearState로 위치가 원점
//   복귀해 nav에 점프 충격을 준다(실기 08-04 재확인). 재초기화 자체는 실증된 필수
//   방어(11m→0.1m)라 유지하고, "어디서 다시 시작하나"만 고친다: 마지막 건전 pose를
//   시드로 캡처해 발행단에서 T_seed ∘ (새 세션 pose)로 합성한다. 추정기 내부는
//   원점 재시작 그대로(무접촉 — 회귀 위험 최소, 롤백=오프셋 제거). ALICE reanchor
//   (map→odom 1회 고정, doc/SW2_LIFELONG_SLAM_SURVEY.md §1)와 동형 패턴.
//
// 이 파일은 상태 없는 순수 함수만 둔다(gtest 대상, Q7). 시드 선택·저장은 estimator.

#include <Eigen/Dense>
#include <algorithm>   // std::min — contaminationOnset, std::upper_bound
#include <cmath>
#include <deque>       // [SW1-1938] 시각 기준 다리 누적기 이력
#include <utility>

namespace reboot_seed
{

// yaw만 남긴 회전 — 시드는 xy·z 병진 + yaw만 계승한다. roll/pitch는 새 세션의
// 중력 정렬이 더 참값이므로(재초기화의 존재 이유가 자세 오염) 계승하지 않는다.
inline Eigen::Matrix3d yawOnly(const Eigen::Matrix3d &R)
{
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    Eigen::Matrix3d Y;
    const double c = std::cos(yaw), s = std::sin(yaw);
    Y << c, -s, 0, s, c, 0, 0, 0, 1;
    return Y;
}

// 발행 합성: published = T_seed ∘ session.
//   p_out = R_seed * p + t_seed,  R_out = R_seed * R
inline void compose(const Eigen::Matrix3d &R_seed, const Eigen::Vector3d &t_seed,
                    Eigen::Vector3d &p, Eigen::Matrix3d &R)
{
    p = R_seed * p + t_seed;
    R = R_seed * R;
}

// 다리(bridge) 병진: 시드 캡처~재init 완료 사이 로봇 이동을 휠 odom pose 델타로
// 보정. 휠 델타는 휠-odom 프레임 값이므로, 캡처 시점의 (시드 yaw − 휠 yaw) 만큼
// 돌려 발행 프레임으로 옮긴다. 휠 odom은 VINS reboot와 무관한 외부 노드라 연속.
inline Eigen::Vector3d bridgeTranslation(double seed_yaw, double wheel_yaw_at_capture,
                                         const Eigen::Vector2d &wheel_delta_xy)
{
    const double a = seed_yaw - wheel_yaw_at_capture;
    const double c = std::cos(a), s = std::sin(a);
    return Eigen::Vector3d(c * wheel_delta_xy.x() - s * wheel_delta_xy.y(),
                           s * wheel_delta_xy.x() + c * wheel_delta_xy.y(), 0.0);
}

// 시드 T 완성: 캡처된 발행 프레임 시드 pose(p_seed, yaw_seed)에 다리(병진 delta_p,
// yaw 회전 delta_yaw = raw gyro 적분 델타)를 얹어 새 세션 원점이 앉을 자리를 만든다.
//   T_seed = Trans(p_seed + delta_p) ∘ Rz(yaw_seed + delta_yaw)
inline void finalizeSeed(const Eigen::Vector3d &p_seed, double yaw_seed,
                         const Eigen::Vector3d &delta_p, double delta_yaw,
                         Eigen::Matrix3d &R_out, Eigen::Vector3d &t_out)
{
    const double yaw = yaw_seed + delta_yaw;
    const double c = std::cos(yaw), s = std::sin(yaw);
    R_out << c, -s, 0, s, c, 0, 0, 0, 1;
    t_out = p_seed + delta_p;
}

// 오염 시작 시각 = 절제 실증과 발산 가드 감지 중 **먼저 온 쪽**. 둘 다 없으면 -1.
//
// [08-12 SW1-1866] 왜 둘을 합치나:
//   원래 오염 실증은 '첫 절제'뿐이었고, 그 밑에는 "절제가 없었다 = 직전 상태가
//   건강하다"는 전제가 깔려 있었다(무절제 failure는 big bias 등이라 신선한 정화
//   pose가 우월 — 강제 reboot A/B로 실증된 설계).
//   발산 가드(use_still_drift_guard)는 그 전제가 깨지는 **제3의 경우**를 만든다:
//   정지 확정인데 VINS가 창 안에서 상한 이상 움직인 상태 = **오염됐는데 절제는 없는**
//   재부팅이다. 이때 선택자가 '건강'으로 오판해 정화pose를 물면 오염된 pose를 그대로
//   계승한다. 28런 A/B 실측: 가드 ON이 정화pose 분기를 3/14 → 8/14로 늘렸고,
//   시드가 크게 튄 두 런(‖xy‖ 5.663m·0.634m)은 **전부 정화pose 분기**였다.
//   앵커 분기는 28런 내내 0.21~0.31m로 얌전했다.
// 오염 시작 시각을 **한 번만** 기록한다(에피소드 첫 실증 시각 고정). 해제는 호출부의
// 에피소드 종료(정화 solve) 한 곳에서만.
// 왜 헬퍼로 빼는가: `if (consec == 0)`처럼 비슷해 보이는 조건으로 쓰면 초과→미달→재초과
//   때 시작 시각이 뒤로 밀리고, 그 사이에 래치된 앵커가 부당하게 자격을 얻어 Q4 방어가
//   조용히 약해진다. 08-12에 실제로 그렇게 썼다가 A/B 직전 점검에서 잡았다.
inline void recordOnsetOnce(double &onset, double t)
{
    if (onset < 0.0)
        onset = t;
}

inline double contaminationOnset(double first_amputate_t, double drift_detect_t)
{
    if (first_amputate_t < 0.0) return drift_detect_t;
    if (drift_detect_t   < 0.0) return first_amputate_t;
    return std::min(first_amputate_t, drift_detect_t);
}

// 1순위(정지 창 앵커) 채택 자격 — Q4 방어: 앵커가 오염 시작 이전에 래치됐을 때만
// 신뢰한다. 오염 실증이 아직 없으면(first_contam_t < 0) 래치 유효성만 본다.
// ※둘째 인자는 '첫 절제'가 아니라 **오염 시작**이다(위 contaminationOnset 참조).
inline bool anchorSeedEligible(double anchor_latch_t, double first_contam_t)
{
    if (anchor_latch_t < 0.0)
        return false;
    return first_contam_t < 0.0 || anchor_latch_t < first_contam_t;
}

// ── 출력 map 핀 (SW1-1866 vins-output-map-anchor) ──
// yaw 회전 + 병진의 일반 합성 한 단계: p ← Rz(yaw)·p + t,  R ← Rz(yaw)·R.
// 표시 핀 사슬 published = T(map→odom) ∘ T(odom←세션) ∘ pose 를 이 함수 2회로 구성.
// yaw-only인 이유: 두 프레임 모두 중력 정렬이라 roll/pitch 성분은 ~0(핀의 z만 병진 반영).
inline void composeYawXYZ(double yaw, const Eigen::Vector3d &t, Eigen::Vector3d &p,
                          Eigen::Matrix3d &R)
{
    const double c = std::cos(yaw), s = std::sin(yaw);
    Eigen::Matrix3d Y;
    Y << c, -s, 0, s, c, 0, 0, 0, 1;
    p = Y * p + t;
    R = Y * R;
}

// ── 발행 앵커 시각 정합 (SW1-1866 08-09 output-anchor-time-consistency) ──
//
// [고친 결함] 구 구현은 T(odom←세션)을 'VINS init 순간'에, T(map→odom) 핀을 '첫 태그
//   검출 순간'에 캡처해 놓고 두 변환을 동시각인 양 곱했다. odom 은 map 대비 고정
//   프레임이 아니라(휠 오도메트리가 흐른다) 두 시각 사이의 드리프트가 발행 pose 에
//   영구 상수 오프셋으로 굳는다 — 실기 실측으로 핀이 init+24초에 온 세션에서 궤적이
//   통째로 회전(12.66°면 3m 지점 0.66m). 그래서 "태그가 보이는 상태로 켜야만" 맞았다.
//
// [올바른 정의] 표시 변환을 '핀과 세션 pose 가 모두 유효한 한 시점 t_a'에서 1회 정한다.
//     T_display = W(t_a) ∘ S(t_a)⁻¹
//       W(t_a) = T(map→odom) ∘ T(odom←base)(t_a)   그 순간 휠이 말하는 map 상 base pose
//       S(t_a) = (T_seed ∘ session)(t_a)           그 순간 발행 직전 pose
//   이러면 t_a 에 발행 pose 가 W 와 정확히 일치하고, init 과 핀의 시각차가 무관해진다.
//
// [회귀 없음] S = I (핀이 init 보다 먼저 도착 = 기존 정상 경로)이면
//   disp_yaw = map_odom_yaw + wheel_yaw, disp_t = W 의 위치가 되어 구 2단 합성과
//   수식이 정확히 일치한다. gtest 로 고정.
//
// yaw-only 인 이유는 composeYawXYZ 와 같다(두 프레임 모두 중력 정렬 → roll/pitch ~0).
inline void computeDisplayAnchor(double map_odom_yaw, const Eigen::Vector3d &map_odom_t,
                                 double wheel_yaw, const Eigen::Vector3d &wheel_t,
                                 const Eigen::Vector3d &p_s, const Eigen::Matrix3d &R_s,
                                 double &disp_yaw, Eigen::Vector3d &disp_t)
{
    const double yaw_s = std::atan2(R_s(1, 0), R_s(0, 0));
    disp_yaw           = map_odom_yaw + wheel_yaw - yaw_s;

    // W 의 위치 = Rz(map_odom_yaw)·wheel_t + map_odom_t
    const double cm = std::cos(map_odom_yaw), sm = std::sin(map_odom_yaw);
    const Eigen::Vector3d w_p(cm * wheel_t.x() - sm * wheel_t.y() + map_odom_t.x(),
                              sm * wheel_t.x() + cm * wheel_t.y() + map_odom_t.y(),
                              wheel_t.z() + map_odom_t.z());

    // disp_t = W_p − Rz(disp_yaw)·S_p  (그래야 Rz(disp_yaw)·S_p + disp_t = W_p)
    const double cd = std::cos(disp_yaw), sd = std::sin(disp_yaw);
    disp_t = Eigen::Vector3d(w_p.x() - (cd * p_s.x() - sd * p_s.y()),
                             w_p.y() - (sd * p_s.x() + cd * p_s.y()),
                             w_p.z() - p_s.z());
}

// ── 다리(bridge) 시작점 정렬 (SW1-1936 reboot-seed-bridge-align) ──
//
// [고친 결함] 옛 다리는 **캡처 시각부터** 회전·휠 병진을 얹었다. 그런데 시드 재료(앵커 pose,
//   정화 pose)는 캡처보다 이른 시각의 자세다. 앵커를 잡은 뒤 로봇을 들어 돌리고 내려놓으면
//   가드가 발동해 앵커를 시드로 쓰는데, 그 사이 회전이 다리에서 통째로 빠져 발행 yaw 가
//   들리기 전 방향으로 되돌아갔다(실기 vpr4 79.5°·kidnap_0922 약 68° 상실, SW1-1936).
//   해법: 재료마다 "그 자세의 시각"에 다리 누적기 값과 휠 pose 를 스냅샷해 두고, 다리를
//   '시드 자세 시각 → 확정'으로 얹는다. 진짜 정지 폭주(앵커 뒤 회전 ≈0)에서는 얹을 것이
//   없으므로 앵커의 원래 목적(폭주 전 자세 복귀)은 그대로다.

// 시드 재료 하나의 다리 시작점 — 그 자세를 잡은 순간의 누적기 값과 휠 pose.
struct BridgeSnap
{
    double t{-1.0};          // 스냅샷 시각(진단용, <0 = 없음)
    double gyro_yaw{0.0};    // MotionGatedYaw::value()
    double wheel_x{0.0}, wheel_y{0.0}, wheel_yaw{0.0};
};

// 운동 구간만 적분하는 다리 yaw 누적기.
//   왜 운동 구간만: 앵커는 몇 분씩 묵을 수 있다. 원시 적분이면 bias 가 경과 시간만큼
//   쌓이고(−0.00035 rad/s × 600 s ≈ 12°), 정지 bias 를 빼도 그 값이 틀리면 같은 문제다
//   (BGZ-LOCK 잠금값은 수백 초 갱신이 없을 수 있고 vpr4 에서 정지 실측과 1.35e-4 차).
//   유휴 동안의 기여를 0 으로 만들면 bias 오차는 실제로 움직인 몇 초에만 곱해진다.
//   게이트는 **원시 자이로와 휠만** 본다 — VINS 상태(Vs·Bgs)는 재초기화 직전에 오염돼
//   있을 수 있어 판정 근거로 쓰지 않는다.
//   한계: 블록 평균 노름 0.02 rad/s(약 1.1°/s) 미만의 느린 외부 회전은 휠이 멈춰 있으면
//   버려진다.
class MotionGatedYaw
{
public:
    static constexpr double kBlockSec   = 0.1;   // 판정 블록 길이
    static constexpr double kGateRadps  = 0.02;  // 블록 평균 원시 자이로 노름 문턱
    static constexpr double kDtSaneMax  = 0.1;   // 세션 경계 epoch dt 제외(기존 dt 위생과 동일)

    // IMU 표본 1개. gyr_raw = 원시 각속도(bias 미차감), wheel_moving = 이 표본 시점 휠 이동,
    //   bias_z = 차감할 z bias(정지 실측 스냅샷, 없으면 0).
    void add(double dt, const Eigen::Vector3d &gyr_raw, bool wheel_moving, double bias_z)
    {
        if (!(dt > 0.0 && dt < kDtSaneMax))
            return;
        blk_dt_ += dt;
        blk_norm_dt_ += gyr_raw.norm() * dt;
        blk_dyaw_ += (gyr_raw.z() - bias_z) * dt;
        blk_wheel_ = blk_wheel_ || wheel_moving;
        if (blk_dt_ >= kBlockSec)
        {
            if (blk_wheel_ || blk_norm_dt_ / blk_dt_ >= kGateRadps)
                total_ += blk_dyaw_;
            blk_dt_ = blk_norm_dt_ = blk_dyaw_ = 0.0;
            blk_wheel_ = false;
        }
    }

    // 반영분 + 진행 중 블록(게이트 전). 두 스냅샷의 차가 다리 회전이다. 진행 중 블록을
    //   게이트 없이 더하므로 스냅샷 경계 오차는 블록 1개(0.1 s)분 이하다.
    double value() const { return total_ + blk_dyaw_; }

private:
    double total_{0.0};
    double blk_dt_{0.0}, blk_norm_dt_{0.0}, blk_dyaw_{0.0};
    bool   blk_wheel_{false};
};

// 다리 시작점 선택 — align 이면 재료 스냅샷(시드 자세 시각), 아니면 옛 동작(캡처 시각).
//   now 는 캡처 순간의 값(옛 동작의 기준). 재료 스냅샷이 없으면(t<0) 캡처 시각으로 폴백.
inline BridgeSnap bridgeStart(bool align, const BridgeSnap &material, const BridgeSnap &now)
{
    return (align && material.t >= 0.0) ? material : now;
}

// ============================================================================
// [SW1-1938] 재초기화 경계 — 다리 끝을 '새 세션 원점 시각'에 맞추기
// ============================================================================
//
// [고친 결함] 다리를 확정(finalize, 새 세션 초기화 완료) 시각까지 얹으면 두 가지가 어긋난다.
//   ① 미계산: clearState 가 imu_buf 를 비워, 캡처 프레임 ~ 비우는 순간 사이 이미 받아 둔
//      IMU 표본(재생 0.15~0.22 s)이 다리에도 새 세션에도 안 들어간다.
//   ② 이중 계산: 새 세션의 yaw 원점(운영 static init = 재부팅 뒤 첫 프레임, 동적 = 초기화
//      성공 때 창의 첫 프레임)은 확정보다 이르다. [원점, 확정] 회전을 다리와 새 세션이 두 번 센다.
//   남는 yaw = 이중 − 미계산 + 세션 추정 오차 → 로봇이 도는 중에 재초기화되면 수 도(재생
//   kidnap −7.3°·+3.1°, 실기 공중 연쇄 −2.2°). 가만히 있을 때의 재초기화는 0 이라 모르고 지나갔다.
//   해법: 누적기를 **표본 시각 기준**으로 한 번씩만 먹이고 값의 이력을 남겨, 다리를
//   '시드 자세 시각 → 새 세션 원점 시각'으로 얹는다. 비우는 표본도 버리기 전에 먹인다.
//
// 왜 표본 시각 기준인가: 재부팅 뒤 새 세션은 비운 표본보다 이른 시각의 프레임부터 다시
//   처리한다(프레임은 처리 지연만큼 뒤처져 있다). 그 프레임들에는 IMU 가 없어 VINS 는 첫 새
//   표본 하나를 거짓 dt 로 되풀이 적분한다. VINS 가 넘겨 주는 dt 를 그대로 쓰면 누적기도
//   그 구간을 두 번 센다. 표본 시각이 앞으로 갈 때만 더하면 표본 하나는 딱 한 번만 들어간다.
class TimedSeedYaw
{
public:
    static constexpr double kHistSec = 60.0;  // 이력 보존 길이 — 재부팅~확정(워밍업 대기 포함)을 덮게

    // 표본 1개(시각 t, 원시 각속도). last_t 이하 시각은 무시한다. 간격이 비정상(>= 0.1 s,
    //   세션 경계·시계 계단)이면 MotionGatedYaw 가 버리고 시각만 이어 받는다.
    void add(double t, const Eigen::Vector3d &gyr_raw, bool wheel_moving, double bias_z)
    {
        if (last_t_ >= 0.0 && !(t > last_t_))
            return;
        const double dt = (last_t_ >= 0.0) ? t - last_t_ : 0.0;
        last_t_ = t;
        acc_.add(dt, gyr_raw, wheel_moving, bias_z);
        hist_.emplace_back(t, acc_.value());
        while (!hist_.empty() && hist_.front().first < t - kHistSec)
            hist_.pop_front();
    }

    double value() const { return acc_.value(); }
    double lastT() const { return last_t_; }

    // 시각 t 의 누적기 값(이웃 두 표본 사이 선형 보간). 이력 범위 밖이면 false.
    bool valueAt(double t, double *out) const
    {
        if (hist_.empty() || t < hist_.front().first || t > hist_.back().first)
            return false;
        auto hi = std::upper_bound(hist_.begin(), hist_.end(), t,
                                   [](double v, const std::pair<double, double> &e) { return v < e.first; });
        if (hi == hist_.begin())
        {
            *out = hist_.front().second;
            return true;
        }
        auto lo = hi - 1;
        if (hi == hist_.end() || hi->first <= lo->first)
        {
            *out = lo->second;
            return true;
        }
        const double a = (t - lo->first) / (hi->first - lo->first);
        *out = lo->second + a * (hi->second - lo->second);
        return true;
    }

private:
    MotionGatedYaw acc_;
    double         last_t_{-1.0};
    std::deque<std::pair<double, double>> hist_;  // (표본 시각, 그 표본까지의 누적기 값)
};

// 프레임별 휠 pose 이력 — 다리 병진을 원점 프레임에서 끊기 위해. 휠 pose 는 받은 최신값이라
//   프레임 처리 순간의 값을 그 프레임 시각 키로 남긴다(다리 시작 스냅샷과 같은 규약).
class FrameWheelHistory
{
public:
    static constexpr double kHistSec = 60.0;
    static constexpr double kTolSec  = 0.02;  // 원점 시각과 프레임 키의 허용 차(td 추정 흔들림)

    void add(double t, double wx, double wy, double wyaw)
    {
        if (!hist_.empty() && !(t > hist_.back().t))
            return;
        hist_.push_back(BridgeSnap{t, 0.0, wx, wy, wyaw});
        while (!hist_.empty() && hist_.front().t < t - kHistSec)
            hist_.pop_front();
    }

    // 시각 t 에 가장 가까운 프레임 기록(차 <= kTolSec). 없으면 false.
    bool at(double t, BridgeSnap *out) const
    {
        const BridgeSnap *best = nullptr;
        double best_d = kTolSec;
        for (const auto &e : hist_)
        {
            const double d = std::abs(e.t - t);
            if (d <= best_d)
            {
                best_d = d;
                best = &e;
            }
        }
        if (!best)
            return false;
        *out = *best;
        return true;
    }

private:
    std::deque<BridgeSnap> hist_;
};

}  // namespace reboot_seed
