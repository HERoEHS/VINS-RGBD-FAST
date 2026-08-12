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
#include <algorithm>   // std::min — contaminationOnset
#include <cmath>

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


}  // namespace reboot_seed
