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

// 1순위(정지 창 앵커) 채택 자격 — Q4 방어: 앵커가 오염 시작(첫 절제) 이전에
// 래치됐을 때만 신뢰한다. 절제가 아직 없으면(first_amputate_t < 0) 래치 유효성만.
inline bool anchorSeedEligible(double anchor_latch_t, double first_amputate_t)
{
    if (anchor_latch_t < 0.0)
        return false;
    return first_amputate_t < 0.0 || anchor_latch_t < first_amputate_t;
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

}  // namespace reboot_seed
