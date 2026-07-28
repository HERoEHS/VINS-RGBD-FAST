#pragma once
// [SW1-1866] yaw 게이지 슬라이드 가드 — 핵심 수학 (헤더 단독, gtest 가능)
//
// 배경(edie_dynamic_obs 포렌식으로 확정):
//   초근접 동적 장애물(사람)이 화면을 점령하면 오염이 marg prior에 구워지고, 이후
//   prior의 잔존 yaw 기울기가 매 solve 창 전체를 yaw 게이지 방향으로 일정량(실측
//   13~16°/solve) 회전시킨다. 창 내 잔차(IMU·휠·비전)는 전부 상대 제약이라 강체
//   회전에 불변 = 저항 불가(폭주 중 재투영 오차 8.3px 동결 실측). 결과는 정지
//   로봇의 -90~-160°/s 연속 가짜 회전, 영구 미복구.
//
// 검출 원리: '같은 물리 프레임의 yaw 추정이 solve 사이에 얼마나 이동했는가'.
//   정상 재선형화 이동은 0.0x°(고주기 TF의 미세 점프와 동일 량), 슬라이드는 13~16°
//   → 문턱 3°로 ~100배 분리. 실제 회전(스핀)은 과거 프레임 추정을 움직이지 않으므로
//   오탐 없음. 시간 상수 불필요(상태 조건 기반).
//
// 처치 원리: 검출량만큼 창 전체+marg prior 선형화점을 z축 역회전(중력 재정렬 v2와
//   동일 기계, 축만 z 고정). 슬라이드가 게이지 방향이므로 역회전도 창 내 비용 불변.
#include <Eigen/Dense>
#include <cmath>

namespace yaw_slide_guard
{

// yaw 차이를 [-180, 180]°로 감아 계산 (±180 경계 통과 시 359° 오검출 방지)
inline double wrappedDeltaDeg(double now_deg, double prev_deg)
{
    return std::remainder(now_deg - prev_deg, 360.0);
}

// 슬라이드 판정 — 정상 재선형화(0.0x°)와 슬라이드(13~16°) 사이 문턱
inline bool isSlide(double slide_deg, double thresh_deg)
{
    return std::fabs(slide_deg) > thresh_deg;
}

// 검출된 슬라이드를 되돌리는 z축 역회전
inline Eigen::Matrix3d counterRotation(double slide_deg)
{
    return Eigen::AngleAxisd(-slide_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())
        .toRotationMatrix();
}

// ── 병진 게이지 슬라이드 (07-27 A/B 실증으로 추가) ──
//   yaw만 봉쇄하면 오염 압력이 병진 게이지(전역 위치, 역시 불관측 방향)로 빠져나간다:
//   가드 ON A/B서 yaw는 ±1~2.5°로 잡혔지만 정지 창 xy가 0.26~1.5 m/s로 활주(6~23m).
//   검출은 yaw와 동일 원리 — '같은 물리 프레임의 위치가 solve 간 이동한 양'.
//   정상 재선형화는 mm, 슬라이드는 실측 0.03~0.15m/solve.
inline bool isPosSlide(const Eigen::Vector3d &d, double thresh_m)
{
    return d.norm() > thresh_m;
}

// ceres pose 블록(x y z ...)의 위치만 평행이동 — 병진 역슬라이드용
inline void translatePoseBlock(double *pose, const Eigen::Vector3d &d)
{
    pose[0] += d.x();
    pose[1] += d.y();
    pose[2] += d.z();
}

}  // namespace yaw_slide_guard
