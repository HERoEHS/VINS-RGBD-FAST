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

// ── 누적 변위 클램프 (07-30, obs_v2 실증으로 추가) ──
//   per-solve 문턱(위 isPosSlide)은 '속도' 제한이라, 문턱 이하로 같은 방향을 지속하는
//   압력(obs_v2 말미: 사람이 30s 서 있는 조건서 p50 2~3.5mm/solve)은 못 막는다 —
//   '문턱=누설률'의 병진판(실측 32s에 0.32m 누적). 이 함수는 '총량' 제한:
//   정지 창 앵커 대비 xy 이탈이 max_m를 넘으면 경계로 되돌리는 보정 벡터를 준다.
//   근거는 분포(bag 의존)가 아니라 '휠+gyro가 창 전체에서 Δpose=0'이라는 외부 사실.
//   초과분만 환원(경계 안쪽 대역은 치유용 자유) / z는 중력·plane 관할이라 불간섭.
inline Eigen::Vector3d cumClampCorrection(const Eigen::Vector3d &dev, double max_m)
{
    const Eigen::Vector3d dxy(dev.x(), dev.y(), 0.0);
    const double n = dxy.norm();
    if (n <= max_m)
        return Eigen::Vector3d::Zero();
    return -dxy * ((n - max_m) / n);
}

// ── 절제 에스컬레이션 판정 (07-31, 실기 정지 폭주 2건 실증으로 추가) ──
//   전제 붕괴 실측: prior 절제는 '다음 solve가 건강한 현재 상태로 재구축'이 전제인데,
//   실기 2건에서 절제 7·15회 반복에도 견인이 지속·증폭(절제 직후 pos_slide
//   0.56~0.65m/solve) — 압력원이 prior 밖(활성 잔차)에 있으면 절제는 무한 반복이고,
//   결말은 어차피 failure detection→reboot인데 그 전에 11~13m 폭주가 발행된다.
//   → '정화(이상 없는 solve) 없는 연속 절제 max회' 도달 시 조기 재초기화 신호를 준다.
//   상태 조건(시간 상수 아님) — 정상 주행 절제 0회 실측(v8·v9)이라 오발 여지 낮음.
inline bool escalationReached(int amputate_streak, int max)
{
    return max > 0 && amputate_streak >= max;  // max<=0 = 비활성
}

}  // namespace yaw_slide_guard
