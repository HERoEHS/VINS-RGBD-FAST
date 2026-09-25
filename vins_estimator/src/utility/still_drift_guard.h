#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cstddef>

// [SW1-1922 09-25] STILL-DRIFT 가드의 "몸체가 실제로 움직였나" 판정 — 순수 수학(헤더-only, gtest 대상)
//
// 배경: STILL-DRIFT 가드(estimator.cpp gaugeSlideGuard)는 휠 끝점(2 s 전 대비 xy·yaw)만으로
//   '지속 정지'를 확정했다. 다리 동작으로 몸체가 기울었다 복귀하면(v15 +217.5~219.6 s,
//   roll 약 11°·pitch 약 9°) 휠은 3 mm 이하로 조용하지만 IMU(몸체)는 0.5 s 창에서 2~3 cm
//   실제로 움직인다 → "정지 중 VINS 폭주"로 오판 → 재초기화 → 이후 0.4~2 m 오차(3/3 재현).
//   같은 파일의 정지 확정(gravityRealignWindow)·clamp 정지 판정은 휠+자이로+다리 게이트
//   3중 검사를 쓰는데 STILL-DRIFT만 빠져 있었다. 이 헤더가 그 빈틈을 메운다.
//
// 설계:
//   ① 자이로는 **노름의 시간 가중 평균**을 쓴다. 벡터 평균은 왕복 운동(기울었다 복귀)에서
//      상쇄되어 0에 가까워지므로 쓰면 안 된다(clamp 정지 판정의 벡터 평균은 한 조각 안의
//      '조용함' 판정용이라 목적이 다르다). 대신 노름 평균은 정지 중 진동(v15 +8~10 s z축
//      4~5 Hz)도 그대로 더하므로 바닥이 벡터 평균보다 높다 — 바닥 실측은 yaml 주석 참조.
//   ② 창은 가드의 **비교 창 길이(now − t_win)** 를 pre_integration 조각들(최신→과거)의
//      dt_buf/gyr_buf를 뒤에서부터 훑어 채운다. 조각 경계에서 멈추지 않는다 — 정지 중엔
//      키프레임이 성겨 한 조각이 수 초일 수도, 수십 ms일 수도 있다.
//   ③ 창을 kMinCoverFrac만큼도 못 채우면 '판단 불가'로 두고 busy=false(기존 동작 유지).
//      조용한 무력화가 되지 않도록 호출측이 covered_sec을 로그에 남긴다.
//   ④ 바이어스 자기참조 방어: VINS가 망가져 Bg 추정이 커지면 ‖gyr−Bg‖가 정지 중에도 커져
//      가드를 스스로 끌 수 있다. 그래서 busy는 **바이어스 차감 평균과 원시 평균이 둘 다**
//      상한 이상일 때만 참이다(정상 Bg는 수 mrad/s라 둘이 거의 같고, 오염된 Bg는 원시 평균이
//      작아 건너뜀이 무효가 된다).
//   ⑤ 다리 경로는 "다리 게이트 겹침" 만으로 끄지 않는다 — 다리 명령 직후·정착 직후는 몸체가
//      조용한데도 게이트가 열려 있어(v15 오프라인: 휠 정지 중 5.4 s, 그중 4.2 s는 창 전체가
//      조용) 가드가 불필요하게 꺼진다. 창 안 0.1 s 블록 최대가 상한 이상(운동 증거)일 때만
//      다리 경로가 성립한다. 이 블록 최대는 창 평균이 희석되는 운동 시작·끝 조각을 잡는다.
namespace still_drift_guard
{
// 창을 이 비율만큼도 못 채우면 판단 불가. 초기화 직후·IMU 공백처럼 표본이 모자란 solve에서
//   '몸체 정지'를 단정하지 않기 위한 하한이지 튜닝 축이 아니다(창 10 조각이면 사실상 도달 못 함).
constexpr double kMinCoverFrac = 0.5;
// 운동 증거용 블록 길이. 0.1 s = 자이로 약 40 표본, 다리 순항(50°/s) 5° 분량 — 창 평균이
//   희석되는 1~2 solve 짜리 시작·끝 조각을 잡기에 충분하고, 단발 스파이크 한 표본에는 반응 안 함.
constexpr double kBlockSec = 0.1;
// IMU 샘플 간격 위생 — 이 파일 processIMU의 dt_sane(0 < dt < 0.1)과 같은 기준. IMU 공백(IMU-GAP)
//   직후의 큰 dt 한 표본이 창을 통째로 채우는 것을 막는다(제외된 시간은 채워지지 않아 판단 불가 쪽).
constexpr double kDtSaneMax = 0.1;

struct GyroWindow
{
    double sum_norm_dt = 0.0;  // Σ ‖gyr − bg‖·dt (바이어스 차감, 시간 가중 합)
    double sum_raw_dt  = 0.0;  // Σ ‖gyr‖·dt (원시, 자기참조 방어용)
    double covered_sec = 0.0;  // 창에 들어온 시간 합
    size_t n           = 0;    // 창에 들어온 샘플 수
    // 블록 최대(운동 증거): 뒤에서부터 kBlockSec 씩 묶은 블록의 노름 평균 최대
    double block_sum   = 0.0;
    double block_cover = 0.0;
    double max_block   = 0.0;

    double meanNorm() const { return covered_sec > 0.0 ? sum_norm_dt / covered_sec : 0.0; }
    double meanRaw() const { return covered_sec > 0.0 ? sum_raw_dt / covered_sec : 0.0; }
    // 마지막 부분 블록은 절반 이상 찼을 때만 센다
    double maxBlockNorm() const
    {
        double m = max_block;
        if (block_cover >= 0.5 * kBlockSec)
            m = std::max(m, block_sum / block_cover);
        return m;
    }
};

// 한 조각의 **뒤에서부터** 창을 채운다. 최신 조각부터 순서대로 호출한다.
//   dt_buf[k]는 gyr_buf[k] 샘플이 대표하는 시간 길이. 창 경계에 걸치는 마지막 샘플은
//   남은 길이만큼만 가중한다(창 길이가 정확히 window_sec이 되도록).
// 반환: 창이 다 찼으면 true(더 이상 과거 조각을 볼 필요 없음).
template <class DtBuf, class GyrBuf>
inline bool accumulateTail(const DtBuf &dt_buf, const GyrBuf &gyr_buf, const Eigen::Vector3d &bg,
                           double window_sec, GyroWindow &w)
{
    const size_t n = std::min(dt_buf.size(), gyr_buf.size());
    for (size_t k = n; k-- > 0;)
    {
        if (w.covered_sec >= window_sec)
            return true;
        const double dt = static_cast<double>(dt_buf[k]);
        if (dt <= 0.0 || dt >= kDtSaneMax)
            continue;  // 비정상 간격은 창에 넣지 않는다(위 ③·kDtSaneMax)
        const double use  = std::min(dt, window_sec - w.covered_sec);
        const Eigen::Vector3d g(gyr_buf[k]);
        const double norm = (g - bg).norm();
        w.sum_norm_dt += norm * use;
        w.sum_raw_dt  += g.norm() * use;
        w.covered_sec += use;
        ++w.n;
        w.block_sum   += norm * use;
        w.block_cover += use;
        if (w.block_cover >= kBlockSec)
        {
            w.max_block   = std::max(w.max_block, w.block_sum / w.block_cover);
            w.block_sum   = 0.0;
            w.block_cover = 0.0;
        }
    }
    return w.covered_sec >= window_sec;
}

// busy = "창 안에서 몸체가 돌았다". busy_thresh ≤ 0이면 검사 안 함(A/B 대조군 관행).
//   창을 min_cover_sec만큼도 못 채웠으면 판단 불가 → false.
//   바이어스 차감 평균과 원시 평균이 **둘 다** 상한 이상이어야 한다(위 ④).
inline bool gyroBusy(const GyroWindow &w, double busy_thresh, double min_cover_sec)
{
    if (busy_thresh <= 0.0)
        return false;
    if (w.covered_sec < min_cover_sec)
        return false;
    return std::min(w.meanNorm(), w.meanRaw()) >= busy_thresh;
}

// 다리 경로의 운동 증거 — 창 안 어느 0.1 s 블록이라도 상한 이상이면 참(위 ⑤).
inline bool motionEvidence(const GyroWindow &w, double busy_thresh, double min_cover_sec)
{
    if (busy_thresh <= 0.0)
        return false;
    if (w.covered_sec < min_cover_sec)
        return false;
    return w.maxBlockNorm() >= busy_thresh;
}

// 이번 solve의 STILL-DRIFT 판정을 건너뛰는 사유. 하나라도 있으면 연속 카운터를 0으로 되돌린다.
//   자이로가 1순위인 이유: 다리 이벤트 검출기는 강제 종료 뒤 쿨다운 고착(SW1-1924)이 있어
//   다리 동작 전체를 항상 덮지 못한다. 자이로는 원인(다리·외란)과 무관하게 몸체 운동을 본다.
//   leg_gated_with_evidence = 다리 게이트 겹침 AND motionEvidence.
enum class SkipReason
{
    None = 0,
    Gyro = 1,
    Leg  = 2
};

inline SkipReason skipReason(bool gyro_busy, bool leg_gated_with_evidence)
{
    if (gyro_busy)
        return SkipReason::Gyro;
    if (leg_gated_with_evidence)
        return SkipReason::Leg;
    return SkipReason::None;
}

inline const char *skipReasonName(SkipReason r)
{
    switch (r)
    {
    case SkipReason::Gyro: return "gyro";
    case SkipReason::Leg:  return "leg";
    default:               return "none";
    }
}

// 연속 초과 카운터 전이(estimator.cpp의 규칙을 순수 함수로 — gtest 대상).
//   sustained: 휠 기준 지속 정지 + 비교 기준 pose 존재. skipped: 몸체 운동으로 이번 solve 판정 건너뜀.
//   over: 창 변위가 상한 초과. 건너뛴 solve는 변위가 커도 세지 않고 카운터를 0으로 되돌린다 —
//   운동 시작 조각(자이로 평균이 아직 희석)에서 센 1~2회가 운동을 가로질러 3연속이 되는 것을 막는다.
inline int nextConsec(int consec, bool sustained, bool skipped, bool over)
{
    if (!sustained || skipped)
        return 0;
    return over ? consec + 1 : 0;
}

inline bool shouldFire(int consec, int needed) { return consec >= needed; }
}  // namespace still_drift_guard
