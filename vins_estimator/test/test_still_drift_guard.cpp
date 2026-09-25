// [SW1-1922] STILL-DRIFT 가드 "몸체 운동" 판정(still_drift_guard.h) + 다리 게이트 구간 연장 검증
//   ① 창 채우기: 조각 경계를 넘어 뒤에서부터, 경계 샘플은 부분 가중, 비정상 dt 는 제외
//   ② 왕복 운동은 벡터 평균이 상쇄되지만 노름 평균은 잡는다 (v15 217 s 기울었다 복귀 회귀)
//   ③ 정지 노이즈는 busy 아님(영평균 가우시안, 실측 바닥 p99 ≈ 0.02), 임계 ≤0 은 검사 안 함, 창 미달은 판단 불가
//   ④ 바이어스 자기참조 방어: Bg 가 오염되면 원시 평균이 작아 busy 가 무효
//   ⑤ 217 s 유사 합성 시퀀스에서 창별 판정 + 운동 시작·끝 조각의 블록 최대(다리 경로 운동 증거)
//   ⑥ 건너뛰기 사유 우선순위(자이로 > 다리) + 카운터 전이 규칙(계획서 1(d))
//   ⑦ LegEventDetector: 강제 종료 구간만 연장 질의에 응답, 정착 종료 구간은 비연장
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <random>
#include <vector>

#include "../src/utility/leg_event_detector.h"
#include "../src/utility/still_drift_guard.h"

namespace sdg = still_drift_guard;

namespace
{
using V3 = Eigen::Vector3d;
constexpr double kThresh = 0.05;   // yaml still_drift_gyro_busy_rad_s 기본값
constexpr double kMinCov = 0.25;   // 창 0.5 s × kMinCoverFrac

struct Piece
{
    std::vector<double> dt;
    std::vector<V3>     gyr;
};

Piece constPiece(int n, double dt, const V3 &g)
{
    Piece p;
    for (int i = 0; i < n; ++i)
    {
        p.dt.push_back(dt);
        p.gyr.push_back(g);
    }
    return p;
}
}  // namespace

// ① 창 채우기 — 두 조각을 최신→과거로 넘기면 경계를 넘어 정확히 window_sec 만큼 채운다
TEST(StillDriftGuard, AccumulateAcrossPieces)
{
    // 과거 조각: 0.3 s @100 Hz, gyro 0.4 / 최신 조각: 0.2 s @100 Hz, gyro 0.1
    Piece older = constPiece(30, 0.01, V3(0.4, 0, 0));
    Piece newer = constPiece(20, 0.01, V3(0.1, 0, 0));
    sdg::GyroWindow w;
    EXPECT_FALSE(sdg::accumulateTail(newer.dt, newer.gyr, V3::Zero(), 0.5, w));  // 0.2 s 만 찼다
    EXPECT_NEAR(w.covered_sec, 0.2, 1e-9);
    EXPECT_TRUE(sdg::accumulateTail(older.dt, older.gyr, V3::Zero(), 0.5, w));   // 나머지 0.3 s
    EXPECT_NEAR(w.covered_sec, 0.5, 1e-9);
    EXPECT_EQ(w.n, 50u);
    // 시간 가중 평균 = (0.1·0.2 + 0.4·0.3) / 0.5 = 0.28, 블록 최대 = 0.4
    EXPECT_NEAR(w.meanNorm(), 0.28, 1e-9);
    EXPECT_NEAR(w.maxBlockNorm(), 0.4, 1e-9);
}

// ① 경계 샘플 부분 가중 + 비정상 dt(IMU 공백) 제외
TEST(StillDriftGuard, PartialLastSampleAndDtSanity)
{
    // 과거 0.05 s 샘플(gyro 1.0) ×10, 최신 0.02 s 샘플(gyro 0.0) ×5 → 창 0.5: 0.1 s 는 0, 0.4 s 는 1.0 → 0.8
    std::vector<double> dt(10, 0.05);
    std::vector<V3>     g(10, V3(1.0, 0, 0));
    for (int i = 0; i < 5; ++i) { dt.push_back(0.02); g.push_back(V3::Zero()); }
    sdg::GyroWindow w;
    EXPECT_TRUE(sdg::accumulateTail(dt, g, V3::Zero(), 0.5, w));
    EXPECT_NEAR(w.covered_sec, 0.5, 1e-12);
    EXPECT_NEAR(w.meanNorm(), 0.8, 1e-12);

    // IMU 공백 뒤의 큰 dt(1.0 s) 한 표본과 kDtSaneMax(0.1) 짜리 표본은 창에 들어가지 않는다
    //   → 정상 표본(0.05 ×4)만으로 0.2 s. 실제 하한 kMinCoverFrac × 창(0.5 × 0.5 = 0.25)에 못 미쳐 판단 불가.
    //   (0.05 ×6 = 0.3 s 였다면 하한을 넘겨 busy 가 된다 — 하한이 실제 값으로 검증되도록 5 표본 경계를 잡는다)
    const double min_cover = sdg::kMinCoverFrac * 0.5;
    std::vector<double> dt2{1.0, 0.1, 0.05, 0.05, 0.05, 0.05};
    std::vector<V3>     g2(6, V3(1.0, 0, 0));
    sdg::GyroWindow w2;
    EXPECT_FALSE(sdg::accumulateTail(dt2, g2, V3::Zero(), 0.5, w2));
    EXPECT_NEAR(w2.covered_sec, 0.2, 1e-12);
    EXPECT_EQ(w2.n, 4u);
    EXPECT_FALSE(sdg::gyroBusy(w2, kThresh, min_cover));   // 0.2 < 0.25 → 판단 불가
    // 정상 표본이 0.3 s(0.05 ×6)면 하한 0.25 를 넘겨 판단 가능 → busy
    //   (경계값 0.25 정확히는 부동소수 누적 순서에 따라 뒤집힐 수 있어 테스트에 쓰지 않는다)
    std::vector<double> dt3{1.0, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
    std::vector<V3>     g3(8, V3(1.0, 0, 0));
    sdg::GyroWindow w3;
    EXPECT_FALSE(sdg::accumulateTail(dt3, g3, V3::Zero(), 0.5, w3));
    EXPECT_NEAR(w3.covered_sec, 0.35, 1e-12);
    EXPECT_TRUE(sdg::gyroBusy(w3, kThresh, min_cover));
}

// ② 왕복 운동: +0.2 로 0.25 s, −0.2 로 0.25 s → 벡터 평균 0, 노름 평균 0.2 → busy
TEST(StillDriftGuard, BackAndForthIsBusy)
{
    std::vector<double> dt(50, 0.01);
    std::vector<V3>     g;
    V3 vsum = V3::Zero();
    for (int i = 0; i < 50; ++i)
    {
        g.emplace_back(i < 25 ? 0.2 : -0.2, 0, 0);
        vsum += g.back();
    }
    EXPECT_NEAR(vsum.norm() / 50.0, 0.0, 1e-12);  // 벡터 평균은 상쇄된다
    sdg::GyroWindow w;
    EXPECT_TRUE(sdg::accumulateTail(dt, g, V3::Zero(), 0.5, w));
    EXPECT_NEAR(w.meanNorm(), 0.2, 1e-9);
    EXPECT_TRUE(sdg::gyroBusy(w, kThresh, kMinCov));
}

// ③ 정지 노이즈(영평균 가우시안 σ=0.012, 고정 시드)는 노름 평균이 양의 편향을 갖지만 상한의 절반 이하
//    / 바이어스 차감 / 임계 ≤0 → 검사 안 함 / 창 미달 → 판단 불가
TEST(StillDriftGuard, QuietNoiseBiasDisableAndCover)
{
    std::mt19937 rng(1922);
    std::normal_distribution<double> nd(0.0, 0.012);  // 실측 정지 바닥: 노름 평균 p99 0.014~0.023(v14/v15/v16)
    const V3 bias(0.003, -0.002, 0.001);              // 정상 Bg 규모(수 mrad/s)
    std::vector<double> dt(200, 0.0025);
    std::vector<V3>     g;
    for (int i = 0; i < 200; ++i) g.push_back(bias + V3(nd(rng), nd(rng), nd(rng)));
    sdg::GyroWindow w;
    EXPECT_TRUE(sdg::accumulateTail(dt, g, bias, 0.5, w));
    EXPECT_GT(w.meanNorm(), 0.012);        // 3축 노름 평균은 σ 보다 크다(양의 편향) — 그래서 벡터 평균보다 바닥이 높다
    EXPECT_LT(w.meanNorm(), 0.5 * kThresh);  // 그래도 상한의 절반 이하(실측 p99 0.023 과 같은 규모)
    EXPECT_FALSE(sdg::gyroBusy(w, kThresh, kMinCov));
    EXPECT_FALSE(sdg::motionEvidence(w, kThresh, kMinCov));  // 블록 최대도 상한 미만

    // 임계 ≤0 → 아무리 커도 busy 아님(대조군)
    Piece big = constPiece(50, 0.01, V3(1.0, 0, 0));
    sdg::GyroWindow w3;
    sdg::accumulateTail(big.dt, big.gyr, V3::Zero(), 0.5, w3);
    EXPECT_FALSE(sdg::gyroBusy(w3, 0.0, kMinCov));
    EXPECT_FALSE(sdg::gyroBusy(w3, -1.0, kMinCov));
    EXPECT_TRUE(sdg::gyroBusy(w3, kThresh, kMinCov));

    // 창 미달(0.1 s 만 있음) → 판단 불가 = false
    Piece short_big = constPiece(10, 0.01, V3(1.0, 0, 0));
    sdg::GyroWindow w4;
    EXPECT_FALSE(sdg::accumulateTail(short_big.dt, short_big.gyr, V3::Zero(), 0.5, w4));
    EXPECT_FALSE(sdg::gyroBusy(w4, kThresh, kMinCov));
}

// ④ 바이어스 자기참조 방어 — Bg 추정이 0.3 rad/s 로 오염되면 차감 평균은 0.3 이지만 원시 평균이 작아 busy 무효
TEST(StillDriftGuard, CorruptedBiasDoesNotDisableGuard)
{
    Piece quiet = constPiece(50, 0.01, V3(0.005, 0, 0));
    sdg::GyroWindow w;
    sdg::accumulateTail(quiet.dt, quiet.gyr, V3(0.3, 0, 0), 0.5, w);
    EXPECT_NEAR(w.meanNorm(), 0.295, 1e-9);
    EXPECT_NEAR(w.meanRaw(), 0.005, 1e-9);
    EXPECT_FALSE(sdg::gyroBusy(w, kThresh, kMinCov));
    // 진짜 운동(0.4)이면 오염된 Bg 로도 둘 다 상한 이상 → busy
    Piece moving = constPiece(50, 0.01, V3(0.4, 0, 0));
    sdg::GyroWindow w2;
    sdg::accumulateTail(moving.dt, moving.gyr, V3(0.3, 0, 0), 0.5, w2);
    EXPECT_TRUE(sdg::gyroBusy(w2, kThresh, kMinCov));
}

// ⑤ 217 s 유사 합성: 정지 1 s → 왕복 1.9 s(진폭 0.3 rad/s 사인 한 주기) → 정지 1 s
//   실측(v15): 몸체 회전 217.77~219.64 s, 가드 창 W1~W3 노름 평균 0.40~0.45, 정지 시 p99 ≈ 0.02.
TEST(StillDriftGuard, SyntheticLegEventWindows)
{
    const double t_on = 1.0, t_off = 2.9, hz = 400.0;
    std::vector<double> dt;
    std::vector<V3>     g;
    std::vector<double> ts;
    for (double t = 0.0; t < 3.9; t += 1.0 / hz)
    {
        double gx = 0.008;
        if (t >= t_on && t < t_off)
            gx = 0.3 * std::sin(2.0 * M_PI * (t - t_on) / (t_off - t_on));  // 기울었다(+) 복귀(−)
        dt.push_back(1.0 / hz);
        g.emplace_back(gx, 0.0, 0.0);
        ts.push_back(t + 1.0 / hz);
    }
    auto winAt = [&](double t_now, double len) {
        std::vector<double> dts;
        std::vector<V3>     gs;
        for (size_t i = 0; i < ts.size() && ts[i] <= t_now + 1e-9; ++i)
        {
            dts.push_back(dt[i]);
            gs.push_back(g[i]);
        }
        sdg::GyroWindow w;
        sdg::accumulateTail(dts, gs, V3::Zero(), len, w);
        return w;
    };
    // 정지 중 창: busy 아님, 운동 증거도 없음
    EXPECT_FALSE(sdg::gyroBusy(winAt(0.9, 0.5), kThresh, kMinCov));
    EXPECT_FALSE(sdg::motionEvidence(winAt(0.9, 0.5), kThresh, kMinCov));
    // 운동 중 창 여러 개(W1~W3 유사: 상승 끝·전환·하강): 모두 busy — 비교 창이 0.6 s 로 길어도 같다
    for (double t : {1.4, 1.9, 2.4, 2.9})
    {
        EXPECT_TRUE(sdg::gyroBusy(winAt(t, 0.5), kThresh, kMinCov)) << "t=" << t;
        EXPECT_TRUE(sdg::gyroBusy(winAt(t, 0.6), kThresh, kMinCov)) << "t=" << t;
    }
    // 방향 전환점을 가운데 둔 창(1.7~2.2 s): 벡터 평균은 0 근처지만 노름 평균은 busy
    EXPECT_TRUE(sdg::gyroBusy(winAt(2.2, 0.5), kThresh, kMinCov));
    // 운동 시작 조각: 운동이 창 끝 0.1 s 에만 있으면 평균은 희석돼 busy 가 아니지만(0.3·sin 의 첫 0.1 s
    //   평균 ≈ 0.05·… ) 블록 최대는 상한 이상 → 다리 게이트와 함께 다리 경로가 잡는다
    {
        sdg::GyroWindow w = winAt(1.15, 0.5);   // 운동 0.15 s 포함
        EXPECT_FALSE(sdg::gyroBusy(w, kThresh, kMinCov));
        EXPECT_TRUE(sdg::motionEvidence(w, kThresh, kMinCov));
    }
    // 운동 종료 0.5 s 뒤부터 다시 조용(증거도 없음)
    EXPECT_FALSE(sdg::gyroBusy(winAt(3.45, 0.5), kThresh, kMinCov));
    EXPECT_FALSE(sdg::motionEvidence(winAt(3.45, 0.5), kThresh, kMinCov));
}

// ⑥ 사유 우선순위 — 자이로가 다리보다 앞선다(쿨다운 고착 SW1-1924 와 무관하게 몸체 운동을 본다)
TEST(StillDriftGuard, SkipReasonPriority)
{
    EXPECT_EQ(sdg::skipReason(false, false), sdg::SkipReason::None);
    EXPECT_EQ(sdg::skipReason(true, false), sdg::SkipReason::Gyro);
    EXPECT_EQ(sdg::skipReason(false, true), sdg::SkipReason::Leg);
    EXPECT_EQ(sdg::skipReason(true, true), sdg::SkipReason::Gyro);
    EXPECT_STREQ(sdg::skipReasonName(sdg::SkipReason::Leg), "leg");
}

// ⑥ 카운터 전이 규칙(계획서 1(d)) — estimator.cpp 가 쓰는 순수 함수
TEST(StillDriftGuard, ConsecCounterRules)
{
    // (a) 건너뜀 없이 3연속 초과 → 3 에서 발동
    int c = 0;
    c = sdg::nextConsec(c, true, false, true);
    c = sdg::nextConsec(c, true, false, true);
    EXPECT_FALSE(sdg::shouldFire(c, 3));
    c = sdg::nextConsec(c, true, false, true);
    EXPECT_EQ(c, 3);
    EXPECT_TRUE(sdg::shouldFire(c, 3));
    // (b) 2연속 뒤 건너뜀 → 0
    c = 2;
    EXPECT_EQ(sdg::nextConsec(c, true, true, true), 0);
    // (c) 건너뛴 solve 는 변위가 커도 증가하지 않는다(위 (b) 와 같은 호출), 이어지는 정상 solve 는 1 부터
    EXPECT_EQ(sdg::nextConsec(0, true, false, true), 1);
    // (d) 지속 정지가 아니면 0 (초과 여부 무관)
    EXPECT_EQ(sdg::nextConsec(2, false, false, true), 0);
    // (e) 초과가 아니면 0
    EXPECT_EQ(sdg::nextConsec(2, true, false, false), 0);
}

// ⑦ 다리 게이트 구간 연장 — 강제 종료 구간만 연장, 정착 종료 구간은 그대로
TEST(LegEventDetectorStillDrift, ForcedCloseExtendedOnlyWhenAsked)
{
    LegEventDetector d;
    LegEventDetector::Params p;
    p.max_duration = 2.0;
    p.post_margin  = 0.5;
    p.pre_margin   = 0.3;
    d.setParams(p);
    // 정지 → t=1.0 부터 3.0 s 동안 계속 움직임(램프 0.3 rad/s) → 변위 0.03 초과 시점 t≈1.11 에 열림
    //   → 2.0 s 상한 강제 종료: 구간 ≈ [0.81, 3.11] (다리는 4.0 까지 계속 움직이지만 쿨다운이라 재개 없음)
    for (double t = 0.0; t < 1.0; t += 0.01) d.onMeasurement(t, 0.0, 0.0);
    for (double t = 1.0; t < 4.0; t += 0.01) d.onMeasurement(t, (t - 1.0) * 0.3, 0.0);
    EXPECT_TRUE(d.everForceClosed());
    // 기본 질의: 강제 종료 구간 끝(≈3.11) 뒤 3.2~3.5 는 게이트 아님
    EXPECT_FALSE(d.overlaps(3.2, 3.5));
    // 연장 질의(0.5 s): 끝 ≈3.61 → 3.2~3.5 는 겹침, 3.7~3.9 는 아님
    EXPECT_TRUE(d.overlaps(3.2, 3.5, 0.5));
    EXPECT_FALSE(d.overlaps(3.7, 3.9, 0.5));
    // 구간 질의 vs 순간 질의: 창 [2.9, 3.15] 는 끝점(3.15)만 보면 밖이지만 구간이면 겹침
    EXPECT_FALSE(d.overlaps(3.15, 3.15));
    EXPECT_TRUE(d.overlaps(2.9, 3.15));

    // 정착 종료 구간은 연장하지 않는다: 새 검출기, 0.3 s 스윙 후 정착 → [pre, last_move+post]
    LegEventDetector e;
    e.setParams(p);
    for (double t = 0.0; t < 1.0; t += 0.01) e.onMeasurement(t, 0.0, 0.0);
    for (double t = 1.0; t < 1.3; t += 0.01) e.onMeasurement(t, (t - 1.0) * 1.0, 0.0);
    for (double t = 1.3; t < 3.0; t += 0.01) e.onMeasurement(t, 0.3, 0.0);
    EXPECT_FALSE(e.everForceClosed());
    // 종료 ≈ 1.29 + 0.5 = 1.79 → 1.85~1.9 는 기본/연장 모두 밖
    EXPECT_FALSE(e.overlaps(1.85, 1.9));
    EXPECT_FALSE(e.overlaps(1.85, 1.9, 0.5));
}
