#pragma once
// [SW1-1872] HF(고주기) IMU 예측 경로 — 순수 수학부.
//
// 배경: upstream 계승 결함 2건(doc/HF_TF_PREDICT_DEFECTS.md §3·§4) 수정의 계약을
//   단위검증 가능한 헤더로 고정한다 (reboot_seed.h 선례).
//   결함① = 재기저 재적분 루프가 acc/gyr을 첫 샘플로 고정(zero-order-hold)
//   결함② = 예측이 창 처리용 acc_0/gyr_0를 자물쇠 없이 공유(이전 샘플 오염)
//   기준 구현 = VINS-Fusion fastPredictIMU (매 샘플 사용 + 예측 전용 이전 샘플 멤버).

#include <Eigen/Dense>

#include "utility.h"  // Utility::deltaQ (헤더-only)

namespace hf_predict
{

// 중점(midpoint) 적분 한 스텝 — VINS-Fusion fastPredictIMU와 동일 수식.
// prev_acc/prev_gyr = "직전 IMU 샘플". 호출자는 예측 경로 전용 멤버를 넘겨야 하며
// 창 경로의 acc_0/gyr_0를 넘기면 결함②가 재발한다.
inline void midpointStep(Eigen::Vector3d &P, Eigen::Quaterniond &Q, Eigen::Vector3d &V,
                         double dt, const Eigen::Vector3d &prev_acc,
                         const Eigen::Vector3d &prev_gyr, const Eigen::Vector3d &acc,
                         const Eigen::Vector3d &gyr, const Eigen::Vector3d &Ba,
                         const Eigen::Vector3d &Bg, const Eigen::Vector3d &g)
{
    const Eigen::Vector3d un_acc_0 = Q * (prev_acc - Ba) - g;
    const Eigen::Vector3d un_gyr   = 0.5 * (prev_gyr + gyr) - Bg;
    Q                              = Q * Utility::deltaQ(un_gyr * dt);
    const Eigen::Vector3d un_acc_1 = Q * (acc - Ba) - g;
    const Eigen::Vector3d un_acc   = 0.5 * (un_acc_0 + un_acc_1);
    P                              = P + dt * V + 0.5 * dt * dt * un_acc;
    V                              = V + dt * un_acc;
}

// 재적분 루프의 표본 선택 계약: 시각과 값을 "같은 큐 원소"에서 꺼내 step에 넘긴다.
// (결함①의 정확한 기전 — 값만 원본 큐 front() 고정 — 을 타입 수준에서 재발 방지.)
// Queue = queue<pair<double, pair<Vector3d, Vector3d>>> (복사본을 받아 소모).
template <typename Queue, typename StepFn>
inline void forEachSample(Queue q, StepFn step)
{
    for (; !q.empty(); q.pop())
        step(q.front().first, q.front().second.first, q.front().second.second);
}

}  // namespace hf_predict
