// [08-31 정지 누수 수정] all_image_frame 가지치기 회귀 테스트
//
// 결함(업스트림 상속): 삽입은 매 프레임 무조건, 삭제는 MARGIN_OLD 분기에만 있어서
// 정지 중(키프레임 미생성 → SECOND_NEW만 반복)엔 맵이 무한 누적됐다(실기 분당 ~19MB).
// 수정: SECOND_NEW 분기에서 버려지는 차차신 프레임 항목을 NON_LINEAR일 때만 즉시 제거.
// 수사 정본: doc/ALL_IMAGE_FRAME_LEAK.md
#include <gtest/gtest.h>

#include "../src/estimator/estimator.h"

namespace
{
// 창(WINDOW_SIZE+1)을 스탬프 t0, t0+1, ... 로 채우고 맵에도 같은 항목을 넣는다.
// pre_integration은 테스트에선 nullptr(실경로에선 항상 할당됨 — delete nullptr 안전성도 겸사 검증).
void fillWindow(Estimator &est, double t0)
{
    map<int, Eigen::Matrix<double, 7, 1>> no_points;
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        est.Headers[i] = t0 + i;
        ImageFrame f(no_points, t0 + i);
        f.pre_integration = nullptr;
        est.all_image_frame.insert(std::make_pair(t0 + i, f));
    }
    est.frame_count = WINDOW_SIZE;
}
}  // namespace

// NON_LINEAR + SECOND_NEW: 버려지는 차차신 프레임 항목이 즉시 제거된다 (수정의 본체)
TEST(AllImageFramePrune, SecondNewErasesDroppedFrameInNonLinear)
{
    Estimator est;
    fillWindow(est, 100.0);
    est.solver_flag          = Estimator::NON_LINEAR;
    est.marginalization_flag = Estimator::MARGIN_SECOND_NEW;

    const double t_drop = est.Headers[WINDOW_SIZE - 1];  // 버려질 차차신 프레임
    est.slideWindow();

    EXPECT_EQ(est.all_image_frame.count(t_drop), 0u) << "차차신 항목이 제거돼야 한다";
    // 나머지(창의 다른 프레임들)는 보존 — MARGIN_OLD가 아니므로
    EXPECT_EQ(est.all_image_frame.size(), static_cast<size_t>(WINDOW_SIZE));
}

// 정지 시나리오 재현: SECOND_NEW 연속 N회에도 맵 크기가 유계 (누수 재발 방지의 핵심 단언)
TEST(AllImageFramePrune, RepeatedSecondNewStaysBounded)
{
    Estimator est;
    fillWindow(est, 100.0);
    est.solver_flag = Estimator::NON_LINEAR;

    map<int, Eigen::Matrix<double, 7, 1>> no_points;
    const size_t bound = WINDOW_SIZE + 2;  // 창 + 진행 중 신규 1
    for (int k = 0; k < 500; k++)
    {
        // 새 프레임 도착 모사: 최신 슬롯 갱신 + 맵 삽입 (processImage:446과 동형)
        const double t_new           = 200.0 + k;
        est.Headers[WINDOW_SIZE]     = t_new;
        ImageFrame f(no_points, t_new);
        f.pre_integration = nullptr;
        est.all_image_frame.insert(std::make_pair(t_new, f));

        est.marginalization_flag = Estimator::MARGIN_SECOND_NEW;
        est.slideWindow();

        ASSERT_LE(est.all_image_frame.size(), bound)
            << k << "회째에 유계 초과 — 정지 누수 재발";
    }
}

// INITIAL 단계는 업스트림 동작 보존: SECOND_NEW라도 아무것도 지우지 않는다
// (초기화는 비키프레임도 IMU 정렬에 쓰므로 게이트가 반드시 필요)
TEST(AllImageFramePrune, InitialPhaseKeepsAllEntries)
{
    Estimator est;
    fillWindow(est, 100.0);
    est.solver_flag          = Estimator::INITIAL;
    est.marginalization_flag = Estimator::MARGIN_SECOND_NEW;

    const size_t before = est.all_image_frame.size();
    est.slideWindow();

    EXPECT_EQ(est.all_image_frame.size(), before) << "INITIAL에선 삭제 금지";
}

// find 가드: 지울 항목이 맵에 없어도 죽지 않는다 (재초기화 직후 등 경계 방어)
TEST(AllImageFramePrune, MissingDropEntryIsHarmless)
{
    Estimator est;
    map<int, Eigen::Matrix<double, 7, 1>> no_points;
    for (int i = 0; i <= WINDOW_SIZE; i++) est.Headers[i] = 100.0 + i;
    est.frame_count = WINDOW_SIZE;  // 맵은 비워둠 — find가 end()를 돌려주는 상황
    est.solver_flag          = Estimator::NON_LINEAR;
    est.marginalization_flag = Estimator::MARGIN_SECOND_NEW;

    EXPECT_NO_THROW(est.slideWindow());
    EXPECT_TRUE(est.all_image_frame.empty());
}
