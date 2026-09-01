// [SW1-1880] static init 자세 부여 회귀 테스트
//
// 결함(업스트림 상속): assignStaticInitPoses가 장부(all_image_frame) 전체를 인덱스 i++로
// 순회하며 창 배열 Rs/Ps(WINDOW_SIZE+1칸)를 부여 → INITIAL 중 SECOND_NEW 폐기 항목이
// 장부에 남아 장부>창이면 ①창 항목이 엉뚱한 슬롯 자세를 받고(인덱스 밀림) ②i가 창을
// 넘어 배열 밖 읽기(UB). 수사 정본: doc/ALL_IMAGE_FRAME_LEAK.md의 잠복 버그 절.
#include <gtest/gtest.h>

#include "../src/estimator/estimator.h"

namespace
{
// 슬롯별로 구별 가능한 자세: T=(j, 2j, 3j), R=z축 j*0.01rad 회전
Eigen::Matrix3d slotR(int j)
{
    return Eigen::AngleAxisd(0.01 * j, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
Eigen::Vector3d slotT(int j) { return {double(j), 2.0 * j, 3.0 * j}; }

void fillWindow(Estimator &est, double t0)
{
    map<int, Eigen::Matrix<double, 7, 1>> no_points;
    for (int j = 0; j <= WINDOW_SIZE; j++)
    {
        est.Headers[j] = t0 + 10.0 * j;  // 창 스탬프 간격 10s (잉여 삽입 여지)
        est.Rs[j]      = slotR(j);
        est.Ps[j]      = slotT(j);
        ImageFrame f(no_points, est.Headers[j]);
        f.pre_integration = nullptr;
        est.all_image_frame.insert(std::make_pair(est.Headers[j], f));
    }
    est.frame_count = WINDOW_SIZE;
}

void addExtraEntry(Estimator &est, double t)
{
    map<int, Eigen::Matrix<double, 7, 1>> no_points;
    ImageFrame f(no_points, t);
    f.pre_integration = nullptr;
    est.all_image_frame.insert(std::make_pair(t, f));
}
}  // namespace

// 정합 상태(장부 == 창): 각 창 항목이 자기 슬롯 자세를 정확히 받는다 (기존 동작 보존)
TEST(StaticInitPoseAssign, ExactWindowGetsOwnSlotPose)
{
    Estimator est;
    fillWindow(est, 100.0);
    est.assignStaticInitPoses();
    for (int j = 0; j <= WINDOW_SIZE; j++)
    {
        auto &f = est.all_image_frame.at(est.Headers[j]);
        EXPECT_TRUE(f.T.isApprox(slotT(j))) << "슬롯 " << j;
        EXPECT_TRUE(f.R.isApprox(slotR(j))) << "슬롯 " << j;
    }
}

// 결함 재현 조건(장부 > 창): 창 항목은 여전히 자기 슬롯 자세를 받아야 한다.
// 수정 전(인덱스 루프)에서는 잉여 항목이 끼어들며 인덱스가 밀려 실패한다 — 결함의 결정론적 실증.
TEST(StaticInitPoseAssign, WindowEntriesCorrectEvenWithExtras)
{
    Estimator est;
    fillWindow(est, 100.0);
    // INITIAL 중 SECOND_NEW 폐기분 모사: 창 스탬프 사이사이에 잉여 3개
    addExtraEntry(est, 105.0);
    addExtraEntry(est, 115.0);
    addExtraEntry(est, 125.0);

    est.assignStaticInitPoses();

    for (int j = 0; j <= WINDOW_SIZE; j++)
    {
        auto &f = est.all_image_frame.at(est.Headers[j]);
        EXPECT_TRUE(f.T.isApprox(slotT(j)))
            << "슬롯 " << j << " 창 항목이 엉뚱한 자세를 받음 (인덱스 밀림)";
    }
}

// 잉여 항목도 쓰레기값이 아닌 '최근접 창 슬롯 자세'를 받아야 한다
// (solveGyroscopeBias가 장부 전 항목을 소비하므로 — 층 2 오염 방지)
TEST(StaticInitPoseAssign, ExtraEntriesGetNearestWindowPose)
{
    Estimator est;
    fillWindow(est, 100.0);
    addExtraEntry(est, 104.9);  // 슬롯 0(100)과 1(110) 사이, 0 쪽에 근접
    addExtraEntry(est, 126.0);  // 슬롯 2(120)와 3(130) 사이, 3 쪽에 근접

    est.assignStaticInitPoses();

    EXPECT_TRUE(est.all_image_frame.at(104.9).T.isApprox(slotT(0)));
    EXPECT_TRUE(est.all_image_frame.at(126.0).T.isApprox(slotT(3)));
}

// 잉여가 많아도(장부가 창의 2배) 전 항목 자세가 유한값 — 배열 밖 쓰레기 유입 부재
TEST(StaticInitPoseAssign, ManyExtrasAllFinite)
{
    Estimator est;
    fillWindow(est, 100.0);
    // 9.1 간격: 창 스탬프(10 배수)와 절대 안 겹치게 — 12개 전부 실삽입 보장 (critic minor 반영)
    for (int k = 0; k < 12; k++) addExtraEntry(est, 101.3 + 9.1 * k);

    est.assignStaticInitPoses();

    for (auto &it : est.all_image_frame)
    {
        EXPECT_TRUE(it.second.T.allFinite()) << "t=" << it.first;
        EXPECT_TRUE(it.second.R.allFinite()) << "t=" << it.first;
    }
}
