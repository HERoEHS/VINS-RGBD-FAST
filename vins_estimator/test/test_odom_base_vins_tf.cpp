// odom→base_vins 합성(composeBaseFootprint) 검증
//   기준: registerPub()의 정적 체인 body→vins/base_link(RIO,TIO)→vins/base_footprint(-z)
//   와 같은 결과여야 한다. Eigen::Isometry3d 체인 합성을 정답으로 삼아 비교한다.
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "../src/utility/odom_base_vins_tf.h"

namespace
{

// Isometry 체인으로 계산한 정답: T_pose ∘ T_body_bl(R_io,t_io) ∘ T_bl_bfp(I, [0,0,-z])
void reference(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q,
               const Eigen::Matrix3d &R_io, const Eigen::Vector3d &t_io, double z,
               Eigen::Vector3d &P_ref, Eigen::Quaterniond &Q_ref)
{
    Eigen::Isometry3d T_pose = Eigen::Isometry3d::Identity();
    T_pose.linear() = Q.toRotationMatrix();
    T_pose.translation() = P;

    Eigen::Isometry3d T_body_bl = Eigen::Isometry3d::Identity();
    T_body_bl.linear() = R_io;
    T_body_bl.translation() = t_io;

    Eigen::Isometry3d T_bl_bfp = Eigen::Isometry3d::Identity();
    T_bl_bfp.translation() = Eigen::Vector3d(0.0, 0.0, -z);

    const Eigen::Isometry3d T = T_pose * T_body_bl * T_bl_bfp;
    P_ref = T.translation();
    Q_ref = Eigen::Quaterniond(T.linear());
}

void expectSame(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q,
                const Eigen::Matrix3d &R_io, const Eigen::Vector3d &t_io, double z)
{
    Eigen::Vector3d P_out, P_ref;
    Eigen::Quaterniond Q_out, Q_ref;
    vins_tf::composeBaseFootprint(P, Q, R_io, t_io, z, P_out, Q_out);
    reference(P, Q, R_io, t_io, z, P_ref, Q_ref);

    EXPECT_NEAR((P_out - P_ref).norm(), 0.0, 1e-12);
    // 쿼터니언은 q와 -q가 같은 회전 — 각도 차로 비교
    EXPECT_NEAR(Q_out.angularDistance(Q_ref), 0.0, 1e-12);
}

}  // namespace

// RIO=I(현재 vio_edie.yaml 상태): base_vins = body + Q·(TIO + [0,0,-z])
TEST(OdomBaseVinsTf, IdentityExtrinsicMatchesChain)
{
    const Eigen::Vector3d TIO_edie(0.1056, 0.0, -0.0941);  // parameters.h 주석의 실측값
    expectSame(Eigen::Vector3d(1.0, 2.0, 0.3),
               Eigen::Quaterniond::Identity(),
               Eigen::Matrix3d::Identity(), TIO_edie,
               vins_tf::kBaseZWheelRadius);
}

// body가 yaw 90° 회전한 상태 — 레버암(TIO)이 회전을 따라가는지
TEST(OdomBaseVinsTf, RotatedBodyLeversArm)
{
    const Eigen::Quaterniond yaw90(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
    expectSame(Eigen::Vector3d(0.5, -0.2, 0.1), yaw90,
               Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.1056, 0.0, -0.0941),
               vins_tf::kBaseZWheelRadius);
}

// RIO가 비단위인 경우(yaml 변경 대비) + 임의 자세 — 일반 케이스
TEST(OdomBaseVinsTf, NonIdentityExtrinsicGeneralPose)
{
    const Eigen::Matrix3d R_io(
        Eigen::AngleAxisd(0.03, Eigen::Vector3d(0.2, -0.5, 0.84).normalized()));
    const Eigen::Quaterniond Q(
        Eigen::AngleAxisd(1.1, Eigen::Vector3d(-0.3, 0.9, 0.31).normalized()));
    expectSame(Eigen::Vector3d(-3.2, 7.7, 0.05), Q.normalized(),
               R_io, Eigen::Vector3d(0.11, -0.005, -0.09),
               vins_tf::kBaseZWheelRadius);
}

// 출력 쿼터니언 정규화 보장 (TF 소비자는 비정규 쿼터니언에 취약)
TEST(OdomBaseVinsTf, OutputQuaternionIsNormalized)
{
    Eigen::Vector3d P_out;
    Eigen::Quaterniond Q_out;
    // 의도적으로 살짝 비정규인 입력
    Eigen::Quaterniond Q(0.71, 0.0, 0.0, 0.71);
    vins_tf::composeBaseFootprint(Eigen::Vector3d::Zero(), Q,
                                  Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
                                  vins_tf::kBaseZWheelRadius, P_out, Q_out);
    EXPECT_NEAR(Q_out.norm(), 1.0, 1e-12);
}

// 이 저장소의 ament_add_gtest는 gtest_main을 자동 링크하지 않아 각 테스트가 main()을 직접 둔다
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
