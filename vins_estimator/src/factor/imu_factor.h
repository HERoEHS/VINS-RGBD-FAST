#pragma once

#include "ros_compat.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../utility/parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
    IMUFactor() = delete;

    IMUFactor(IntegrationBase *_pre_integration) : pre_integration(_pre_integration) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const
    {
        Eigen::Vector3d    Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                              parameters[0][5]);

        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

        Eigen::Vector3d    Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4],
                              parameters[2][5]);

        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

        // Eigen::Matrix<double, 15, 15> Fd;
        // Eigen::Matrix<double, 15, 12> Gd;

        // Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
        // Eigen::Quaterniond pQj = Qi * delta_q;
        // Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
        // Eigen::Vector3d pBaj = Bai;
        // Eigen::Vector3d pBgj = Bgi;

        // Vi + Qi * delta_v - g * sum_dt = Vj;
        // Qi * delta_q = Qj;

        // delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
        // delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
        // delta_q = Qi.inverse() * Qj;

#if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);

        // [SW1-1837] covariance가 PD를 잃거나(lam_min≤0) 장기-dt로 ill-conditioned가 되면 LLT(cov⁻¹)가
        // garbage sqrt_info를 만들어 P/V가 1e26까지 폭발(→5m 점프→reboot). 대칭화 + "상대" floor로
        // 조건수를 ~1e7로 묶는다. eps=max(1e-12, 1e-7·max|diag|) — 단기-dt엔 절대 floor, 장기-dt엔
        // 스케일 비례 floor (절대 1e-12만으론 장기-dt에서 siV가 1e11로 누수됐던 문제 해결).
#ifdef VIO_NUMERIC_FIX
        const double cov_scale = pre_integration->covariance.diagonal().cwiseAbs().maxCoeff();
        const double pd_eps    = (1e-7 * cov_scale > 1e-12) ? 1e-7 * cov_scale : 1e-12;
        Eigen::Matrix<double, 15, 15> cov_pd =
            0.5 * (pre_integration->covariance + pre_integration->covariance.transpose());
        cov_pd.diagonal().array() += pd_eps;
        Eigen::Matrix<double, 15, 15> sqrt_info =
            Eigen::LLT<Eigen::Matrix<double, 15, 15>>(cov_pd.inverse()).matrixL().transpose();
#else
        Eigen::Matrix<double, 15, 15> sqrt_info =
            Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse())
                .matrixL()
                .transpose();
#endif
        // sqrt_info.setIdentity();
        residual = sqrt_info * residual;

#if 0  // [SW1-1837 진단] 조건수 계측 (필요 시 1로). 관찰자 효과 있어 측정 시 OFF 권장
        // [SW1-1837 진단] IMU factor 조건수 계측 — sqrt_info=chol(cov⁻¹)를 어느 상태 블록이 키우는지 분리.
        //  P/R/V가 크면 진짜 병리(궤적 폭주 유발), Ba/Bg만 크면 bias 공분산이 원래 작아 생기는 정상 artifact.
        //  cost·로그 절약을 위해 경고(>1e8) 근처(si_max>1e6) 케이스만 고유값 분해+출력.
        {
            double si_max = sqrt_info.cwiseAbs().maxCoeff();
            if (si_max > 1e6)
            {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(
                    pre_integration->covariance, Eigen::EigenvaluesOnly);
                double lam_min = es.eigenvalues().minCoeff();
                double lam_max = es.eigenvalues().maxCoeff();
                ROS_WARN(
                    "[IMU-COND] dt=%.3f cond=%.2e lam_min=%.2e | siP=%.2e siR=%.2e siV=%.2e siBa=%.2e siBg=%.2e",
                    pre_integration->sum_dt, lam_max / lam_min, lam_min,
                    sqrt_info.block<3, 15>(O_P, 0).cwiseAbs().maxCoeff(),
                    sqrt_info.block<3, 15>(O_R, 0).cwiseAbs().maxCoeff(),
                    sqrt_info.block<3, 15>(O_V, 0).cwiseAbs().maxCoeff(),
                    sqrt_info.block<3, 15>(O_BA, 0).cwiseAbs().maxCoeff(),
                    sqrt_info.block<3, 15>(O_BG, 0).cwiseAbs().maxCoeff());
            }
        }
#endif

        if (jacobians)
        {
            double          sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);

            if (pre_integration->jacobian.maxCoeff() > 1e8 ||
                pre_integration->jacobian.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in IMU preintegration (preint jacobian)");
                // std::cout << pre_integration->jacobian << std::endl;
                ///                ROS_BREAK();
            }

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(
                    jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(
                    Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                Eigen::Quaterniond corrected_delta_q =
                    pre_integration->delta_q *
                    Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) =
                    -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q))
                         .bottomRightCorner<3, 3>();
#endif

                jacobian_pose_i.block<3, 3>(O_V, O_R) =
                    Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in IMU factor (pose_i jacobian)");
                    // std::cout << sqrt_info << std::endl;
                    // ROS_BREAK();
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(
                    jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) =
                    -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                // Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q *
                // Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                // jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse()
                // * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
                    -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q)
                         .bottomRightCorner<3, 3>() *
                    dq_dbg;
#endif

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

                // ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
                // ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(
                    jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                Eigen::Quaterniond corrected_delta_q =
                    pre_integration->delta_q *
                    Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) =
                    Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj)
                        .bottomRightCorner<3, 3>();
#endif

                jacobian_pose_j = sqrt_info * jacobian_pose_j;

                // ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                // ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(
                    jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

                // ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
                // ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    // bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1>
    // &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    // void checkCorrection();
    // void checkTransition();
    // void checkJacobian(double **parameters);
    IntegrationBase *pre_integration;
};
