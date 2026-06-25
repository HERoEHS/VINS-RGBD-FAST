#pragma once

#include "utility.h"
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

// [SW1-1837] VIO 수치 안정성 수정 토글(clean A/B). 정의 시: IMU sqrt_info 상대 클램프 +
// 장기 preintegration(재init 갭) IMU factor 가드. 주석 처리하면 원본 동작(no-fix).
// [SW1-1837] VIO 수치 안정성 수정 토글. 정의 시: IMU sqrt_info 상대 클램프(조건수 ~1e7 제한)로
// 장기-dt/PD-loss covariance에서 LLT(cov⁻¹)가 P/V 1e26으로 폭발하던 문제 해결. 주석 처리하면 원본 동작.
// (clean 2×2: 클램프 단독으로 198m→2.74m. 장기-dt 가드는 불필요·단독 유해로 판명되어 미채택.)
#define VIO_NUMERIC_FIX

const double FOCAL_LENGTH = 460.0;
const int    WINDOW_SIZE  = 10;
const int    NUM_OF_CAM   = 1;
const int    NUM_OF_F     = 1000;

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int    ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;
extern double VEL_N_wheel, GYR_N_wheel;   // 휠 preintegration 노이즈 (VIW-Fusion)

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d              G;

extern double      BIAS_ACC_THRESHOLD;
extern double      BIAS_GYR_THRESHOLD;
extern double      SOLVER_TIME;
extern int         NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMAGE_TOPIC;
extern std::string DEPTH_TOPIC;
extern std::string IMU_TOPIC;
extern double      TD;
extern double      TR;
extern int         ESTIMATE_TD;
extern int         ROLLING_SHUTTER;
extern double      ROW, COL;

extern int IMAGE_SIZE;

extern double          DEPTH_MIN_DIST;
extern double          DEPTH_MAX_DIST;
extern unsigned short  DEPTH_MIN_DIST_MM;
extern unsigned short  DEPTH_MAX_DIST_MM;
extern int             MAX_CNT;
extern int             MAX_CNT_SET;
extern int             MIN_DIST;
extern int             FREQ;
extern double          F_THRESHOLD;
extern int             SHOW_TRACK;
extern int             EQUALIZE;
extern int             FISHEYE;
extern std::string     FISHEYE_MASK;
extern std::string     CAM_NAMES;
extern int             STEREO_TRACK;
extern bool            PUB_THIS_FRAME;
extern Eigen::Matrix3d Ric;

extern std::vector<std::string> SEMANTIC_LABEL;
extern std::vector<std::string> STATIC_LABEL;
extern std::vector<std::string> DYNAMIC_LABEL;

extern int NUM_GRID_ROWS;
extern int NUM_GRID_COLS;

extern int FRONTEND_FREQ;

extern int USE_IMU;
extern int NUM_THREADS;

extern int STATIC_INIT;

extern int FIX_DEPTH;

// ===== Wheel odometry tight-coupling (VIW-Fusion 휠 factor 이식, SW1-1829) =====
extern int             USE_WHEEL;                 // 휠 factor 사용 여부 (0=미사용 → 기존 VINS 동작 유지)
extern std::string     WHEEL_TOPIC;               // 휠 오도메트리 토픽 (nav_msgs/Odometry)
extern Eigen::Matrix3d RIO;                        // 휠(odom) → body extrinsic 회전 (T_io)
extern Eigen::Vector3d TIO;                        // 휠(odom) → body extrinsic 병진
extern double          SX, SY, SW;                 // 휠 intrinsic 스케일 (vel x/y, yaw)
extern double          TD_WHEEL;                   // 휠 시간오프셋 초기값
extern int             ESTIMATE_EXTRINSIC_WHEEL;   // Step2: 휠 extrinsic 온라인 추정 (0=고정)
extern int             ESTIMATE_INTRINSIC_WHEEL;   // Step2: 휠 intrinsic 온라인 추정 (0=고정)
extern int             ESTIMATE_TD_WHEEL;          // Step2: 휠 td 온라인 추정 (0=고정)

// ===== Zero-velocity Update (ZUPT, SW1-1837) =====
extern int    USE_ZUPT;          // 정지 시 속도 0 제약 사용 여부 (0=미사용)
extern double ZUPT_VEL_THRESH;   // 정지 판정: 평균 wheel 선속도 임계 [m/s]
extern double ZUPT_GYR_THRESH;   // 정지 판정: 평균 wheel 각속도 임계 [rad/s]
extern double ZUPT_WEIGHT;       // zero-velocity 잔차 가중치 (클수록 강하게 0)

// ===== Accelerometer-bias prior (SW1-1836) =====
extern int    USE_ACC_BIAS_PRIOR;   // acc bias를 target(0)으로 당기는 prior 사용 여부 (0=미사용)
extern double ACC_BIAS_PRIOR_W_XY;  // 수평(ax,ay) prior 가중치 (클수록 강하게 0)
extern double ACC_BIAS_PRIOR_W_Z;   // 수직(az) prior 가중치 (이미 정확 → 보통 0)

// ===== Wheel velocity outlier 게이팅 (SW1-1837) =====
// 비물리적 속도 글리치(예: 타임스탬프 dt→0로 106 m/s)를 적분 전 하드 드롭해 발산 방어.
extern int    USE_WHEEL_VEL_GATE;  // wheel 속도 outlier 게이팅 사용 여부 (0=미사용)
extern double WHEEL_VEL_MAX;       // 선속도 |v| 상한 [m/s], 초과 샘플 드롭
extern double WHEEL_GYR_MAX;       // 각속도 |w| 상한 [rad/s], 초과 샘플 드롭

void readParameters(rclcpp::Node* node);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE      = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P  = 0,
    O_R  = 3,
    O_V  = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
