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

// ===== 정지 시 중력 재정렬 (SW1-1837, gravity realignment at standstill) =====
extern int    USE_GRAVITY_ALIGN;          // 정지 프레임 acc 평균으로 잔여 roll/pitch 사후 교정 (0=미사용)
extern double GRAVITY_ALIGN_WEIGHT;       // 잔차 가중치 = 1/σ_angle [1/rad]
extern double GRAVITY_ALIGN_VEL_THRESH;   // 정지 판정: 평균 wheel 선속도 임계 [m/s] (ZUPT와 별도 — use_zupt:0이어도 동작)
extern double GRAVITY_ALIGN_GYR_THRESH;   // 정지 판정: 평균 wheel 각속도 임계 [rad/s]
extern double GRAVITY_ALIGN_MIN_ANGLE;    // [rad] 모드 2/3: 발동 임계 — 이 미만 오차는 보정 안함
extern double GRAVITY_ALIGN_MAX_ANGLE;    // [rad] 모드 2/3: 1회 보정 상한 — marg prior 소각 근사 유효 범위
extern double GRAVITY_ALIGN_COOLDOWN;     // [s]   모드 2/3: 발동 간 최소 간격 — 연쇄 발동 churn 방지

// ===== Accelerometer-bias prior (SW1-1836) =====
extern int    USE_ACC_BIAS_PRIOR;   // acc bias를 target(0)으로 당기는 prior 사용 여부 (0=미사용)
extern double ACC_BIAS_PRIOR_W_XY;  // 수평(ax,ay) prior 가중치 (클수록 강하게 0)
extern double ACC_BIAS_PRIOR_W_Z;   // 수직(az) prior 가중치 (이미 정확 → 보통 0)

// ===== Vertical-velocity soft constraint (SW1-1837, planar-motion Level1) =====
extern int    USE_VERTICAL_VEL;     // 월드 수직속도 Vz를 0으로 상시 당기는 제약 사용 여부 (0=미사용)
extern double VERTICAL_VEL_WEIGHT;  // 잔차 가중치 (클수록 강하게 0). σ≈1/w [m/s]

// ===== Ground-plane constraint (SW1-1837, VIW-Fusion plane_factor 이식) =====
extern int    USE_PLANE;                        // 지면평면 제약 사용 여부 (0=미사용)
extern double PITCH_N_INV, ROLL_N_INV, ZPW_N_INV;  // 평면 잔차 sqrt_info (pitch/roll 자세, z 높이)

// ===== Body-frame NHC (SW1-1837, planar-motion Level2) =====
extern int    USE_BODY_NHC;               // 바디(바퀴) 프레임 vy·vz≈0 제약 사용 여부 (0=미사용)
extern double NHC_Y_WEIGHT, NHC_Z_WEIGHT; // 잔차 가중치 (횡/수직). σ≈1/w [m/s]

// ===== Wheel velocity outlier 게이팅 (SW1-1837) =====
// 비물리적 속도 글리치(예: 타임스탬프 dt→0로 106 m/s)를 적분 전 하드 드롭해 발산 방어.
extern int    USE_WHEEL_VEL_GATE;  // wheel 속도 outlier 게이팅 사용 여부 (0=미사용)
extern double WHEEL_VEL_MAX;       // 선속도 |v| 상한 [m/s], 초과 샘플 드롭
extern double WHEEL_GYR_MAX;       // 각속도 |w| 상한 [rad/s], 초과 샘플 드롭

// ===== [SW1-1837] 이벤트 게이팅 (Phase 1: 다리각) =====
// 다리 인출입 중엔 휠(v=w=0 주장)·plane(수평 강제)·vert(vz=0 강제)가 전부 틀린 제약이 됨
// (07-13 실측: 다리 스윙 순간 자세 오차 1.6~3.4° 주입) → 이벤트 시각 구간의 factor를 skip.
extern int         USE_EVENT_GATING;   // 마스터 토글 (0=기존 동작 유지)
extern int         GATE_LEG;           // 다리각 이벤트 게이팅 사용 여부
extern double      LEG_POS_MIN;        // [rad] 정착 기준점 대비 변위 시작 임계(엔코더 LSB 플리커 면역)
extern double      LEG_RATE_MIN;       // [rad/s] 진행 중 '아직 움직임' 판정 임계(종료·정착용)
extern double      LEG_CMD_POS_MIN;    // [rad] 명령-실측 차 시작 임계(선행 트리거)
extern double      LEG_PRE_MARGIN;     // [s] 시작 소급 마진
extern double      LEG_POST_MARGIN;    // [s] 종료 후 유지 마진
extern double      GATE_MAX_DURATION;  // [s] 연속 게이팅 상한(휠 앵커 상실 발산 방지)
extern std::string LEG_STATE_TOPIC;    // 다리 실측 각도 토픽 (joint_states)
extern std::string LEG_CMD_TOPIC_L;    // 왼다리 위치 명령 토픽
extern std::string LEG_CMD_TOPIC_R;    // 오른다리 위치 명령 토픽

// ===== 고속 회전 비전 게이팅 (SW1-1837, yaw 드리프트 처방) =====
//   GT 확정: gyro yaw −0.11%로 거의 완벽, VINS 융합 −2%(비전이 고속 스핀서 오염).
//   고속 회전 프레임의 재투영 factor를 skip → 그 구간 gyro(IMU preint)에 위임.
extern int    USE_YAW_GATING;        // 마스터 토글 (0=기존 동작 유지)
extern double YAW_GATE_GYR_THRESH;   // [rad/s] 프레임 평균 |ω−Bg| 이 값 초과 시 비전 관측 skip

// ===== Bg_z 잠금 (SW1-1837, yaw 드리프트 최종 처방) =====
//   최적화기가 yaw 불일치를 Bg_z(gyro z-bias)로 도피시켜 참값의 15~40배로 과대추정하는
//   것이 yaw 드리프트의 단일 지배 원인(인과 봉인 probe: 고정 시 v7 −24°→−2.7°).
//   수렴 후 Bg_z를 상수 고정하되, 발동 전 |Bg_z| 검증으로 나쁜 값 고정을 방지.
extern int    USE_BGZ_LOCK;    // 0=기존 동작 유지, 1=수렴 후 Bg_z 고정
extern double BGZ_LOCK_DELAY;  // [s] 발동 전 수렴 대기 시간
extern double BGZ_LOCK_MAX;    // [rad/s] 발동 시점 |Bg_z| 허용 상한(초과 시 잠금 보류)

// ===== 휠 회전 잔차 주변화 (SW1-1837, yaw 드리프트 처방) =====
//   휠 twist는 +65ms 지연(diff_drive_controller rolling mean)으로 회전 전이 구간서
//   틀린 delta_q를 만들어 몸체 yaw를 오염(v7 A/B: 잔차 제거 시 드리프트 −45%·결정론 회복).
//   1이면 휠 factor를 위치 3x3 제약만으로 재구성(회전은 gyro가 우월: GT −0.11% vs 휠 +2.7%).
extern int    WHEEL_ROT_MARGINALIZE; // 0=기존 6자유도 유지, 1=회전 잔차 주변화

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
