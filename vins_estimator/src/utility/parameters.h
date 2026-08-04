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

// ===== 스핀 중 accel 신뢰 강등 (SW1-1837, 전방 활주 처방) =====
//   스핀 중 원심가속 ω²r(레버 0.1056m)이 중력 기울기 1~2°로 오해돼 회전 비례 전방
//   활주(닫힌루프 잔여 오차 지배)를 만든다(v10 포렌식 07-26). 고속 회전 샘플의 acc
//   노이즈만 인플레 → 자세는 gyro, 위치는 휠에 위임. 근거·수치는 spin_acc_deweight.h.
extern int    USE_SPIN_ACC_DEWEIGHT;         // 마스터 토글 (0=기존 동작 유지)
extern double SPIN_ACC_DEWEIGHT_GYR_THRESH;  // [rad/s] 샘플 |ω−Bg| 초과 시 발동
extern double SPIN_ACC_DEWEIGHT_FACTOR;      // acc 노이즈 σ 배율 (>1)

// ===== Bg_z 잠금 (SW1-1837, yaw 드리프트 최종 처방) =====
//   최적화기가 yaw 불일치를 Bg_z(gyro z-bias)로 도피시켜 참값의 15~40배로 과대추정하는
//   것이 yaw 드리프트의 단일 지배 원인(인과 봉인 probe: 고정 시 v7 −24°→−2.7°).
//   발동은 상태 기반(정지 지속 + 추정 안정 + 크기 가드) — 시간 고정 delay는 검증 bag
//   안무 의존이라 B2C(시동 직후 조작)에 부적합했음. 조건별 근거는 bgz_lock.h 참조.
extern int    USE_BGZ_LOCK;       // 0=기존 동작 유지, 1=상태 조건 성립 시 Bg_z 고정
extern double BGZ_LOCK_DELAY;     // [s] 첫 최적화 후 최소 대기(1s 과도 스파이크 회피 벨트)
extern double BGZ_LOCK_STILL_SEC; // [s] 정지 지속 요구 시간(= 안정성 판정 창 길이)
extern double BGZ_LOCK_STAB_MAX;  // [rad/s] 창 내 Bg_z 변동폭(max-min) 허용 상한
extern double BGZ_LOCK_FALLBACK_SEC; // [s] 정지 누적 시 중앙값 폴백 발동(요동 세션 대응, <=0=off)
extern double BGZ_LOCK_MAX;       // [rad/s] |추정 − 정지 실측| 허용 상한(괴리 시 잠금 보류)
                                  //   구 절대 크기 가드에서 변경 — warm 참 bias(온도 표류)와
                                  //   인플레는 크기로 구분 불가, 정지 실측과의 거리로만 구분됨
extern double BGZ_RELOCK_DELTA;   // [rad/s] 재잠금 문턱: |정지 실측 − 잠금값| 초과 시
                                  //   온도 표류 추종 갱신 (<=0=재잠금 off)
extern double BGZ_RELOCK_WIN_SEC; // [s] 재잠금 판정용 연속 정지 창(=중앙값 창). 실기
                                  //   온도 표류 관찰로 튜닝 대상(cold bag 미검증)
extern double BGZ_RELOCK_COOLDOWN;// [s] 재잠금 최소 간격 — 잦은 갱신 방지

// ===== 고주기 body TF (SW1-1837) =====
//   기본 body TF는 윈도 최적화 후 발행이라 실기 100~500ms 지연 — rviz TF 비교·실시간
//   소비(nav/도킹)에 부적합. 1이면 IMU 전파 자세(imu_propagate와 동일)를 100Hz 스로틀로
//   map→body TF 송출하고 저주기 송출은 중단(이중 소스 널뛰기 방지).
extern int PUB_HF_BODY_TF;        // 0=기존(최적화 후 저주기), 1=IMU 전파 고주기 TF
extern double HF_BODY_TF_TAU;     // [s] 고주기 TF 발행단 스무딩 시정수(예측-보정 스냅 떨림
                                  //   감쇠). 클수록 부드럽지만 반응 지연 증가. <=0=스무딩 끔

// ===== rviz 비교용 VINS footprint TF (SW1-1837) =====
//   body(IMU)는 축중심에서 TIO(0.1056,0,-0.0941)m 떨어져 있어 실 로봇 base_link와
//   그냥 겹치면 항상 ~14cm 어긋나 보임(회전 시 레버암 원). 1이면 body에 정적 자식
//   vins/base_link(=RIO/TIO)·vins/base_footprint를 붙여 bringup TF와 같은 의미의
//   점끼리 비교 가능. base_footprint는 diff_drive가 부모(odom→)를 이미 갖고 있어
//   이름 충돌 금지 → vins/ 네임스페이스 필수.
extern int PUB_VINS_FOOTPRINT_TF; // 0=끔(기본), 1=vins/base_link·vins/base_footprint 정적 TF

// ===== 게이지 슬라이드 가드 (SW1-1866) =====
//   동적 장애물(초근접+화면 점령)이 marg prior를 오염시키면 매 solve 창 전체가 게이지
//   (불관측) 방향으로 일정량 이동하는 폭주가 발생: yaw 13~16°/solve(-90~-160°/s 연속
//   회전) — yaw만 막으면 병진 게이지로 압력이 전이(0.26~1.5m/s 활주, A/B 실증) →
//   yaw+병진 동시 봉쇄로 게이지 자유 방향 밀폐. 검출='같은 물리 프레임의 solve 간
//   재추정 이동'(정상: yaw 0.0x°·위치 mm), 처치=창+marg prior 선형화점 역변환.
//   원리 상세: yaw_slide_guard.h
// ===== 정지 상대운동 잠금 (SW1-1866) =====
//   정지 확정(휠+gyro 합의) 구간의 인접 프레임에 '상대 위치·yaw=0' 관측을 주입 —
//   오염 비전이 새 프레임을 돌려 놓는 '배치 오차'(사후 가드의 사각)를 태어날 때 차단.
extern int    USE_STILL_MOTION_LOCK;    // 0=끔, 1=정지 구간 상대운동 잠금
extern double STILL_LOCK_POS_W;         // 위치 가중치 = 1/σ_p [1/m]
extern double STILL_LOCK_YAW_W;         // yaw 가중치 = 1/σ_yaw [1/rad]

extern int    USE_GAUGE_SLIDE_GUARD;    // 0=끔, 1=검출+역변환(yaw·병진)
extern int    USE_STILL_CUM_GUARD;      // 0=끔, 1=정지 창 누적 xy 변위 상한(총량 유계)
extern double STILL_CUM_XY_MAX;         // [m] 정지 창 앵커 대비 누적 xy 상한
extern double STILL_CUM_Z_MAX;          // [m] 정지 창 z 래칫 상한(0=끔) — 앵커는 창 간 계승
extern int    KEEP_WHEEL_IN_LEG_EVENT;  // 1=다리 이벤트 중 휠 factor 유지(기본) 0=하드 skip 롤백
extern double YAW_SLIDE_GUARD_THRESH;   // [rad] 주행 중 solve당 yaw 이동 문턱(기본 3°)
extern double POS_SLIDE_GUARD_THRESH;   // [m]   주행 중 solve당 위치 이동 문턱(기본 0.05)
extern double YAW_SLIDE_GUARD_STILL_THRESH;  // [rad] 정지 확정 시 문턱(기본 0.1° — pose 고정)
extern double POS_SLIDE_GUARD_STILL_THRESH;  // [m]   정지 확정 시 문턱(기본 0.005)
extern int    GUARD_ESCALATION_MAX;     // 정화 없는 연속 prior 절제 상한(0=비활성) — 도달 시 조기 재초기화

// ===== 휠 회전 잔차 주변화 (SW1-1837, yaw 드리프트 처방) =====
//   휠 twist는 +65ms 지연(diff_drive_controller rolling mean)으로 회전 전이 구간서
//   틀린 delta_q를 만들어 몸체 yaw를 오염(v7 A/B: 잔차 제거 시 드리프트 −45%·결정론 회복).
//   1이면 휠 factor를 위치 3x3 제약만으로 재구성(회전은 gyro가 우월: GT −0.11% vs 휠 +2.7%).
extern int    WHEEL_ROT_MARGINALIZE; // 0=기존 6자유도 유지, 1=회전 잔차 주변화

// ===== 정지 yaw 래칫 가드 (SW1-1866 08-04, v14 굽힘 타임라인 실증) =====
//   정지 창에서 창 전체 yaw의 문턱 이하 미세 게이지 슬라이드 총량 유계.
extern double STILL_CUM_YAW_MAX_DEG;  // [deg] 정지 창 누적 yaw 상한 (<=0 비활성)

// ===== 재초기화 pose 시드 계승 (SW1-1866 reboot-pose-seed) =====
//   조기 재초기화(4선) 시 마지막 건전 pose를 시드로 발행단 합성 — 원점 점프 제거.
extern int    USE_REBOOT_POSE_SEED;   // 0=기존(원점 복귀), 1=시드 계승

// ===== init/출발 워밍업 게이트 (SW1-1866) — 전부 표본 수 조건(시간 상수 금지) =====
extern int    WARMUP_GATE_STILL_SAMPLES;   // 1단 정지 실증 연속 표본 수 (<=0 게이트 비활성)
extern int    WARMUP_GATE_IMU_SAMPLES;     // 2단 IMU 표본 축적 수 (ALICE 100 선례)
extern int    WARMUP_GATE_MOVING_SAMPLES;  // 조기 폴백: 주행 연속 실증 표본 수 (<=0 없음)
extern int    WARMUP_GATE_BUDGET_SAMPLES;  // 예산 폴백: 총 관측 상한 (무한 대기 금지)

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
