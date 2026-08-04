#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "../feature_manager/feature_manager.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../initial/initial_sfm.h"
#include "../initial/solve_5pts.h"
#include "../utility/parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

#include "../feature_tracker/feature_tracker.h"

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>

#include "../factor/imu_factor.h"
#include "../factor/wheel_factor.h"
#include "../utility/leg_event_detector.h"
#include "../utility/bgz_lock.h"
#include "../utility/warmup_init_gate.h"
#include "../factor/marginalization_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/projection_factor.h"
#include "../factor/projection_td_factor.h"
#include <ceres/ceres.h>

#include <opencv2/core/eigen.hpp>
#include <queue>
#include <unordered_map>

#include <sophus/se3.h>
#include <sophus/so3.h>

class Estimator
{
public:
    Estimator();

    void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration,
                    const Vector3d &angular_velocity);

    void processImage(map<int, Eigen::Matrix<double, 7, 1>> &image, const std_msgs::msg::Header &header);

    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points,
                      Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();

    bool initialStructure();

    bool visualInitialAlign();

    bool visualInitialAlignWithDepth();

    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);

    void slideWindow();

    void slideWindowNew();

    void slideWindowOld();

    void solveOdometry();

    void optimization();

    void vector2double();

    void double2vector();

    bool failureDetection();
    // [SW1-1866 reboot-pose-seed] failure 확정 후·clearState 전에 호출 — 시드 캡처
    void captureRebootSeed(double stamp);
    // 재init 완료 시 다리(휠/gyro 델타)를 얹어 T_seed 확정
    void finalizeRebootSeed();
    // 발행단 합성: published = T_seed ∘ session (seed_active_ 아니면 무변경)
    void seedTransform(Eigen::Vector3d &p, Eigen::Matrix3d &R) const;

    bool staticInitialAlignWithDepth();

    void updateLatestStates();

    Matrix3d predictMotion(double t0, double t1);

    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);

    void predict(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);

    bool IMUAvailable(double t);

    bool initialStructureWithDepth();
    void movingConsistencyCheck(set<int> &removeIndex);

    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                             Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                             double depth, Vector3d &uvi, Vector3d &uvj);
    double reprojectionError3D(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                               Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                               double depth, Vector3d &uvi, Vector3d &uvj);

    bool
    getIMUInterval(double t0, double t1,
                   std::vector<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> &imu_vector);
    void
    initFirstIMUPose(std::vector<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> &imu_vector);

    // ===== Wheel (VIW-Fusion 휠 factor 이식, SW1-1829) =====
    void inputWheel(double t, const Vector3d &linearVelocity, const Vector3d &angularVelocity);
    void processWheel(double t, double dt, const Vector3d &linear_velocity,
                      const Vector3d &angular_velocity);
    bool WheelAvailable(double t);
    bool getWheelInterval(double t0, double t1,
                          std::vector<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> &wheel_vector);

    // ===== [SW1-1837] 이벤트 게이팅 (Phase 1: 다리각) =====
    void inputLegState(double t, double theta_l, double theta_r);  // joint_states 실측
    void inputLegCommand(double t, double target, bool left);      // 위치 명령(선행 트리거)
    bool isLegGated(double t0, double t1);                         // factor skip 판정

    // ===== [SW1-1837] 정지 시 중력 재정렬 v2: 창 전체 자세 보정 (use_gravity_align: 2) =====
    void gravityRealignWindow();  // optimization() 직후 호출 — 정지 확정 시 1회 ΔR 일괄 보정
    void gaugeSlideGuard();       // [SW1-1866] optimization() 직후 호출 — 게이지 슬라이드
                                  //   (동적 장애물 폭주, yaw·병진) 검출·역변환

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD        = 0,
        MARGIN_SECOND_NEW = 1
    };
    bool           openExEstimation;
    FeatureTracker featureTracker;

    SolverFlag          solver_flag;
    MarginalizationFlag marginalization_flag;
    Vector3d            g;
    // extrinsic
    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    // VIO state vector
    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double   td{};

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];

    // [SW1-1837] 중력 재정렬 v2 상태 — 명시 초기화 필수(latest_Bg 미초기화 사고의 교훈)
    bool     grav_realign_done{false};                         // 정지당 1회 발동 래치
    Vector3d ba_at_realign{Vector3d::Zero()};                  // 보정 시점 Ba (관문 ④: 재수렴 관찰용)
    double   grav_realign_last_t{-1.0e18};                     // 마지막 발동 시각 (쿨다운 판정용)

    // [SW1-1837] 고속 회전 비전 게이팅 — skip한 관측 수(A/B 진단 로그용)
    long     yaw_gated_obs_{0};

    // [SW1-1866] 게이지 슬라이드 가드 상태 — 직전 solve의 최신 프레임 스탬프/yaw/위치
    double   yaw_guard_prev_stamp_{-1.0};
    double   yaw_guard_prev_yaw_deg_{0.0};
    Vector3d yaw_guard_prev_pos_{Vector3d::Zero()};
    long     yaw_guard_trigger_cnt_{0};
    int      yaw_guard_consec_{0};      // 연속 발동 solve 수 — 오염 지속 판정(절제 트리거)
    double   yaw_guard_last_warn_t_{-1.0e18};
    int      guard_amputate_streak_{0};      // 정화(무이상 solve) 없는 연속 절제 횟수
    bool     guard_escalation_fire_{false};  // 연속 절제 상한 도달 → failureDetection이 조기 재초기화

    // [SW1-1866] 정지 창 누적 변위 가드 상태 — 앵커는 정지 연속 확인 후 래치
    int      still_cum_streak_{0};                    // 연속 정지 확정 solve 수
    bool     still_cum_valid_{false};                 // 앵커 래치 여부
    Vector3d still_cum_anchor_{Vector3d::Zero()};     // 정지 창 기준 위치
    long     still_cum_trigger_cnt_{0};
    double   still_cum_last_warn_t_{-1.0e18};

    // [SW1-1866 07-31] 정지 z 래칫 가드 상태 — 앵커는 정지 창 간 '계승'
    //   (휠 병진 없음 + 다리각 복귀 시. 재래치만 하면 이벤트 중 z 스텝이 구워짐)
    double   still_cum_z_anchor_{0.0};
    bool     still_cum_z_valid_{false};
    bool     anchor_history_valid_{false};   // 이전 정지 창 앵커 실존(계승 후보) 여부
    double   anchor_yaw_deg_{0.0};           // 앵커 시점 VINS yaw(진단용 — 판정엔 미사용)
    double   anchor_net_yaw_rad_{0.0};       // 앵커 이후 물리 순회전(gyro-Bg 적분) — xy 계승 판정
    double   z_anchor_leg_l_{0.0}, z_anchor_leg_r_{0.0};  // 앵커 시점 다리각(계승 판정 기준)
    long     still_cum_z_trigger_cnt_{0};
    double   still_cum_z_last_warn_t_{-1.0e18};

    // [SW1-1866 08-04] 정지 yaw 래칫 가드 상태 — 앵커는 매 정지 창 신규(주행 중 실회전
    //   탓에 창 간 계승 불가 — z와 다른 점). 물리 회전 판정은 raw gyro 적분 −
    //   정지 실측 bias(rest 중앙값)×경과시간 — Bgs 기반이면 잠금 잔차가 슬라이드의
    //   원인이자 기준이 되는 자기참조로 가드가 무력화됨([YAW-DEV] 진단 실증). raw
    //   적분은 창 중간 재잠금(Bgs 교체)에도 불변.
    double   still_cum_yaw_anchor_deg_{0.0};
    double   still_cum_yaw_raw_net_rad_{0.0};  // 래치 이후 raw gyro z 적분(bias 미차감)
    double   still_cum_yaw_elapsed_{0.0};      // 래치 이후 경과시간(rest 중앙값 차감용)
    long     still_cum_yaw_trigger_cnt_{0};
    double   still_cum_yaw_last_warn_t_{-1.0e18};
    std::atomic<bool>   z_anchor_wheel_moved_{false};     // 앵커 이후 휠 병진 발생 → 계승 차단
    std::atomic<double> latest_leg_l_{0.0}, latest_leg_r_{0.0};  // 최신 다리각(inputLegState)
    std::atomic<bool>   latest_leg_valid_{false};

    // [SW1-1866] init/출발 워밍업 게이트 — 정지·다리 안정 실증+표본 축적 전 init 보류
    //   (판정 로직은 warmup_init_gate.h, 배선은 processIMU 표본 공급 + processImage 관문)
    warmup_init_gate::Gate warmup_gate_;
    std::atomic<bool> leg_gate_active_now_{false};  // 다리 이벤트 진행 중(게이트 표본 입력, inputLegState 갱신)
    bool warmup_fallback_logged_{false};   // 저신뢰(폴백) init 경고 1회
    bool warmup_realign_logged_{false};    // 폴백 후 재정렬 기회 신호 1회(v1=로그만)

    // [SW1-1866 reboot-pose-seed] 재초기화 pose 시드 계승 (수학=utility/reboot_seed.h)
    //   ── T_seed(발행 합성 오프셋)와 다리 적분·캡처 스냅샷은 clearState에서 지우지
    //   않는다(Q6: 상태 수명 > 세션 수명이 존재 이유). 시드 '재료'(정화 pose·앵커
    //   래치 시각·첫 절제 시각)는 세션 스코프라 clearState에서 리셋한다 — 캡처가
    //   clearState보다 먼저 실행되므로(processImage failure 경로) 안전.
    bool     seed_active_{false};                       // T_seed 합성 on (발행단)
    Eigen::Matrix3d seed_R_{Eigen::Matrix3d::Identity()};  // T_seed 회전(yaw only)
    Eigen::Vector3d seed_P_{Eigen::Vector3d::Zero()};      // T_seed 병진
    bool     seed_pending_{false};       // 캡처됨, 재init 완료 시 다리 얹어 확정 대기
    Eigen::Vector3d seed_cap_P_{Eigen::Vector3d::Zero()};  // 캡처된 시드 pose(발행 프레임)
    double   seed_cap_yaw_{0.0};
    double   seed_cap_gyro_yaw_{0.0};    // 캡처 시점 다리 적분 스냅샷
    double   seed_cap_wheel_x_{0.0}, seed_cap_wheel_y_{0.0}, seed_cap_wheel_yaw_{0.0};
    double   bridge_gyro_yaw_rad_{0.0};  // raw gyro z 상시 적분(리셋 금지 — 다리 yaw)
    long     seed_apply_cnt_{0};         // 텔레메트리
    // 시드 재료(세션 스코프 — clearState 리셋)
    double   clean_pose_t_{-1.0};        // 마지막 정화 solve 시각 (2순위 시드)
    Eigen::Vector3d clean_P_{Eigen::Vector3d::Zero()};
    double   clean_yaw_{0.0};
    double   anchor_latch_t_{-1.0};      // 앵커 래치 시각 (1순위 Q4 자격 판정)
    double   amputate_first_t_{-1.0};    // 현 에피소드 첫 절제 시각
    // 휠 odom 최신 pose (외부 노드라 reboot 무관 연속 — 다리 병진 소스, 콜백 갱신)
    std::atomic<double> latest_wheel_x_{0.0}, latest_wheel_y_{0.0}, latest_wheel_yaw_{0.0};

    // [SW1-1837] Bg_z 잠금 상태 — 상태 기반 발동 추적기·정지 실측·재잠금(온도 표류 추종)
    bool               bgz_locked_{false};
    bgz_lock::Tracker  bgz_lock_tracker_;
    bgz_lock::RestBias bgz_rest_;                // 정지 중 원시 gyro z 중앙값(직접 물리 관측)
    double             bgz_rest_t_{0.0};         // 실측 표본용 dt 누적 의사시간(processIMU)
    std::atomic<double> last_wheel_speed_{0.0};  // 최근 휠 twist 크기 — 준정지(느린 잔여
                                                 // 회전) 오인 방지용 정지 판별자(inputWheel 갱신)
    double             bgz_locked_val_{0.0};     // 현재 잠긴 값 (재잠금 괴리 판정 기준)
    double             bgz_last_relock_t_{-1.0e18};  // 마지막 재잠금 시각 (쿨다운)

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)]{};
    Vector3d         acc_0, gyr_0;

    vector<double>   dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    // ===== Wheel preintegration 상태/버퍼 (IMU 미러) =====
    Matrix3d              rio;                       // 휠-body extrinsic 회전 (T_io)
    Vector3d              tio;                        // 휠-body extrinsic 병진
    double                sx = 1, sy = 1, sw = 1;     // 휠 intrinsic (vel x/y, yaw 스케일)
    double                td_wheel{};                 // 휠 시간오프셋
    bool                  openExWheelEstimation{};    // Step2: 온라인 휠 extrinsic 추정
    bool                  openIxEstimation{};         // Step2: 온라인 휠 intrinsic 추정
    bool                  first_wheel{};
    WheelIntegrationBase *pre_integrations_wheel[(WINDOW_SIZE + 1)]{};
    Vector3d              vel_0_wheel, gyr_0_wheel;
    vector<double>        dt_buf_wheel[(WINDOW_SIZE + 1)];
    vector<Vector3d>      linear_velocity_buf_wheel[(WINDOW_SIZE + 1)];
    vector<Vector3d>      angular_velocity_buf_wheel[(WINDOW_SIZE + 1)];
    WheelIntegrationBase *tmp_wheel_pre_integration{};

    int frame_count{};  // cl:滑动窗口中帧的数目,最大为滑窗大小
    int sum_of_back{}, sum_of_front{}, sum_of_invalid{};

    FeatureManager    f_manager;
    MotionEstimator   m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu{};
    bool is_valid{};
    bool failure_occur{};

    vector<Vector3d> key_poses;
    double           initial_timestamp{};

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE]{};

    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS]{};
    double para_Feature[NUM_OF_F][SIZE_FEATURE]{};
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE]{};
    double para_Td[1][1]{};
    // ===== Wheel 최적화 파라미터 블록 (Step1 고정 / Step2 free) =====
    double para_Ex_Pose_wheel[1][SIZE_POSE]{};   // T_io (휠 extrinsic)
    double para_Ix_sx_wheel[1][1]{};
    double para_Ix_sy_wheel[1][1]{};
    double para_Ix_sw_wheel[1][1]{};
    double para_Td_wheel[1][1]{};

    // ===== Ground-plane constraint (SW1-1837, VIW-Fusion plane_factor 이식) =====
    double          para_plane_R[1][4]{{0, 0, 0, 1}};   // 지면평면 방향 quaternion (x,y,z,w)
    double          para_plane_Z[1][1]{};               // 지면평면 높이 zpw
    Eigen::Matrix3d rpw                 = Eigen::Matrix3d::Identity();  // 평면 방향(world→plane)
    double          zpw                 = 0.0;          // 평면 높이
    bool            openPlaneEstimation = false;        // VI 초기화 완료 후 평면 추정 활성
    void            initPlane();                        // 윈도 pose 평균으로 평면 초기화

    int    find_solved[WINDOW_SIZE + 1]{};

    MarginalizationInfo *last_marginalization_info{};
    vector<double *>     last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase        *tmp_pre_integration{};

    // relocalization variable
    bool             relocalization_info{};
    double           relo_frame_stamp{};
    double           relo_frame_index{};
    int              relo_frame_local_index{};
    vector<Vector3d> match_points;
    double           relo_Pose[SIZE_POSE]{};
    Matrix3d         drift_correct_r;
    Vector3d         drift_correct_t;
    Vector3d         prev_relo_t;
    Matrix3d         prev_relo_r;
    Vector3d         relo_relative_t;
    Quaterniond      relo_relative_q;
    double           relo_relative_yaw{};

    std::mutex m_imu, m_propagate;

    bool   init_imu{};
    double prevTime = -1;

    queue<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_buf;

    // ===== Wheel 비동기 입력 버퍼 (inputWheel → getWheelInterval) =====
    std::mutex                            m_wheel;
    queue<pair<double, Eigen::Vector3d>>  wheelVelBuf;
    queue<pair<double, Eigen::Vector3d>>  wheelGyrBuf;
    double                                prevTime_wheel = -1, curTime_wheel{};

    // ===== [SW1-1837] 다리 이벤트 구간 마킹 (이벤트 게이팅) =====
    // 콜백 스레드(입력)와 process 스레드(optimization 판정)가 함께 접근 → mutex 필수
    std::mutex       m_leg_gate;
    LegEventDetector leg_gate;

    // [SW1-1837] ★잠복 버그 수정: Eigen 기본 생성자는 메모리를 초기화하지 않는데,
    //   latest_Bg는 VIO 초기화 완료 전에도 predictMotion()의 gyro bias로 읽힌다(estimator.cpp:2359).
    //   힙 쓰레기가 NaN 패턴이면 회전 예측 전체가 NaN → 특징 추적 전멸 → 초기화 영구 불가.
    //   (객체 레이아웃이 바뀌자 발현 — 07-13/14 재생 동결 사건의 근본 원인. 명시 초기화로 차단)
    double             latest_time{};
    Eigen::Vector3d    latest_P  = Eigen::Vector3d::Zero();
    Eigen::Quaterniond latest_Q  = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    latest_V  = Eigen::Vector3d::Zero();
    Eigen::Vector3d    latest_Ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d    latest_Bg = Eigen::Vector3d::Zero();
    bool               initFirstPoseFlag{};
};
