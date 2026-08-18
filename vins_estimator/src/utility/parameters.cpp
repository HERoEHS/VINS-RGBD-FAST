#include "parameters.h"

#include <thread>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;
double VEL_N_wheel, GYR_N_wheel;

// ===== Zero-velocity Update (ZUPT, SW1-1837) =====
int    USE_ZUPT;
double ZUPT_VEL_THRESH, ZUPT_GYR_THRESH, ZUPT_WEIGHT;

// ===== 정지 시 중력 재정렬 (SW1-1837) =====
int    USE_GRAVITY_ALIGN;
double GRAVITY_ALIGN_WEIGHT, GRAVITY_ALIGN_VEL_THRESH, GRAVITY_ALIGN_GYR_THRESH;
double GRAVITY_ALIGN_MIN_ANGLE, GRAVITY_ALIGN_MAX_ANGLE, GRAVITY_ALIGN_COOLDOWN;

// ===== Accelerometer-bias prior (SW1-1836) =====
int    USE_ACC_BIAS_PRIOR;
double ACC_BIAS_PRIOR_W_XY, ACC_BIAS_PRIOR_W_Z;

// ===== Vertical-velocity soft constraint (SW1-1837, planar-motion Level1) =====
int    USE_VERTICAL_VEL;
double VERTICAL_VEL_WEIGHT;

// ===== Ground-plane constraint (SW1-1837, VIW-Fusion plane_factor 이식) =====
int    USE_PLANE;
double PITCH_N_INV, ROLL_N_INV, ZPW_N_INV;

// ===== Body-frame NHC (SW1-1837, planar-motion Level2) =====
int    USE_BODY_NHC;
double NHC_Y_WEIGHT, NHC_Z_WEIGHT;

// ===== Wheel velocity outlier 게이팅 (SW1-1837) =====
int    USE_WHEEL_VEL_GATE;
double WHEEL_VEL_MAX, WHEEL_GYR_MAX;

// ===== [SW1-1837] 이벤트 게이팅 (Phase 1: 다리각) =====
int         USE_EVENT_GATING;
int         GATE_LEG;
double      LEG_POS_MIN, LEG_RATE_MIN, LEG_CMD_POS_MIN;
double      LEG_PRE_MARGIN, LEG_POST_MARGIN, GATE_MAX_DURATION;
int         KEEP_WHEEL_IN_LEG_EVENT = 1;  // 1=다리 이벤트 중 휠 factor 유지(기본) 0=하드 skip(레거시 롤백)
std::string LEG_STATE_TOPIC, LEG_CMD_TOPIC_L, LEG_CMD_TOPIC_R;

// ===== 고속 회전 비전 게이팅 (SW1-1837, yaw 처방) =====
int    USE_YAW_GATING;
double YAW_GATE_GYR_THRESH;

// ===== 스핀 중 accel 신뢰 강등 (SW1-1837, 전방 활주 처방) =====
int    USE_SPIN_ACC_DEWEIGHT;
double SPIN_ACC_DEWEIGHT_GYR_THRESH = 0.6;
double SPIN_ACC_DEWEIGHT_FACTOR     = 10.0;

// ===== Bg_z 잠금 (SW1-1837, yaw 최종 처방) =====
int    USE_BGZ_LOCK;
double BGZ_LOCK_DELAY     = 3.0;
double BGZ_LOCK_STILL_SEC = 2.0;
double BGZ_LOCK_STAB_MAX     = 2e-4;
double BGZ_LOCK_FALLBACK_SEC = 8.0;
double BGZ_LOCK_MAX          = 0.001;
double BGZ_RELOCK_DELTA      = 5e-4;
double BGZ_RELOCK_WIN_SEC    = 10.0;
double BGZ_RELOCK_COOLDOWN   = 10.0;

// ===== 고주기 body TF (SW1-1837) =====
int    PUB_HF_BODY_TF = 0;
double HF_BODY_TF_RATE_HZ = 100.0;
double HF_BODY_TF_TAU = 0.05;
int    PUB_VINS_FOOTPRINT_TF = 0;
int    PUB_ODOM_BASE_VINS_TF = 0;
int    USE_STILL_MOTION_LOCK = 0;
double STILL_LOCK_POS_W = 500.0;   // σ_p 2mm
double STILL_LOCK_YAW_W = 573.0;   // σ_yaw 0.1°
int    USE_GAUGE_SLIDE_GUARD = 0;
int    USE_STILL_CUM_GUARD = 0;
double STILL_CUM_XY_MAX = 0.03;
double STILL_CUM_Z_MAX  = 0.0;  // 0=끔 — yaml 키 없는 구형 config에서 동작 불변(보수 기본)
double YAW_SLIDE_GUARD_THRESH = 3.0 * M_PI / 180.0;
int    GUARD_ESCALATION_MAX   = 3;  // 정화 없는 연속 절제 상한(0=비활성) — 조기 재초기화 판정
double POS_SLIDE_GUARD_THRESH = 0.05;
double YAW_SLIDE_GUARD_STILL_THRESH = 0.1 * M_PI / 180.0;
double POS_SLIDE_GUARD_STILL_THRESH = 0.005;
// [SW1-1866 08-10 R1] failureDetection 문턱 — 기본값은 종전 하드코딩 값 그대로(무변경).
double FAILURE_BA_MAX = 2.5;
double FAILURE_BG_MAX = 1.0;
double FAILURE_DP_MAX = 5.0;
double FAILURE_DZ_MAX = 1.0;

// ===== 휠 회전 잔차 주변화 (SW1-1837, yaw 처방) =====
int    WHEEL_ROT_MARGINALIZE;

// ===== 정지 yaw 래칫 가드 (SW1-1866 08-04) =====
double STILL_CUM_YAW_MAX_DEG;

// ===== 재초기화 pose 시드 계승 (SW1-1866 reboot-pose-seed) =====
int    USE_REBOOT_POSE_SEED;

// ===== 정지 중 발산 가드 (SW1-1866 08-11) — 기본값도 yaml이 정본, 아래는 키 부재 시 폴백 =====
int    USE_STILL_DRIFT_GUARD = 0;      // 기본 0 = 종전 동작
double STILL_DRIFT_MAX       = 0.03;
double STILL_DRIFT_WINDOW_SEC   = 0.5;
double STILL_CHECK_DURATION_SEC = 2.0;
double STILL_CHECK_XY_TOL = 0.005;
double STILL_CHECK_YAW_TOL_DEG = 2.0;
int    STILL_DRIFT_CONSEC    = 3;

// ===== 출력 map 핀 (SW1-1866 vins-output-map-anchor) =====
int    USE_OUTPUT_MAP_ANCHOR;

// ===== init/출발 워밍업 게이트 (SW1-1866) =====
int    WARMUP_GATE_STILL_SAMPLES;
int    WARMUP_GATE_IMU_SAMPLES;
int    WARMUP_GATE_MOVING_SAMPLES;
int    WARMUP_GATE_BUDGET_SAMPLES;

// ===== Wheel odometry tight-coupling (SW1-1829) =====
int             USE_WHEEL;
std::string     WHEEL_TOPIC;
Eigen::Matrix3d RIO = Eigen::Matrix3d::Identity();
Eigen::Vector3d TIO = Eigen::Vector3d::Zero();
double          SX = 1.0, SY = 1.0, SW = 1.0;
double          TD_WHEEL = 0.0;
int             ESTIMATE_EXTRINSIC_WHEEL;
int             ESTIMATE_INTRINSIC_WHEEL;
int             ESTIMATE_TD_WHEEL;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMAGE_TOPIC;
std::string DEPTH_TOPIC;
std::string IMU_TOPIC;

int MAX_CNT;
int MAX_CNT_SET;
int MIN_DIST;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int EQUALIZE;
int FISHEYE;
std::string FISHEYE_MASK;
std::string CAM_NAMES;
int STEREO_TRACK;
bool PUB_THIS_FRAME;
Eigen::Matrix3d Ric;

double ROW, COL;
int IMAGE_SIZE;
double TD, TR;

double DEPTH_MIN_DIST;
double DEPTH_MAX_DIST;
unsigned short DEPTH_MIN_DIST_MM;
unsigned short DEPTH_MAX_DIST_MM;

std::vector<std::string> SEMANTIC_LABEL;
std::vector<std::string> STATIC_LABEL;
std::vector<std::string> DYNAMIC_LABEL;

int NUM_GRID_ROWS;
int NUM_GRID_COLS;

int FRONTEND_FREQ;
int USE_IMU;
int NUM_THREADS;
int STATIC_INIT;

int FIX_DEPTH;

template <typename T>
T readParam(rclcpp::Node* node, const std::string &name)
{
    T ans{};
    node->declare_parameter(name, ans);
    if (node->get_parameter(name, ans))
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Loaded " << name << ": " << ans);
    }
    else
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to load " << name);
        rclcpp::shutdown();
    }
    return ans;
}

void readParameters(rclcpp::Node* node)
{
    std::string config_file;
    config_file = readParam<std::string>(node, "config_file");
    cv::FileStorage fsSettings;
    fsSettings.open(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    NUM_THREADS = fsSettings["num_threads"];
    if (NUM_THREADS <= 1)
    {
        NUM_THREADS = std::thread::hardware_concurrency();
    }

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["depth_topic"] >> DEPTH_TOPIC;

    MAX_CNT = fsSettings["max_cnt"];
    MAX_CNT_SET = MAX_CNT;
    MIN_DIST = fsSettings["min_dist"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];

    std::string VINS_FOLDER_PATH = readParam<std::string>(node, "vins_folder");
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES = config_file;

    DEPTH_MIN_DIST = fsSettings["depth_min_dist"];
    DEPTH_MAX_DIST = fsSettings["depth_max_dist"];
    DEPTH_MIN_DIST_MM = DEPTH_MIN_DIST * 1000;
    DEPTH_MAX_DIST_MM = DEPTH_MAX_DIST * 1000;

    NUM_GRID_ROWS = fsSettings["num_grid_rows"];
    NUM_GRID_COLS = fsSettings["num_grid_cols"];
    RCLCPP_INFO(node->get_logger(), "NUM_GRID_ROWS: %d, NUM_GRID_COLS: %d", NUM_GRID_ROWS, NUM_GRID_COLS);

    FRONTEND_FREQ = fsSettings["frontend_freq"];
    RCLCPP_INFO(node->get_logger(), "FRONTEND_FREQ: %d", FRONTEND_FREQ);

    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    RCLCPP_INFO(node->get_logger(), "keyframe_parallax: %f", MIN_PARALLAX);
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    USE_IMU = fsSettings["imu"];
    RCLCPP_INFO(node->get_logger(), "USE_IMU: %d", USE_IMU);
    if (USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        // 휠 preintegration 노이즈 (없으면 기본값) — Step1 휠 factor 이식용
        VEL_N_wheel = fsSettings["vel_n_wheel"].empty() ? 0.05 : (double)fsSettings["vel_n_wheel"];
        GYR_N_wheel = fsSettings["gyr_n_wheel"].empty() ? 0.05 : (double)fsSettings["gyr_n_wheel"];
        G.z() = fsSettings["g_norm"];
    }

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    IMAGE_SIZE = ROW * COL;
    RCLCPP_INFO(node->get_logger(), "ROW: %f COL: %f", ROW, COL);

    for (auto iter : fsSettings["semantic_label"])
        SEMANTIC_LABEL.emplace_back(iter.string());

    for (auto iter : fsSettings["static_label"])
        STATIC_LABEL.emplace_back(iter.string());

    for (auto iter : fsSettings["dynamic_label"])
        DYNAMIC_LABEL.emplace_back(iter.string());

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        RCLCPP_WARN(node->get_logger(), "have no prior about extrinsic param, calibrate extrinsic param");
        RIC.emplace_back(Eigen::Matrix3d::Identity());
        TIC.emplace_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.txt";
    }
    else
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            RCLCPP_WARN(node->get_logger(), "Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.txt";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            RCLCPP_WARN(node->get_logger(), "fix extrinsic param");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        Ric = eigen_R;
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        RCLCPP_INFO_STREAM(node->get_logger(), "Extrinsic_R : " << std::endl << RIC[0]);
        RCLCPP_INFO_STREAM(node->get_logger(), "Extrinsic_T : " << std::endl << TIC[0].transpose());
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        RCLCPP_INFO_STREAM(node->get_logger(), "Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        RCLCPP_INFO_STREAM(node->get_logger(), "Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        RCLCPP_INFO_STREAM(node->get_logger(), "rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    STATIC_INIT = fsSettings["static_init"];
    if (!fsSettings["fix_depth"].empty())
        FIX_DEPTH = fsSettings["fix_depth"];
    else
        FIX_DEPTH = 1;

    // ===== Wheel odometry tight-coupling (SW1-1829) =====
    // use_wheel 키가 없으면 USE_WHEEL=0 → 휠 코드 경로 전부 skip(기존 VINS 동작 유지)
    USE_WHEEL = fsSettings["use_wheel"].empty() ? 0 : (int)fsSettings["use_wheel"];
    if (USE_WHEEL)
    {
        fsSettings["wheel_topic"] >> WHEEL_TOPIC;
        printf("WHEEL_TOPIC: %s\n", WHEEL_TOPIC.c_str());

        // 휠 preintegration 노이즈 (위 IMU 블록에서 이미 읽었으나, USE_IMU=0인 경우 대비 재확인)
        VEL_N_wheel = fsSettings["vel_n_wheel"].empty() ? 0.05 : (double)fsSettings["vel_n_wheel"];
        GYR_N_wheel = fsSettings["gyr_n_wheel"].empty() ? 0.05 : (double)fsSettings["gyr_n_wheel"];

        // 휠 intrinsic 스케일 (없으면 1.0)
        SX = fsSettings["sx"].empty() ? 1.0 : (double)fsSettings["sx"];
        SY = fsSettings["sy"].empty() ? 1.0 : (double)fsSettings["sy"];
        SW = fsSettings["sw"].empty() ? 1.0 : (double)fsSettings["sw"];

        // 휠-body extrinsic (T_io): wheel.yaml에서 4x4 행렬로 제공
        cv::Mat cv_T_io;
        fsSettings["body_T_wheel"] >> cv_T_io;
        if (!cv_T_io.empty())
        {
            Eigen::Matrix4d T_io;
            cv::cv2eigen(cv_T_io, T_io);
            RIO = T_io.block<3, 3>(0, 0);
            TIO = T_io.block<3, 1>(0, 3);
        }
        RCLCPP_INFO_STREAM(node->get_logger(), "Wheel Extrinsic_R(RIO):" << std::endl << RIO);
        RCLCPP_INFO_STREAM(node->get_logger(), "Wheel Extrinsic_T(TIO):" << TIO.transpose());

        TD_WHEEL = fsSettings["td_wheel"].empty() ? 0.0 : (double)fsSettings["td_wheel"];
        // Step1: extrinsic/intrinsic/td 모두 고정(=0). Step2에서 config로 1 지정 가능
        ESTIMATE_EXTRINSIC_WHEEL = fsSettings["estimate_extrinsic_wheel"].empty()
                                       ? 0 : (int)fsSettings["estimate_extrinsic_wheel"];
        ESTIMATE_INTRINSIC_WHEEL = fsSettings["estimate_intrinsic_wheel"].empty()
                                       ? 0 : (int)fsSettings["estimate_intrinsic_wheel"];
        ESTIMATE_TD_WHEEL = fsSettings["estimate_td_wheel"].empty()
                                ? 0 : (int)fsSettings["estimate_td_wheel"];
        RCLCPP_INFO(node->get_logger(),
                    "USE_WHEEL: 1, sx=%.4f sy=%.4f sw=%.4f, est_ex=%d est_ix=%d est_td=%d",
                    SX, SY, SW, ESTIMATE_EXTRINSIC_WHEEL, ESTIMATE_INTRINSIC_WHEEL, ESTIMATE_TD_WHEEL);
    }

    // ===== Zero-velocity Update (ZUPT, SW1-1837) =====
    // use_zupt 키 없으면 0 → 비활성(기존 동작 유지). 정지 구간 속도0 제약으로 z drift 완화.
    USE_ZUPT = fsSettings["use_zupt"].empty() ? 0 : (int)fsSettings["use_zupt"];
    if (USE_ZUPT)
    {
        ZUPT_VEL_THRESH = fsSettings["zupt_vel_thresh"].empty() ? 0.02 : (double)fsSettings["zupt_vel_thresh"];
        ZUPT_GYR_THRESH = fsSettings["zupt_gyr_thresh"].empty() ? 0.02 : (double)fsSettings["zupt_gyr_thresh"];
        ZUPT_WEIGHT     = fsSettings["zupt_weight"].empty() ? 100.0 : (double)fsSettings["zupt_weight"];
        RCLCPP_INFO(node->get_logger(), "USE_ZUPT: 1, vel_th=%.3f gyr_th=%.3f weight=%.1f",
                    ZUPT_VEL_THRESH, ZUPT_GYR_THRESH, ZUPT_WEIGHT);
    }

    // ===== 정지 시 중력 재정렬 (SW1-1837) =====
    // use_gravity_align 키 없으면 0 → 비활성(기존 동작 유지). 정지 프레임 acc 평균(≈중력 방향)으로
    // 이벤트 게이팅이 못 막은 잔여 roll/pitch 오차를 사후 교정. 임계는 ZUPT와 별도(use_zupt:0이어도 동작).
    USE_GRAVITY_ALIGN = fsSettings["use_gravity_align"].empty() ? 0 : (int)fsSettings["use_gravity_align"];
    if (USE_GRAVITY_ALIGN)
    {
        GRAVITY_ALIGN_WEIGHT     = fsSettings["gravity_align_weight"].empty() ? 200.0 : (double)fsSettings["gravity_align_weight"];
        GRAVITY_ALIGN_VEL_THRESH = fsSettings["gravity_align_vel_thresh"].empty() ? 0.02 : (double)fsSettings["gravity_align_vel_thresh"];
        GRAVITY_ALIGN_GYR_THRESH = fsSettings["gravity_align_gyr_thresh"].empty() ? 0.02 : (double)fsSettings["gravity_align_gyr_thresh"];
        // 모드 2(창 전체 보정) 전용 — config는 도(°) 단위, 내부는 rad
        GRAVITY_ALIGN_MIN_ANGLE = (fsSettings["gravity_align_min_angle"].empty() ? 0.5
                                       : (double)fsSettings["gravity_align_min_angle"]) * M_PI / 180.0;
        GRAVITY_ALIGN_MAX_ANGLE = (fsSettings["gravity_align_max_angle"].empty() ? 3.0
                                       : (double)fsSettings["gravity_align_max_angle"]) * M_PI / 180.0;
        GRAVITY_ALIGN_COOLDOWN  = fsSettings["gravity_align_cooldown"].empty() ? 10.0
                                       : (double)fsSettings["gravity_align_cooldown"];
        RCLCPP_INFO(node->get_logger(),
                    "USE_GRAVITY_ALIGN: %d, weight=%.1f vel_th=%.3f gyr_th=%.3f min=%.1fdeg max=%.1fdeg cooldown=%.1fs",
                    USE_GRAVITY_ALIGN, GRAVITY_ALIGN_WEIGHT, GRAVITY_ALIGN_VEL_THRESH,
                    GRAVITY_ALIGN_GYR_THRESH, GRAVITY_ALIGN_MIN_ANGLE * 180.0 / M_PI,
                    GRAVITY_ALIGN_MAX_ANGLE * 180.0 / M_PI, GRAVITY_ALIGN_COOLDOWN);
    }

    // ===== Accelerometer-bias prior (SW1-1836) =====
    // use_acc_bias_prior 키 없으면 0 → 비활성(기존 동작 유지). 수평 acc bias 과대추정 억제용.
    // 부팅 시 IMU 드라이버가 bias를 ~0 보정하므로 target=0에 당긴다. az는 이미 정확 → w_z 기본 0.
    USE_ACC_BIAS_PRIOR = fsSettings["use_acc_bias_prior"].empty() ? 0 : (int)fsSettings["use_acc_bias_prior"];
    if (USE_ACC_BIAS_PRIOR)
    {
        ACC_BIAS_PRIOR_W_XY = fsSettings["acc_bias_prior_w_xy"].empty() ? 50.0 : (double)fsSettings["acc_bias_prior_w_xy"];
        ACC_BIAS_PRIOR_W_Z  = fsSettings["acc_bias_prior_w_z"].empty()  ? 0.0  : (double)fsSettings["acc_bias_prior_w_z"];
        RCLCPP_INFO(node->get_logger(), "USE_ACC_BIAS_PRIOR: 1, w_xy=%.2f w_z=%.2f (target=0)",
                    ACC_BIAS_PRIOR_W_XY, ACC_BIAS_PRIOR_W_Z);
    }

    // ===== Vertical-velocity soft constraint (SW1-1837, planar-motion Level1) =====
    // use_vertical_vel 키 없으면 0 → 비활성(기존 동작 유지). 지면 로봇 z-drift 억제(주행 중 상시 Vz→0).
    USE_VERTICAL_VEL = fsSettings["use_vertical_vel"].empty() ? 0 : (int)fsSettings["use_vertical_vel"];
    if (USE_VERTICAL_VEL)
    {
        VERTICAL_VEL_WEIGHT = fsSettings["vertical_vel_weight"].empty() ? 20.0 : (double)fsSettings["vertical_vel_weight"];
        RCLCPP_INFO(node->get_logger(), "USE_VERTICAL_VEL: 1, weight=%.2f (Vz->0)", VERTICAL_VEL_WEIGHT);
    }

    // ===== Ground-plane constraint (SW1-1837, VIW-Fusion plane_factor 이식) =====
    // use_plane 키 없으면 0 → 비활성(기존 동작 유지). z 위치+roll/pitch 자세를 추정 평면에 묶음.
    // ⚠️ USE_WHEEL 필요(평면 factor가 휠 extrinsic 사용). 단일 전역평면 가정(경사 미대응).
    USE_PLANE = fsSettings["use_plane"].empty() ? 0 : (int)fsSettings["use_plane"];
    if (USE_PLANE)
    {
        PITCH_N_INV = fsSettings["pitch_n_inv"].empty() ? 10.0 : (double)fsSettings["pitch_n_inv"];
        ROLL_N_INV  = fsSettings["roll_n_inv"].empty()  ? 10.0 : (double)fsSettings["roll_n_inv"];
        ZPW_N_INV   = fsSettings["zpw_n_inv"].empty()   ? 10.0 : (double)fsSettings["zpw_n_inv"];
        RCLCPP_INFO(node->get_logger(), "USE_PLANE: 1, pitch/roll/zpw_n_inv=%.2f/%.2f/%.2f",
                    PITCH_N_INV, ROLL_N_INV, ZPW_N_INV);
    }

    // ===== Body-frame NHC (SW1-1837, planar-motion Level2) =====
    // use_body_nhc 키 없으면 0 → 비활성(기존 동작 유지). 바퀴 프레임 횡(vy)·수직(vz) 속도를 0으로
    // 상시 소프트 제약. 월드 vz(use_vertical_vel)와 달리 바디 기준이라 경사에서도 참(지형 강건).
    USE_BODY_NHC = fsSettings["use_body_nhc"].empty() ? 0 : (int)fsSettings["use_body_nhc"];
    if (USE_BODY_NHC)
    {
        NHC_Y_WEIGHT = fsSettings["nhc_y_weight"].empty() ? 300.0 : (double)fsSettings["nhc_y_weight"];
        NHC_Z_WEIGHT = fsSettings["nhc_z_weight"].empty() ? 300.0 : (double)fsSettings["nhc_z_weight"];
        RCLCPP_INFO(node->get_logger(), "USE_BODY_NHC: 1, w_y/w_z=%.2f/%.2f (body vy,vz->0)",
                    NHC_Y_WEIGHT, NHC_Z_WEIGHT);
    }

    // ===== Wheel velocity outlier 게이팅 (SW1-1837) =====
    // 키 없으면 0 → 비활성(기존 동작 유지). 임계는 두 bag(_old/_base) 분포의 골 기반 통계값.
    //   실제 주행 본체: |v|≤0.57 m/s, |w|≤~3.0 rad/s / 글리치: |v|>1.0, |w|>5.0 → 그 사이 골.
    USE_WHEEL_VEL_GATE = fsSettings["use_wheel_vel_gate"].empty()
                             ? 0 : (int)fsSettings["use_wheel_vel_gate"];
    if (USE_WHEEL_VEL_GATE)
    {
        WHEEL_VEL_MAX = fsSettings["wheel_vel_max"].empty() ? 0.8 : (double)fsSettings["wheel_vel_max"];
        WHEEL_GYR_MAX = fsSettings["wheel_gyr_max"].empty() ? 4.0 : (double)fsSettings["wheel_gyr_max"];
        RCLCPP_INFO(node->get_logger(), "USE_WHEEL_VEL_GATE: 1, vel_max=%.2f m/s gyr_max=%.2f rad/s",
                    WHEEL_VEL_MAX, WHEEL_GYR_MAX);
    }

    // ===== [SW1-1837] 이벤트 게이팅 (Phase 1: 다리각) =====
    // 키 없으면 0 → 비활성(기존 동작 유지). 근거·설계는 doc/EVENT_GATING.md 참조.
    USE_EVENT_GATING = fsSettings["use_event_gating"].empty()
                           ? 0 : (int)fsSettings["use_event_gating"];
    if (USE_EVENT_GATING)
    {
        GATE_LEG          = fsSettings["gate_leg"].empty() ? 1 : (int)fsSettings["gate_leg"];
        LEG_POS_MIN       = fsSettings["leg_pos_min"].empty() ? 0.03 : (double)fsSettings["leg_pos_min"];
        LEG_RATE_MIN      = fsSettings["leg_rate_min"].empty() ? 0.05 : (double)fsSettings["leg_rate_min"];
        LEG_CMD_POS_MIN   = fsSettings["leg_cmd_pos_min"].empty() ? 0.02 : (double)fsSettings["leg_cmd_pos_min"];
        LEG_PRE_MARGIN    = fsSettings["leg_pre_margin"].empty() ? 0.3 : (double)fsSettings["leg_pre_margin"];
        LEG_POST_MARGIN   = fsSettings["leg_post_margin"].empty() ? 0.5 : (double)fsSettings["leg_post_margin"];
        GATE_MAX_DURATION = fsSettings["gate_max_duration"].empty() ? 2.0 : (double)fsSettings["gate_max_duration"];
        // [SW1-1866 08-03] 다리 이벤트 중 휠 factor 유지 — 3-bag 판별로 skip 은퇴.
        //   키 없으면 1 = factor 유지(신규 기본). 0 = 하드 skip(구동작 롤백)
        KEEP_WHEEL_IN_LEG_EVENT = fsSettings["keep_wheel_mode_in_leg_event"].empty()
                                      ? 1 : (int)fsSettings["keep_wheel_mode_in_leg_event"];
        if (fsSettings["leg_state_topic"].empty()) LEG_STATE_TOPIC = "/joint_states";
        else fsSettings["leg_state_topic"] >> LEG_STATE_TOPIC;
        if (fsSettings["leg_cmd_topic_l"].empty()) LEG_CMD_TOPIC_L = "/edie/l_leg_position_controller/command";
        else fsSettings["leg_cmd_topic_l"] >> LEG_CMD_TOPIC_L;
        if (fsSettings["leg_cmd_topic_r"].empty()) LEG_CMD_TOPIC_R = "/edie/r_leg_position_controller/command";
        else fsSettings["leg_cmd_topic_r"] >> LEG_CMD_TOPIC_R;
        RCLCPP_INFO(node->get_logger(),
                    "USE_EVENT_GATING: 1 (gate_leg=%d, rate>%.3f rad/s, margin -%.1f/+%.1f s, "
                    "cap %.1f s, keep_wheel=%d[1=factor 유지 0=skip 롤백])",
                    GATE_LEG, LEG_RATE_MIN, LEG_PRE_MARGIN, LEG_POST_MARGIN, GATE_MAX_DURATION,
                    KEEP_WHEEL_IN_LEG_EVENT);
    }

    // ===== 고속 회전 비전 게이팅 (SW1-1837) =====
    USE_YAW_GATING = fsSettings["use_yaw_gating"].empty() ? 0 : (int)fsSettings["use_yaw_gating"];
    if (USE_YAW_GATING)
    {
        YAW_GATE_GYR_THRESH = fsSettings["yaw_gate_gyr_thresh"].empty()
                                  ? 0.6 : (double)fsSettings["yaw_gate_gyr_thresh"];
        RCLCPP_INFO(node->get_logger(),
                    "USE_YAW_GATING: 1 (프레임 평균 |w-Bg| > %.3f rad/s (%.0f deg/s) 시 비전 관측 skip)",
                    YAW_GATE_GYR_THRESH, YAW_GATE_GYR_THRESH * 180.0 / M_PI);
    }

    // ===== 스핀 중 accel 신뢰 강등 (SW1-1837) =====
    USE_SPIN_ACC_DEWEIGHT = fsSettings["use_spin_acc_deweight"].empty()
                                ? 0 : (int)fsSettings["use_spin_acc_deweight"];
    if (USE_SPIN_ACC_DEWEIGHT)
    {
        SPIN_ACC_DEWEIGHT_GYR_THRESH = fsSettings["spin_acc_deweight_gyr_thresh"].empty()
                                           ? 0.6 : (double)fsSettings["spin_acc_deweight_gyr_thresh"];
        SPIN_ACC_DEWEIGHT_FACTOR = fsSettings["spin_acc_deweight_factor"].empty()
                                       ? 10.0 : (double)fsSettings["spin_acc_deweight_factor"];
        RCLCPP_INFO(node->get_logger(),
                    "USE_SPIN_ACC_DEWEIGHT: 1 (샘플 |w-Bg| > %.3f rad/s (%.0f deg/s) 시 acc 노이즈 x%.1f)",
                    SPIN_ACC_DEWEIGHT_GYR_THRESH,
                    SPIN_ACC_DEWEIGHT_GYR_THRESH * 180.0 / M_PI, SPIN_ACC_DEWEIGHT_FACTOR);
    }

    // ===== Bg_z 잠금 (SW1-1837) =====
    USE_BGZ_LOCK = fsSettings["use_bgz_lock"].empty() ? 0 : (int)fsSettings["use_bgz_lock"];
    if (USE_BGZ_LOCK)
    {
        BGZ_LOCK_DELAY = fsSettings["bgz_lock_delay"].empty()
                             ? 3.0 : (double)fsSettings["bgz_lock_delay"];
        BGZ_LOCK_STILL_SEC = fsSettings["bgz_lock_still_sec"].empty()
                                 ? 2.0 : (double)fsSettings["bgz_lock_still_sec"];
        BGZ_LOCK_STAB_MAX = fsSettings["bgz_lock_stab_max"].empty()
                                ? 2e-4 : (double)fsSettings["bgz_lock_stab_max"];
        BGZ_LOCK_FALLBACK_SEC = fsSettings["bgz_lock_fallback_sec"].empty()
                                    ? 8.0 : (double)fsSettings["bgz_lock_fallback_sec"];
        BGZ_LOCK_MAX = fsSettings["bgz_lock_max"].empty()
                           ? 0.001 : (double)fsSettings["bgz_lock_max"];
        BGZ_RELOCK_DELTA = fsSettings["bgz_relock_delta"].empty()
                               ? 5e-4 : (double)fsSettings["bgz_relock_delta"];
        BGZ_RELOCK_WIN_SEC = fsSettings["bgz_relock_win_sec"].empty()
                                 ? 10.0 : (double)fsSettings["bgz_relock_win_sec"];
        BGZ_RELOCK_COOLDOWN = fsSettings["bgz_relock_cooldown"].empty()
                                  ? 10.0 : (double)fsSettings["bgz_relock_cooldown"];
        RCLCPP_INFO(node->get_logger(),
                    "USE_BGZ_LOCK: 1 (상태 기반: 최소 %.1fs + 정지 %.1fs 지속 + 변동폭<%.1e"
                    " (요동 세션은 정지 %.1fs 누적 시 중앙값 폴백) + |추정-정지실측|<%.4f"
                    " rad/s이면 Bg_z 고정, 실측 괴리>%.1e 시 재잠금)",
                    BGZ_LOCK_DELAY, BGZ_LOCK_STILL_SEC, BGZ_LOCK_STAB_MAX,
                    BGZ_LOCK_FALLBACK_SEC, BGZ_LOCK_MAX, BGZ_RELOCK_DELTA);
    }

    // ===== 고주기 body TF (SW1-1837) =====
    PUB_HF_BODY_TF =
        fsSettings["publish_hf_body_tf"].empty() ? 0 : (int)fsSettings["publish_hf_body_tf"];
    if (PUB_HF_BODY_TF)
    {
        // [SW1-1866 08-05] 발행률 스로틀 — 키 없으면 100Hz(기존 하드코딩 값) 유지.
        //   0 이하는 무의미(무제한 발행=IMU rate)라 100으로 폴백.
        HF_BODY_TF_RATE_HZ = fsSettings["hf_body_tf_rate_hz"].empty()
                                 ? 100.0 : (double)fsSettings["hf_body_tf_rate_hz"];
        if (HF_BODY_TF_RATE_HZ <= 0.0)
            HF_BODY_TF_RATE_HZ = 100.0;
        RCLCPP_INFO(node->get_logger(), "HF_BODY_TF_RATE: %.1f Hz", HF_BODY_TF_RATE_HZ);
        HF_BODY_TF_TAU = fsSettings["hf_body_tf_tau"].empty()
                             ? 0.05 : (double)fsSettings["hf_body_tf_tau"];
        RCLCPP_INFO(node->get_logger(),
                    "PUB_HF_BODY_TF: 1 (map->body TF를 IMU 전파 자세로 100Hz 송출, "
                    "저주기 송출 중단, 스무딩 tau=%.3fs)",
                    HF_BODY_TF_TAU);
    }

    // ===== rviz 비교용 VINS footprint TF (SW1-1837) =====
    PUB_VINS_FOOTPRINT_TF = fsSettings["publish_vins_footprint_tf"].empty()
                                ? 0 : (int)fsSettings["publish_vins_footprint_tf"];
    if (PUB_VINS_FOOTPRINT_TF)
        RCLCPP_INFO(node->get_logger(),
                    "PUB_VINS_FOOTPRINT_TF: 1 (body에 vins/base_link·vins/base_footprint "
                    "정적 TF 부착 — rviz서 bringup TF와 비교용)");

    // ===== 도킹 소비용 odom→base_vins TF =====
    PUB_ODOM_BASE_VINS_TF = fsSettings["publish_odom_base_vins_tf"].empty()
                                ? 0 : (int)fsSettings["publish_odom_base_vins_tf"];
    if (PUB_ODOM_BASE_VINS_TF)
        RCLCPP_INFO(node->get_logger(),
                    "PUB_ODOM_BASE_VINS_TF: 1 (odom→base_vins 발행 — 도킹 앵커·전파가 "
                    "같은 VINS 추정을 쓰도록 하는 간선. map→body와 묶음 발행)");

    // ===== 정지 상대운동 잠금 (SW1-1866) =====
    USE_STILL_MOTION_LOCK = fsSettings["use_still_motion_lock"].empty()
                                ? 0 : (int)fsSettings["use_still_motion_lock"];
    if (USE_STILL_MOTION_LOCK)
    {
        const double sp = fsSettings["still_lock_pos_sigma_m"].empty()
                              ? 0.002 : (double)fsSettings["still_lock_pos_sigma_m"];
        const double sy = fsSettings["still_lock_yaw_sigma_deg"].empty()
                              ? 0.1 : (double)fsSettings["still_lock_yaw_sigma_deg"];
        STILL_LOCK_POS_W = 1.0 / std::max(sp, 1e-6);
        STILL_LOCK_YAW_W = 1.0 / std::max(sy * M_PI / 180.0, 1e-9);
        RCLCPP_INFO(node->get_logger(),
                    "USE_STILL_MOTION_LOCK: 1 (정지 확정 인접 프레임 상대운동 잠금 — "
                    "σ_p=%.3fm σ_yaw=%.2f°)", sp, sy);
    }

    // ===== 게이지 슬라이드 가드 (SW1-1866) =====
    USE_GAUGE_SLIDE_GUARD = fsSettings["use_gauge_slide_guard"].empty()
                                ? 0 : (int)fsSettings["use_gauge_slide_guard"];
    if (USE_GAUGE_SLIDE_GUARD)
    {
        const double thresh_deg = fsSettings["yaw_slide_guard_thresh_deg"].empty()
                                      ? 3.0 : (double)fsSettings["yaw_slide_guard_thresh_deg"];
        YAW_SLIDE_GUARD_THRESH = thresh_deg * M_PI / 180.0;
        POS_SLIDE_GUARD_THRESH = fsSettings["pos_slide_guard_thresh_m"].empty()
                                     ? 0.05 : (double)fsSettings["pos_slide_guard_thresh_m"];
        const double still_deg = fsSettings["yaw_slide_guard_still_thresh_deg"].empty()
                                     ? 0.1 : (double)fsSettings["yaw_slide_guard_still_thresh_deg"];
        YAW_SLIDE_GUARD_STILL_THRESH = still_deg * M_PI / 180.0;
        POS_SLIDE_GUARD_STILL_THRESH = fsSettings["pos_slide_guard_still_thresh_m"].empty()
                                     ? 0.005 : (double)fsSettings["pos_slide_guard_still_thresh_m"];
        RCLCPP_INFO(node->get_logger(),
                    "USE_GAUGE_SLIDE_GUARD: 1 (solve 간 과거 프레임 이동 문턱 — 주행 "
                    "yaw>%.1f°/pos>%.2fm, 정지 확정 시 yaw>%.2f°/pos>%.3fm로 조임)",
                    thresh_deg, POS_SLIDE_GUARD_THRESH, still_deg, POS_SLIDE_GUARD_STILL_THRESH);
        // [07-31] 절제 에스컬레이션 — 실기 정지 폭주 2건(절제 7·15회 무효) 근거
        GUARD_ESCALATION_MAX = fsSettings["guard_escalation_max"].empty()
                                   ? 3 : (int)fsSettings["guard_escalation_max"];
        if (GUARD_ESCALATION_MAX > 0)
            RCLCPP_INFO(node->get_logger(),
                        "GUARD_ESCALATION_MAX: %d (정화 없는 연속 prior 절제 상한 — 도달 시 "
                        "조기 재초기화로 폭주 발행 차단)", GUARD_ESCALATION_MAX);
    }

    // ===== failureDetection 문턱 (SW1-1866 08-10, R1) =====
    //   ⚠️어떤 조건문 안에도 넣지 않는다 — 과거 정지 판정이 조건부 로드되는 남의 파라미터
    //   (GRAVITY_ALIGN_VEL_THRESH)를 재사용해 use_gravity_align:0이면 0으로 남아 판정이
    //   영구 불합격했던 버그가 있었다. 재초기화 판정은 항상 살아있어야 하므로 무조건 로드.
    //   키가 없으면 종전 하드코딩 값 → 구형 config 동작 불변.
    FAILURE_BA_MAX = fsSettings["failure_ba_max"].empty()
                         ? 2.5 : (double)fsSettings["failure_ba_max"];
    FAILURE_BG_MAX = fsSettings["failure_bg_max"].empty()
                         ? 1.0 : (double)fsSettings["failure_bg_max"];
    FAILURE_DP_MAX = fsSettings["failure_dp_max"].empty()
                         ? 5.0 : (double)fsSettings["failure_dp_max"];
    FAILURE_DZ_MAX = fsSettings["failure_dz_max"].empty()
                         ? 1.0 : (double)fsSettings["failure_dz_max"];
    RCLCPP_INFO(node->get_logger(),
                "FAILURE 문턱: Ba>%.2f m/s² / Bg>%.2f rad/s / Δp>%.2fm / Δz>%.2fm "
                "(Δp·Δz는 solve 간 델타라 누적 드리프트는 못 봄 — 누적 판정은 별도)",
                FAILURE_BA_MAX, FAILURE_BG_MAX, FAILURE_DP_MAX, FAILURE_DZ_MAX);

    // ===== 정지 창 누적 변위 가드 (SW1-1866, 07-30) =====
    //   per-solve 문턱은 속도 제한이라 문턱 이하 지속 압력의 총량을 못 막음(obs_v2 말미
    //   0.32m 실증). 상한 근거는 판정 기준 0.03m(정지 중 그 이상 이동은 어떤 경우에도
    //   거짓)이지 bag 분포가 아님.
    USE_STILL_CUM_GUARD = fsSettings["use_still_cum_guard"].empty()
                              ? 0 : (int)fsSettings["use_still_cum_guard"];
    if (USE_STILL_CUM_GUARD)
    {
        STILL_CUM_XY_MAX = fsSettings["still_cum_xy_max_m"].empty()
                               ? 0.03 : (double)fsSettings["still_cum_xy_max_m"];
        // [07-31] z 래칫 가드 — 키 없으면 0(끔): 구형 config 동작 불변
        STILL_CUM_Z_MAX = fsSettings["still_cum_z_max_m"].empty()
                              ? 0.0 : (double)fsSettings["still_cum_z_max_m"];
        RCLCPP_INFO(node->get_logger(),
                    "USE_STILL_CUM_GUARD: 1 (정지 창 앵커 대비 누적 xy 상한 %.3fm — "
                    "문턱 이하 지속 병진 누설의 총량 유계 / z 래칫 상한 %.3fm%s)",
                    STILL_CUM_XY_MAX, STILL_CUM_Z_MAX,
                    STILL_CUM_Z_MAX > 0.0 ? ", 앵커 창 간 계승" : "=끔");
    }

    // ===== 휠 회전 잔차 주변화 (SW1-1837) =====
    WHEEL_ROT_MARGINALIZE = fsSettings["wheel_rot_marginalize"].empty()
                                ? 0 : (int)fsSettings["wheel_rot_marginalize"];
    if (WHEEL_ROT_MARGINALIZE)
        RCLCPP_INFO(node->get_logger(),
                    "WHEEL_ROT_MARGINALIZE: 1 (휠 factor 회전 잔차 주변화, 위치 3x3만 제약)");

    // ===== 정지 yaw 래칫 가드 (SW1-1866 08-04) — 키 없으면 비활성 =====
    STILL_CUM_YAW_MAX_DEG = fsSettings["still_cum_yaw_max_deg"].empty()
                                ? 0.0 : (double)fsSettings["still_cum_yaw_max_deg"];
    if (STILL_CUM_YAW_MAX_DEG > 0.0)
        RCLCPP_INFO(node->get_logger(),
                    "STILL_CUM_YAW_GUARD: 정지 창 누적 yaw 상한 %.2fdeg",
                    STILL_CUM_YAW_MAX_DEG);

    // ===== 재초기화 pose 시드 계승 (reboot-pose-seed) — 키 없으면 비활성 =====
    USE_REBOOT_POSE_SEED = fsSettings["use_reboot_pose_seed"].empty()
                               ? 0 : (int)fsSettings["use_reboot_pose_seed"];

    // ===== 정지 중 발산 가드 (08-11) =====
    // ⚠️조건부 블록 '밖'에서 로드 — GRAVITY_ALIGN_VEL_THRESH가 use_gravity_align:1일 때만
    //   로드돼 0으로 남았던 함정(4차 A/B서 still 항상 불합격의 범인)과 같은 사고 방지.
    USE_STILL_DRIFT_GUARD = fsSettings["use_still_drift_guard"].empty()
                                ? 0 : (int)fsSettings["use_still_drift_guard"];
    if (!fsSettings["still_drift_max_m"].empty())
        STILL_DRIFT_MAX = (double)fsSettings["still_drift_max_m"];
    if (!fsSettings["still_drift_window_sec"].empty())
        STILL_DRIFT_WINDOW_SEC = (double)fsSettings["still_drift_window_sec"];
    if (!fsSettings["still_check_duration_sec"].empty())
        STILL_CHECK_DURATION_SEC = (double)fsSettings["still_check_duration_sec"];
    if (!fsSettings["still_check_xy_tol_m"].empty())
        STILL_CHECK_XY_TOL = (double)fsSettings["still_check_xy_tol_m"];
    if (!fsSettings["still_check_yaw_tol_deg"].empty())
        STILL_CHECK_YAW_TOL_DEG = (double)fsSettings["still_check_yaw_tol_deg"];
    if (!fsSettings["still_drift_consec_solves"].empty())
        STILL_DRIFT_CONSEC = (int)fsSettings["still_drift_consec_solves"];
    if (USE_STILL_DRIFT_GUARD)
    {
        // 설계 전제: 측정 창은 지속 정지 창보다 짧아야 한다(같으면 정지 진입 전환 구간을
        //   물어 v14서 3/3 오탐 재현됨). 어긋나면 조용히 틀리지 말고 시작 시 경고.
        if (STILL_DRIFT_WINDOW_SEC >= STILL_CHECK_DURATION_SEC)
            RCLCPP_WARN(node->get_logger(),
                        "STILL_DRIFT: win(%.2fs) >= still(%.2fs) — 정지 진입 전환 구간이 창에 "
                        "섞여 오탐 위험(v14 실증). win < still 로 설정 권장",
                        STILL_DRIFT_WINDOW_SEC, STILL_CHECK_DURATION_SEC);
        RCLCPP_INFO(node->get_logger(),
                    "STILL_DRIFT_GUARD: 1 (휠 %.1fs 정지(<%.0fmm·<%.1fdeg) 확정 중 VINS가 %.2fs 창에서 "
                    "%.0fmm 초과 이동이 연속 %d회면 발동)",   // ⚠️안내문에 이벤트 판별 문구를 넣지 말 것
                    //   — grep 집계가 안내문을 이벤트로 오집계한다(08-11 5회 재발한 함정).
                    STILL_CHECK_DURATION_SEC, STILL_CHECK_XY_TOL * 1000.0,
                    STILL_CHECK_YAW_TOL_DEG, STILL_DRIFT_WINDOW_SEC, STILL_DRIFT_MAX * 1000.0,
                    STILL_DRIFT_CONSEC);
    }
    USE_OUTPUT_MAP_ANCHOR = fsSettings["use_output_map_anchor"].empty()
                                ? 0 : (int)fsSettings["use_output_map_anchor"];
    if (USE_OUTPUT_MAP_ANCHOR)
        RCLCPP_INFO(node->get_logger(),
                    "OUTPUT_MAP_ANCHOR: 1 (map→odom 핀 ∘ init 휠 스냅샷을 발행단 합성)");
    if (USE_REBOOT_POSE_SEED)
        RCLCPP_INFO(node->get_logger(),
                    "REBOOT_POSE_SEED: 1 (재초기화 시 마지막 건전 pose 시드 계승)");

    // ===== init/출발 워밍업 게이트 (SW1-1866) — 키 없으면 비활성(기존 동작) =====
    WARMUP_GATE_STILL_SAMPLES = fsSettings["warmup_gate_still_samples"].empty()
                                    ? 0 : (int)fsSettings["warmup_gate_still_samples"];
    WARMUP_GATE_IMU_SAMPLES = fsSettings["warmup_gate_imu_samples"].empty()
                                  ? 100 : (int)fsSettings["warmup_gate_imu_samples"];
    WARMUP_GATE_MOVING_SAMPLES = fsSettings["warmup_gate_moving_samples"].empty()
                                     ? 100 : (int)fsSettings["warmup_gate_moving_samples"];
    WARMUP_GATE_BUDGET_SAMPLES = fsSettings["warmup_gate_budget_samples"].empty()
                                     ? 3000 : (int)fsSettings["warmup_gate_budget_samples"];
    if (WARMUP_GATE_STILL_SAMPLES > 0)
        RCLCPP_INFO(node->get_logger(),
                    "WARMUP_GATE: 정지 실증 %d + IMU 표본 %d, 폴백(주행 %d / 예산 %d)",
                    WARMUP_GATE_STILL_SAMPLES, WARMUP_GATE_IMU_SAMPLES,
                    WARMUP_GATE_MOVING_SAMPLES, WARMUP_GATE_BUDGET_SAMPLES);

    fsSettings.release();
}
