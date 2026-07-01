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

// ===== Accelerometer-bias prior (SW1-1836) =====
int    USE_ACC_BIAS_PRIOR;
double ACC_BIAS_PRIOR_W_XY, ACC_BIAS_PRIOR_W_Z;

// ===== Vertical-velocity soft constraint (SW1-1837, planar-motion Level1) =====
int    USE_VERTICAL_VEL;
double VERTICAL_VEL_WEIGHT;

// ===== Wheel velocity outlier 게이팅 (SW1-1837) =====
int    USE_WHEEL_VEL_GATE;
double WHEEL_VEL_MAX, WHEEL_GYR_MAX;

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

    fsSettings.release();
}
