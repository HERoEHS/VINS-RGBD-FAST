#include "estimator.h"
#include "../utility/visualization.h"
#include "../utility/reboot_seed.h"
#include "../utility/hf_predict.h"
#include "../factor/zero_velocity_factor.h"
#include "../factor/still_motion_factor.h"
#include "../factor/acc_bias_prior_factor.h"
#include "../factor/vertical_velocity_factor.h"
#include "../factor/plane_factor.h"
#include "../factor/body_nhc_factor.h"
#include "../factor/gravity_align_factor.h"
#include "../utility/gravity_window_realign.h"
#include "../utility/yaw_slide_guard.h"
#include "../utility/yaw_gating.h"
#include <map>
#include "../utility/bgz_lock.h"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <iterator>
#include <string>
#include <utility>

Estimator::Estimator() : f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info   = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td                            = TD;
    g                             = G;

    // ===== Wheel extrinsic/intrinsic 초기값 (Step1: 고정 calib) =====
    rio      = RIO;
    tio      = TIO;
    sx       = SX;
    sy       = SY;
    sw       = SW;
    td_wheel = TD_WHEEL;

    // ===== [SW1-1837] 이벤트 게이팅 detector 파라미터 배선 (config → detector) =====
    if (USE_EVENT_GATING)
    {
        LegEventDetector::Params lp;
        lp.pos_min      = LEG_POS_MIN;
        lp.rate_min     = LEG_RATE_MIN;
        lp.cmd_pos_min  = LEG_CMD_POS_MIN;
        lp.pre_margin   = LEG_PRE_MARGIN;
        lp.post_margin  = LEG_POST_MARGIN;
        lp.max_duration = GATE_MAX_DURATION;
        std::lock_guard<std::mutex> lk(m_leg_gate);
        leg_gate.setParams(lp);
    }

    // ===== [SW1-1866] 워밍업 게이트 파라미터 배선 (config → gate) =====
    {
        warmup_init_gate::Params wp;
        wp.still_samples  = WARMUP_GATE_STILL_SAMPLES;
        wp.imu_samples    = WARMUP_GATE_IMU_SAMPLES;
        wp.moving_samples = WARMUP_GATE_MOVING_SAMPLES;
        wp.budget_samples = WARMUP_GATE_BUDGET_SAMPLES;
        warmup_gate_.setParams(wp);
    }

    featureTracker.readIntrinsicParameter(CAM_NAMES);
    if (FISHEYE)
    {
        featureTracker.fisheye_mask = cv::imread(FISHEYE_MASK, 0);
        if (!featureTracker.fisheye_mask.data)
        {
            ROS_INFO("load mask fail");
            ROS_BREAK();
        }
        else
            ROS_INFO("load mask success");
    }
    featureTracker.initGridsDetector();
}

void Estimator::clearState()
{
    m_imu.lock();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_imu.unlock();

    // 휠 비동기 버퍼 초기화
    m_wheel.lock();
    while (!wheelVelBuf.empty())
        wheelVelBuf.pop();
    while (!wheelGyrBuf.empty())
        wheelGyrBuf.pop();
    m_wheel.unlock();

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;

        // 휠 preintegration/버퍼 초기화 (IMU 미러)
        dt_buf_wheel[i].clear();
        linear_velocity_buf_wheel[i].clear();
        angular_velocity_buf_wheel[i].clear();
        if (pre_integrations_wheel[i] != nullptr)
            delete pre_integrations_wheel[i];
        pre_integrations_wheel[i] = nullptr;

        // cl
        find_solved[i] = 0;
        // cl
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }
    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }
    first_imu = false, sum_of_back = 0;
    // [SW1-1837] Bg_z 잠금 상태 초기화 — 재시작 시 새로 정지·수렴 관찰부터
    bgz_locked_        = false;
    bgz_lock_tracker_  = bgz_lock::Tracker{};
    bgz_rest_          = bgz_lock::RestBias{};
    bgz_rest_t_        = 0.0;
    bgz_locked_val_    = 0.0;
    bgz_last_relock_t_ = -1.0e18;
    sum_of_front      = 0;
    frame_count       = 0;
    solver_flag       = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;

    openExEstimation = false;

    // 휠 상태 초기화
    first_wheel           = false;
    openExWheelEstimation = false;
    openIxEstimation      = false;
    prevTime_wheel        = -1;

    delete tmp_pre_integration;
    delete tmp_wheel_pre_integration;

    delete last_marginalization_info;

    tmp_pre_integration       = nullptr;
    tmp_wheel_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur       = false;
    relocalization_info = false;

    // [SW1-1837] 중력 재정렬 v2 래치 리셋 (failure reboot 시 이전 정지 상태 이월 방지)
    grav_realign_done   = false;
    ba_at_realign       = Vector3d::Zero();
    grav_realign_last_t = -1.0e18;

    // [SW1-1866] 게이지 슬라이드 가드 리셋 (재초기화 직후 재기저를 슬라이드로 오인 방지)
    yaw_guard_prev_stamp_   = -1.0;
    yaw_guard_prev_yaw_deg_ = 0.0;
    yaw_guard_prev_pos_     = Vector3d::Zero();
    yaw_guard_consec_       = 0;
    yaw_guard_last_warn_t_  = -1.0e18;
    guard_amputate_streak_  = 0;
    guard_escalation_fire_  = false;
    still_cum_streak_       = 0;
    still_cum_valid_        = false;
    still_cum_anchor_       = Vector3d::Zero();
    still_cum_last_warn_t_  = -1.0e18;
    still_cum_z_anchor_     = 0.0;
    still_cum_z_valid_      = false;
    anchor_history_valid_   = false;
    anchor_net_yaw_rad_     = 0.0;
    z_anchor_wheel_moved_   = false;
    still_cum_z_last_warn_t_ = -1.0e18;
    still_cum_yaw_anchor_deg_  = 0.0;
    still_cum_yaw_raw_net_rad_ = 0.0;
    still_cum_yaw_elapsed_     = 0.0;
    still_cum_yaw_last_warn_t_ = -1.0e18;
    // [reboot-pose-seed] 시드 '재료'는 세션 스코프 → 리셋. T_seed(seed_active_/R/P)·
    //   다리 적분(bridge_gyro_yaw_rad_)·캡처 스냅샷(seed_cap_*)·seed_pending_은
    //   의도적으로 리셋하지 않는다(Q6: 캡처→clearState→재init을 관통해야 함).
    clean_pose_t_     = -1.0;
    anchor_latch_t_   = -1.0;
    amputate_first_t_ = -1.0;
    // [08-11] 발산 가드 이력 — 새 세션의 pose는 옛 세션과 다른 원점이라 이어붙이면 안 된다.
    still_drift_hist_.clear();
    still_drift_consec_  = 0;
    still_drift_first_t_ = -1.0;

    // [SW1-1866] 워밍업 게이트 리셋 — 재초기화(에스컬레이션 reboot 포함) 시 게이트 재가동.
    //   ⓐ 처방의 핵심 경로: reboot 직후 다리 애니메이션 중이면 leg_stable=false로
    //   init이 보류된다(GUARD_RESET_PATH_CHECKLIST Q3: 병리 중 리셋의 게이트 우회 차단).
    warmup_gate_.reset();
    warmup_fallback_logged_ = false;
    warmup_realign_logged_  = false;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();

    latest_Q = Eigen::Quaterniond(1, 0, 0, 0);
    // [SW1-1872] 예측 경로 전용 직전 샘플도 함께 리셋 (재부팅 시 이전 에피소드 값 이월 방지)
    latest_acc_0 = Eigen::Vector3d::Zero();
    latest_gyr_0 = Eigen::Vector3d::Zero();

    init_imu = true;

    prevTime = -1;

    initFirstPoseFlag = false;
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration,
                           const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0     = linear_acceleration;
        gyr_0     = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] =
            new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        // if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int      j        = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr   = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc   = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;

    // [SW1-1837] Bg_z 잠금용 정지 실측 공급 — 반드시 IMU 주기(~380Hz)로.
    //   프레임 주기(~13Hz) 표본은 2s 창에 ~26개뿐이라 중앙값 표준오차 ~1.2e-3
    //   → 재잠금 문턱(5e-4)보다 커서 오발동·잠금값 오염(1차 회귀 실측 실패).
    //   IMU 주기면 ~760개 → ~2e-4로 문턱의 2.5σ 밖. 시간축은 dt 누적 의사시간
    //   (창 스팬 판정에만 쓰여 절대시각 불필요).
    // [SW1-1866 07-31] 앵커 계승용 물리 순회전 적분 — VINS yaw 추정은 다리 이벤트가
    //   미끄러뜨리는(yaw_slide) 오염 신호라 계승 판정에 부적격(계측 run 실증: 로봇
    //   무회전인데 dyaw>5°로 재래치 → 래칫 소각 실패). bias 보정 gyro z 적분이 참값.
    // ★dt 위생 가드 (SW1-1866 08-09 — 이 블록의 '모든' 누적기에 적용)
    //   세션 경계(첫 샘플·clearState 직후)의 dt는 초기화 전 기준시각과 빼져 epoch급
    //   (~1.8e9s)이 된다. IMU 주기(~2.6ms)의 수십 배를 넘으면 물리적 dt가 아니다.
    //   [고친 결함] 이 가드는 원래 bridge_gyro_yaw_rad_ 한 곳에만 있었고, 아래 3개는
    //   무방비였다. 실측 역산으로 확정: [CUM-GUARD] net=406410490.3deg = 7.09e6 rad,
    //   gyro 0.004rad/s로 나누면 1.77e9s = 정확히 epoch. 오염 결과는
    //     · anchor_net_yaw_rad_        → 첫 정지 창 xy 앵커 '계승' 판정이 항상 불가
    //     · still_cum_yaw_raw_net_rad_ → yaw 래칫 가드의 편차 기준 오염
    //     · still_cum_yaw_elapsed_     → 경과시간 ≈1.8e9초
    const bool dt_sane = (dt > 0.0 && dt < 0.1);
    if (dt_sane)
    {
        anchor_net_yaw_rad_ += (angular_velocity.z() - Bgs[frame_count].z()) * dt;
        // [SW1-1866 08-04] yaw 래칫 가드용 raw 적분 — bias 미차감(자기참조 절단, 차감은
        //   판정 시점에 정지 실측 중앙값으로). 재잠금(Bgs 교체)에 불변.
        still_cum_yaw_raw_net_rad_ += angular_velocity.z() * dt;
        still_cum_yaw_elapsed_ += dt;
        // [reboot-pose-seed] 다리 yaw 적분 — 상시·무리셋(clearState 생존). 시드 캡처~재init
        //   완료 구간의 실회전을 스냅샷 차분으로 복원한다. 수 초 구간이라 bias 오차 무시.
        bridge_gyro_yaw_rad_ += angular_velocity.z() * dt;
    }

    // [SW1-1866] 워밍업 게이트 표본 공급 — IMU 표본 1개당 1회(게이트 계약).
    //   입력은 기존 판정 재사용(R4): 휠 정지 atomic + 다리 이벤트 게이트 활성 여부.
    //   READY 도달 후에는 게이트 내부에서 no-op이라 상시 호출해도 비용 없음.
    warmup_gate_.onSample(last_wheel_speed_.load() < bgz_lock::kStillWheelMax,
                          !leg_gate_active_now_.load());

    if (USE_BGZ_LOCK && solver_flag == SolverFlag::NON_LINEAR)
    {
        bgz_rest_t_ += dt;
        // 정지 판정 3중: gyro(bias 보정)·추정 속도·휠 twist. 휠 조건이 준정지
        //   (느린 잔여 회전, gyro 문턱 통과) 오인을 구조적으로 차단한다.
        const bool still_now =
            (angular_velocity - Bgs[frame_count]).norm() < bgz_lock::kStillGyrRadps &&
            Vs[frame_count].norm() < bgz_lock::kStillVelMps &&
            last_wheel_speed_.load() < bgz_lock::kStillWheelMax;
        // 보관 창은 재잠금 창(10s) 기준 — 초기 잠금은 ready/median(2s)로 접미 판독
        bgz_rest_.update(bgz_rest_t_, angular_velocity.z(), still_now, BGZ_RELOCK_WIN_SEC);
    }
}

// 휠 오도메트리 preintegration (processIMU 미러, SW1-1829)
// linear_velocity = odom twist.linear, angular_velocity = odom twist.angular
void Estimator::processWheel(double t, double dt, const Vector3d &linear_velocity,
                            const Vector3d &angular_velocity)
{
    (void)t;  // 시간은 호출부에서 dt로 환산 — VIW 시그니처 유지를 위해 인자만 보존

    // [SW1-1837] 주의: wheel velocity 글리치 게이팅은 여기서 per-sample 드롭하지 않는다.
    //   per-sample 드롭은 preintegration의 dt 회계·연속성을 깨 오히려 발산 악화(_gt 5m→61m, A/B 확정).
    //   대신 optimization()의 WheelFactor 추가 지점에서 "글리치 포함 구간의 factor만 skip"한다
    //   (preintegration 자체는 손상시키지 않고 그 구간은 IMU/비전이 받침). [[edie-wheel-odom-glitch]]

    if (!first_wheel)
    {
        first_wheel = true;
        vel_0_wheel = linear_velocity;
        gyr_0_wheel = angular_velocity;
    }

    if (!pre_integrations_wheel[frame_count])
    {
        pre_integrations_wheel[frame_count] =
            new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel};
    }
    if (frame_count != 0)
    {
        pre_integrations_wheel[frame_count]->push_back(dt, linear_velocity, angular_velocity);
        tmp_wheel_pre_integration->push_back(dt, linear_velocity, angular_velocity);

        dt_buf_wheel[frame_count].push_back(dt);
        linear_velocity_buf_wheel[frame_count].push_back(linear_velocity);
        angular_velocity_buf_wheel[frame_count].push_back(angular_velocity);
    }
    vel_0_wheel = linear_velocity;
    gyr_0_wheel = angular_velocity;
}

void Estimator::processImage(map<int, Eigen::Matrix<double, 7, 1>> &image,
                             const std_msgs::msg::Header                &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    // FeaturePerFrame
    // FeaturePerId
    // feature
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = rclcpp::Time(header.stamp).seconds();

    if (USE_IMU)
    {
        double curTime = rclcpp::Time(header.stamp).seconds() + td;

        while (!IMUAvailable(curTime))
        {
            printf("waiting for imu ... \r");
            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }

        std::vector<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_vector;
        getIMUInterval(prevTime, curTime, imu_vector);
        if (!initFirstPoseFlag)
            initFirstIMUPose(imu_vector);
        for (size_t i = 0; i < imu_vector.size(); i++)
        {
            double dt;
            if (i == 0)
                dt = imu_vector[i].first - prevTime;
            else if (i == imu_vector.size() - 1)
                dt = curTime - imu_vector[i - 1].first;
            else
                dt = imu_vector[i].first - imu_vector[i - 1].first;
            processIMU(dt, imu_vector[i].second.first, imu_vector[i].second.second);
        }
        prevTime = curTime;
    }

    // ===== 휠 오도메트리 적분 (IMU 루프 미러, SW1-1829) =====
    if (USE_WHEEL)
    {
        // 휠 타임스탬프 정합: 이미지 시각 + td(카메라-IMU) − td_wheel(휠 오프셋)
        double curTime_w = rclcpp::Time(header.stamp).seconds() + td - td_wheel;

        while (!WheelAvailable(curTime_w))
        {
            printf("waiting for wheel ... \r");
            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }

        std::vector<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> wheel_vector;
        getWheelInterval(prevTime_wheel, curTime_w, wheel_vector);
        for (size_t i = 0; i < wheel_vector.size(); i++)
        {
            double dt;
            if (i == 0)
                dt = wheel_vector[i].first - prevTime_wheel;
            else if (i == wheel_vector.size() - 1)
                dt = curTime_w - wheel_vector[i - 1].first;
            else
                dt = wheel_vector[i].first - wheel_vector[i - 1].first;
            processWheel(wheel_vector[i].first, dt, wheel_vector[i].second.first,
                         wheel_vector[i].second.second);
        }
        prevTime_wheel = curTime_w;
    }

    ImageFrame imageframe(image, rclcpp::Time(header.stamp).seconds());
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(rclcpp::Time(header.stamp).seconds(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    // 휠 임시 preintegration도 다음 프레임 대비 재생성 (누수 방지 위해 기존 것 삭제)
    if (USE_WHEEL)
    {
        delete tmp_wheel_pre_integration;
        tmp_wheel_pre_integration =
            new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel};
    }

    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres =
                f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(
                    corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0]             = calib_ric;
                RIC[0]             = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    TicToc opt_time;
    if (solver_flag == INITIAL)
    {
        if (USE_IMU && !STATIC_INIT)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (rclcpp::Time(header.stamp).seconds() - initial_timestamp) > 0.1)
                {
                    // [SW1-1837] init scale을 IMU가 아니라 depth로 푼다.
                    //   기존 initialStructure()는 monocular SfM + IMU로 scale 추정 →
                    //   EDIE 지면 평면주행은 IMU 병진여기 부족 → scale 폭주(첫 위치 146m) → 발산.
                    //   initialStructureWithDepth()는 depth로 메트릭 scale 직접 + 저여기/정지 폴백 보유.
                    result            = initialStructureWithDepth();
                    initial_timestamp = rclcpp::Time(header.stamp).seconds();
                }
                // if init sfm success
                if (result)
                {
                    solver_flag = NON_LINEAR;
                    finalizeRebootSeed();  // [reboot-pose-seed] 재init 완료 — T_seed 확정(시드 없으면 no-op)
                    solveOdometry();
                    slideWindow();
                    f_manager.removeFailures();
                    ROS_INFO("Initialization finish!");
                    last_R  = Rs[WINDOW_SIZE];
                    last_P  = Ps[WINDOW_SIZE];
                    last_R0 = Rs[0];
                    last_P0 = Ps[0];
                }
                else
                    slideWindow();
            }
            else
                frame_count++;
        }
        else
        {
            f_manager.triangulateWithDepth(Ps, tic, ric);

            if (USE_IMU)
            {
                // [SW1-1866] 워밍업 게이트 관문 — 정지(휠)+다리각 안정의 연속 실증과
                //   IMU 표본 축적 전에는 static init을 보류한다(ⓐ 재초기화의 다리
                //   애니메이션 착지 차단). WAIT면 가장 오래된 프레임을 밀어내(오염 창
                //   폐기) 게이트가 열린 시점의 창이 최신 정지 데이터로 구성되게 한다.
                //   FALLBACK = 주행 실증/예산 소진 → 저신뢰 init 허용(무한 대기 금지,
                //   R2 하드 요건). 근거·검증은 doc/WARMUP_GATE_PROPOSAL.md.
                const bool warmup_hold =
                    WARMUP_GATE_STILL_SAMPLES > 0 &&
                    warmup_gate_.verdict() == warmup_init_gate::Verdict::WAIT;
                if (warmup_hold && frame_count == WINDOW_SIZE)
                {
                    static rclcpp::Clock warmup_clk;
                    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("vins_warmup_gate"), warmup_clk, 2000,
                                         "[WARMUP-GATE] init 보류 — 정지 실증 %d / 표본 %d (총 %d)",
                                         warmup_gate_.stillStreak(), warmup_gate_.imuCollected(),
                                         warmup_gate_.totalSamples());
                    slideWindow();
                }
                else if (frame_count == WINDOW_SIZE)
                {
                    if (WARMUP_GATE_STILL_SAMPLES > 0 && warmup_gate_.fellBack() &&
                        !warmup_fallback_logged_)
                    {
                        warmup_fallback_logged_ = true;
                        ROS_WARN("[WARMUP-GATE] 정지 실증 실패(주행 실증/예산 소진) — 저신뢰 init 진행");
                    }
                    int i = 0;
                    for (auto &frame_it : all_image_frame)
                    {
                        frame_it.second.R = Rs[i];
                        frame_it.second.T = Ps[i];
                        i++;
                    }
                    if (ESTIMATE_EXTRINSIC != 2)
                    {
                        solveGyroscopeBias(all_image_frame, Bgs);
                        for (int j = 0; j <= WINDOW_SIZE; j++)
                        {
                            pre_integrations[j]->repropagate(Vector3d::Zero(), Bgs[j]);
                        }
                        optimization();
                        updateLatestStates();
                        solver_flag = NON_LINEAR;
                        finalizeRebootSeed();  // [reboot-pose-seed] 재init 완료 — T_seed 확정(시드 없으면 no-op)
                        slideWindow();
                        ROS_INFO("Initialization finish!");
                        // [SW1-1837] VI 초기화 완료 → 지면평면 초기화 후 제약 활성
                        if (USE_PLANE) { initPlane(); openPlaneEstimation = true; }
                        last_R  = Rs[WINDOW_SIZE];
                        last_P  = Ps[WINDOW_SIZE];
                        last_R0 = Rs[0];
                        last_P0 = Ps[0];
                    }
                }
            }
            else
            {
                if (frame_count == WINDOW_SIZE)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    finalizeRebootSeed();  // [reboot-pose-seed] 재init 완료 — T_seed 확정(시드 없으면 no-op)
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
            }

            if (frame_count < WINDOW_SIZE)
            {
                frame_count++;
                int prev_frame   = frame_count - 1;
                Ps[frame_count]  = Ps[prev_frame];
                Vs[frame_count]  = Vs[prev_frame];
                Rs[frame_count]  = Rs[prev_frame];
                Bas[frame_count] = Bas[prev_frame];
                Bgs[frame_count] = Bgs[prev_frame];
            }
        }
    }
    else
    {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);

        f_manager.triangulateWithDepth(Ps, tic, ric);
        optimization();
        // [SW1-1837] 정지 시 중력 재정렬 v2 — 최적화·marg 완료 직후가 유일하게 안전한 삽입점:
        //   여기서 창 상태와 marg prior 선형화점을 같은 ΔR로 돌려야 이후의
        //   slideWindow(상대 pose만 사용)·updateLatestStates(보정된 상태 재독)가 자동 정합된다.
        if (USE_GRAVITY_ALIGN >= 2)  // 2=창 회전만 / 3=창 회전+Ba 동시 재설정
            gravityRealignWindow();
        // [SW1-1866] 게이지 슬라이드 가드 — 같은 삽입점(최적화·marg 완료 직후) 이유 동일.
        //   중력 재정렬 뒤 순서: 재정렬의 yaw 부작용은 2차 미소량(≤0.16°)이라 문턱(3°) 미달
        if (USE_GAUGE_SLIDE_GUARD)
            gaugeSlideGuard();
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        set<int> removeIndex;
        movingConsistencyCheck(removeIndex);
        if (SHOW_TRACK)
        {
            for (auto iter = image.begin(), iter_next = image.begin(); iter != image.end();
                 iter = iter_next)
            {
                ++iter_next;
                auto it = removeIndex.find(iter->first);

                if (it != removeIndex.end())
                {
                    image.erase(iter);
                }
            }
        }

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = true;
            // [reboot-pose-seed] clearState 전에 시드 캡처 — 순서가 성립 조건
            //   (앵커·정화 pose 등 재료가 clearState에서 지워지기 전 마지막 지점)
            if (USE_REBOOT_POSE_SEED)
                captureRebootSeed(Headers[frame_count]);
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.emplace_back(Ps[i]);

        last_R  = Rs[WINDOW_SIZE];
        last_P  = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }

    static double whole_opt_time = 0;
    // [SW1-1866 08-09] 표시 앵커 — 핀이 init보다 늦게 와도 그 시점 세션 pose로 잡는다.
    //   이미 잡혔거나 핀 미수신이면 no-op(비용 0).
    maybeCaptureDisplayAnchor();

    static size_t cnt_frame      = 0;
    ++cnt_frame;
    whole_opt_time += opt_time.toc();
    ROS_DEBUG("average opt costs: %f", whole_opt_time / cnt_frame);
}

/**
 * @brief   视觉的结构初始化
 * @Description 确保IMU有充分运动激励
 *              relativePose()找到具有足够视差的两帧,由F矩阵恢复R、t作为初始值
 *              sfm.construct() 全局纯视觉SFM 恢复滑动窗口帧的位姿
 *              visualInitialAlign()视觉惯性联合初始化
 * @return  bool true:初始化成功
 */
bool Estimator::initialStructure()
{
    // check imu observibility
    bool is_imu_excited = false;
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d                          sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end();
             frame_it++)
        {
            double   dt    = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g     = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end();
             frame_it++)
        {
            double   dt    = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            // cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));  // 标准差
        // ROS_WARN("IMU variation %f!", var);
        if (var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            // return false;
        }
        else
        {
            is_imu_excited = true;
        }
    }

    TicToc t_sfm;
    // global sfm
    Quaterniond        Q[frame_count + 1];
    Vector3d           T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int        imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id    = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.emplace_back(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()});
            tmp_feature.observation_depth.emplace_back(imu_j, it_per_frame.depth);
        }
        sfm_f.emplace_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int      l;
    //保证具有足够的视差,由F矩阵恢复Rt
    //第l帧是从第一帧开始到滑动窗口中第一个满足与当前帧的平均视差足够大的帧，会作为参考帧到下面的全局sfm使用
    //此处的relative_R，relative_T为当前帧到参考帧（第l帧）的坐标系变换Rt
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        // ROS_INFO("Not enough features!");
        return false;
    }

    //对窗口中每个图像帧求解sfm问题
    //得到所有图像帧相对于参考帧的姿态四元数Q、平移向量T和特征点坐标sfm_tracked_points。
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    // solve pnp for all frame
    //对于所有的图像帧，包括不在滑动窗口中的，提供初始的RT估计，然后solvePnP进行优化,得到每一帧的姿态
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator      it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R            = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T            = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        // points: map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            it             = sfm_tracked_points.find(feature_id);
            if (it != sfm_tracked_points.end())
            {
                Vector3d    world_pts = it->second;
                cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                pts_3_vector.push_back(pts_3);
                Vector2d    img_pts = id_pts.second.head<2>();
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        /**
         *bool cv::solvePnP(    求解pnp问题
         *   InputArray  objectPoints,   特征点的3D坐标数组
         *   InputArray  imagePoints,    特征点对应的图像坐标
         *   InputArray  cameraMatrix,   相机内参矩阵
         *   InputArray  distCoeffs,     失真系数的输入向量
         *   OutputArray     rvec,       旋转向量
         *   OutputArray     tvec,       平移向量
         *   bool    useExtrinsicGuess = false, 为真则使用提供的初始估计值
         *   int     flags = SOLVEPNP_ITERATIVE 采用LM优化
         *)
         */
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        //这里也同样需要将坐标变换矩阵转变成图像帧位姿，并转换为IMU坐标系的位姿
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp              = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }

    // Rs Ps ric init
    //进行视觉惯性联合初始化
    if (visualInitialAlignWithDepth())
    {
        if (!is_imu_excited)
        {
            // 利用加速度平均值估计Bas
            Vector3d sum_a(0, 0, 0);
            for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end();
                 frame_it++)
            {
                double   dt    = frame_it->second.pre_integration->sum_dt;
                Vector3d tmp_a = frame_it->second.pre_integration->delta_v / dt;
                sum_a += tmp_a;
            }
            Vector3d avg_a;
            avg_a = sum_a * 1.0 / ((int)all_image_frame.size() - 1);

            Vector3d tmp_Bas = avg_a - Utility::g2R(avg_a).inverse() * G;
            ROS_WARN_STREAM("accelerator bias initial calibration " << tmp_Bas.transpose());
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                Bas[i] = tmp_Bas;
            }
        }
        return true;
    }
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }
}

bool Estimator::initialStructureWithDepth()
{
    // check imu observibility
    bool is_imu_excited = false;

    map<double, ImageFrame>::iterator frame_it;
    Vector3d                          sum_a;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end();
         frame_it++)
    {
        double   dt    = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        sum_a += tmp_g;
    }
    Vector3d aver_g;
    aver_g     = sum_a * 1.0 / ((int)all_image_frame.size() - 1);
    double var = 0;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end();
         frame_it++)
    {
        double   dt    = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
    }
    var = sqrt(var / ((int)all_image_frame.size() - 1));  // 标准差
    // ROS_WARN("IMU variation %f!", var);
    if (var < 0.25)
    {
        ROS_INFO("IMU excitation not enouth!");
        // return false;
    }
    else
    {
        is_imu_excited = true;
    }

    if (visualInitialAlignWithDepth())
    {
        if (!is_imu_excited)
        {
            Vector3d tmp_Bas = aver_g - Utility::g2R(aver_g).inverse() * G;
            ROS_WARN_STREAM("accelerator bias initial calibration " << tmp_Bas.transpose());
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                Bas[i] = tmp_Bas;
            }
        }
        return true;
    }
    else
    {
        staticInitialAlignWithDepth();
        return true;
    }

    ROS_INFO("misalign visual structure with IMU");
    return false;
}

bool Estimator::visualInitialAlign()
{
    TicToc   t_g;
    VectorXd x;
    // solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result)
    {
        ROS_ERROR("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri                              = all_image_frame[Headers[i]].R;
        Vector3d Pi                              = all_image_frame[Headers[i]].T;
        Ps[i]                                    = Pi;
        Rs[i]                                    = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    // triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    // f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));
    f_manager.triangulateWithDepth(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    // ROS_DEBUG("the scale is %f\n", s);
    //  do repropagate here
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int                               kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // if (it_per_id.used_num < 4)
        //     continue;
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0  = Utility::g2R(g);
    double   yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0           = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g            = R0 * g;
    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

/**
 * @brief   视觉惯性联合初始化
 * @Description 陀螺仪的偏置校准(加速度偏置没有处理) 计算速度V[0:n] 重力g 尺度s
 *              更新了Bgs后，IMU测量量需要repropagate
 *              得到尺度s和重力g的方向后，需更新所有图像帧在世界坐标系下的Ps、Rs、Vs
 * @return  bool true：成功
 */
bool Estimator::staticInitialAlignWithDepth()
{
    // 利用加速度平均值估计Bgs, Bas, g
    map<double, ImageFrame>::iterator frame_it;
    Vector3d                          sum_a(0, 0, 0);
    Vector3d                          sum_w(0, 0, 0);
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end();
         frame_it++)
    {
        sum_a +=
            frame_it->second.pre_integration->delta_v / frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_w;
        for (auto &gyr_msg : frame_it->second.pre_integration->gyr_buf)
        {
            tmp_w += gyr_msg;
        }
        sum_w += tmp_w / frame_it->second.pre_integration->gyr_buf.size();
    }
    Vector3d avg_a   = sum_a * 1.0 / ((int)all_image_frame.size() - 1);
    Vector3d avg_w   = sum_w * 1.0 / ((int)all_image_frame.size() - 1);
    g                = avg_a.normalized() * G.z();
    Vector3d tmp_Bas = avg_a - g;

    // solveGyroscopeBias(all_image_frame, Bgs);
    ROS_WARN_STREAM("gyroscope bias initial calibration " << avg_w.transpose());
    ROS_WARN_STREAM("accelerator bias initial calibration " << tmp_Bas.transpose());

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        Bgs[i] = avg_w;
        Bas[i] = tmp_Bas;
    }

    //陀螺仪的偏置bgs改变，重新计算预积分
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Bas[i], Bgs[i]);
    }

    //通过将重力旋转到z轴上，得到世界坐标系与摄像机坐标系c0之间的旋转矩阵rot_diff
    Matrix3d R0  = Utility::g2R(g);
    double   yaw = Utility::R2ypr(R0).x();
    R0           = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g            = R0 * G;

    Matrix3d rot_diff = R0;
    //所有变量从参考坐标系c0旋转到世界坐标系w
    for (int i = 0; i <= frame_count; i++)
    {
        ROS_ERROR("%d farme's Ps is %f | %f | %f\n", i, Ps[i].x(), Ps[i].y(),
                  Ps[i].z());  // shan add
        ROS_ERROR("%d farme's Vs is %f | %f | %f\n", i, Vs[i].x(), Vs[i].y(), Vs[i].z());
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
        // Vs[i] = Vector3d(0, 0, 0);
    }
    ROS_DEBUG_STREAM("static g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

/**
 * @brief   视觉惯性联合初始化
 * @Description 陀螺仪的偏置校准(加速度偏置没有处理) 计算速度V[0:n] 重力g 尺度s
 *              更新了Bgs后，IMU测量量需要repropagate
 *              得到尺度s和重力g的方向后，需更新所有图像帧在世界坐标系下的Ps、Rs、Vs
 * @return  bool true：成功
 */
bool Estimator::visualInitialAlignWithDepth()
{
    TicToc   t_g;
    VectorXd x;

    // solve scale
    //计算陀螺仪偏置，尺度，重力加速度和速度
    solveGyroscopeBias(all_image_frame, Bgs);

    if (!LinearAlignmentWithDepth(all_image_frame, g, x))
    {
        ROS_ERROR("solve g failed!");
        return false;
    }

    // do repropagate here
    //陀螺仪的偏置bgs改变，重新计算预积分
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    // change state
    // 得到所有图像帧的位姿Ps、Rs，并将其置为关键帧
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri                              = all_image_frame[Headers[i]].R;
        Vector3d Pi                              = all_image_frame[Headers[i]].T;
        Ps[i]                                    = Pi;
        Rs[i]                                    = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    // do repropagate here
    //陀螺仪的偏置bgs改变，重新计算预积分
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    // ROS_ERROR("before %f | %f | %f\n", Ps[1].x(), Ps[1].y(), Ps[1].z());//shan
    // add 将Ps、Vs、depth尺度s缩放
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = Ps[i] - Rs[i] * TIC[0] - (Ps[0] - Rs[0] * TIC[0]);
    // ROS_ERROR("after  %f | %f | %f\n", Ps[1].x(), Ps[1].y(), Ps[1].z());//shan
    // add
    int                               kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    //通过将重力旋转到z轴上，得到世界坐标系与摄像机坐标系c0之间的旋转矩阵rot_diff
    Matrix3d R0  = Utility::g2R(g);
    double   yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0           = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g            = R0 * g;
    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    //所有变量从参考坐标系c0旋转到世界坐标系w
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

/**
 * @brief   判断两帧有足够视差30且内点数目大于12则可进行初始化，同时得到R和T
 * @Description    判断每帧到窗口最后一帧对应特征点的平均视差是否大于30
                solveRelativeRT()通过基础矩阵计算当前帧与第l帧之间的R和T,并判断内点数目是否足够
 * @param[out]   relative_R 当前帧到第l帧之间的旋转矩阵R
 * @param[out]   relative_T 当前帧到第l帧之间的平移向量T
 * @param[out]   L 保存滑动窗口中与当前帧满足初始化条件的那一帧
 * @return  bool 1:可以进行初始化;0:不满足初始化条件
*/

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with
    // newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        // corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        corres = f_manager.getCorrespondingWithDepth(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (auto &corre : corres)
            {
                Vector2d pts_0(corre.first(0) / corre.first(2), corre.first(1) / corre.first(2));
                Vector2d pts_1(corre.second(0) / corre.second(2),
                               corre.second(1) / corre.second(2));
                // Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                // Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax    = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 &&
                m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to "
                          "triangulate the whole structure",
                          average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulateWithDepth(Ps, tic, ric);
        //        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        optimization();
    }
}

// [SW1-1837] 정지 시 중력 재정렬 v2 — 창 전체 자세 보정 (use_gravity_align: 2)
//   soft factor(모드 1)의 실패 원인(최적화 안 줄다리기 → 자세 못 돌리고 xy 전가)을 구조로 회피:
//   최적화 '밖'에서 정지 확정 시 1회, 창 전체와 marg prior 선형화점을 같은 ΔR로 강체 회전.
//   원리·소각 근사 한계는 utility/gravity_window_realign.h 헤더 주석 참조.
void Estimator::gravityRealignWindow()
{
    // --- 정지 확정 = 최신 2프레임 모두 3중 판정(휠 정지 AND IMU 정온 AND 게이팅 제외) 통과 ---
    //   2프레임(~0.2s)이면 IMU 수십 샘플 → 중력 방향 측정에 충분(w1000 probe 발견 A: 짧은 정지 커버)
    constexpr double kGyrQuiet  = 0.05;  // [rad/s] Bg 차감 후 자이로 평균 노름 상한
    constexpr double kAccStdMax = 0.5;   // [m/s^2] acc 편차 RMS 상한 (28~29Hz 구조 공진 흡수 여유)
    Vector3d acc_sum   = Vector3d::Zero();
    size_t   acc_n     = 0;
    bool     confirmed = true;
    for (int i = frame_count - 1; i <= frame_count; i++)
    {
        if (i < 1 || !pre_integrations_wheel[i] || pre_integrations_wheel[i]->sum_dt < 1e-3 ||
            !pre_integrations[i] || pre_integrations[i]->acc_buf.size() < 5 ||
            isLegGated(Headers[i - 1], Headers[i]))  // 다리 이벤트 중 '휠 정지'는 몸체 정지 아님
        {
            confirmed = false;
            break;
        }
        const double ga_dt    = pre_integrations_wheel[i]->sum_dt;
        const double ga_v_avg = pre_integrations_wheel[i]->delta_p.norm() / ga_dt;
        const double ga_w_avg = 2.0 * std::acos(std::min(1.0, std::fabs(
                                    pre_integrations_wheel[i]->delta_q.w()))) / ga_dt;
        if (ga_v_avg >= GRAVITY_ALIGN_VEL_THRESH || ga_w_avg >= GRAVITY_ALIGN_GYR_THRESH)
        {
            confirmed = false;
            break;
        }
        const auto &accs = pre_integrations[i]->acc_buf;
        const auto &gyrs = pre_integrations[i]->gyr_buf;
        Vector3d a_mean = Vector3d::Zero(), g_mean = Vector3d::Zero();
        for (const auto &a : accs) a_mean += a;
        for (const auto &g : gyrs) g_mean += g;
        a_mean /= static_cast<double>(accs.size());
        g_mean /= static_cast<double>(gyrs.size());
        double a_var = 0.0;
        for (const auto &a : accs) a_var += (a - a_mean).squaredNorm();
        if ((g_mean - Bgs[i]).norm() > kGyrQuiet ||
            std::sqrt(a_var / static_cast<double>(accs.size())) > kAccStdMax)
        {
            confirmed = false;
            break;
        }
        acc_sum += a_mean * static_cast<double>(accs.size());
        acc_n   += accs.size();
    }

    if (!confirmed)
    {
        // 움직임 재개 → 래치 해제 + Ba 이동량 보고(검증 관문 ④: 보정이 Ba로 전가됐는지 관찰)
        if (grav_realign_done)
        {
            RCLCPP_INFO(rclcpp::get_logger("vins_gravity_realign"),
                        "[GRAV-REALIGN] release t=%.3f dBa=%.4f",
                        Headers[frame_count], (Bas[frame_count] - ba_at_realign).norm());
            grav_realign_done = false;
        }
        return;
    }
    if (grav_realign_done)  // 정지당 1회 — soft처럼 상시 인력을 걸지 않는다(스냅·복원력 회피)
        return;

    // 실측 '위' 방향 — 모드 2: acc 평균 − 추정 Ba (자기일관 표적).
    //   모드 3: raw acc 평균 — 모드 2 A/B 실증: (acc−Ba) 표적은 Ba가 흡수한 자세 오차
    //   (긴 정지 raw 기준 ~3°)를 <0.5°로 보아 보정을 못 건다 → raw를 표적으로 삼고
    //   아래에서 Ba를 새 자세와 정합하게 재설정한다(근거는 gravity_window_realign.h 주석).
    const Vector3d acc_mean = acc_sum / static_cast<double>(acc_n);
    const Vector3d u_meas   = (USE_GRAVITY_ALIGN == 3) ? acc_mean : acc_mean - Bas[frame_count];
    double         angle  = 0.0;
    const Matrix3d dR     = gravity_realign::computeDeltaR(u_meas, Rs[frame_count],
                                                           GRAVITY_ALIGN_MAX_ANGLE, &angle);
    if (angle < GRAVITY_ALIGN_MIN_ANGLE)
        return;  // 이미 정렬 — 미발동(래치도 안 걸어 장기 정지 중 드리프트 재평가 허용)
    if (Headers[frame_count] - grav_realign_last_t < GRAVITY_ALIGN_COOLDOWN)
        return;  // 쿨다운 — 짧은 정지 연쇄(모드 3 A/B: 7건/30s)가 만든 xy 국소 churn 방지

    // --- 창 전체 강체 회전: pivot=현재 위치(정지 중) → 현재 위치 불변, 과거 궤적만 기울임 ---
    const Vector3d pivot = Ps[frame_count];
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        Ps[i] = pivot + dR * (Ps[i] - pivot);
        Rs[i] = dR * Rs[i];
        Vs[i] = dR * Vs[i];
    }

    // --- (모드 3) Ba 동시 재설정: 새 자세와 정합하는 값으로 균일 이동 ---
    //   안 하면 IMU 잔차가 '옛 Ba' 기준으로 보정을 되돌리는 힘을 만든다(관문 ④의 복원력).
    //   균일 이동이라 인접 프레임 bias 랜덤워크 잔차는 불변. 이동량이 J_ba 선형 범위를
    //   넘을 수 있어 창 내 preintegration을 재전파(초기화 경로와 동일 관용구)로 정확화.
    Vector3d d_ba = Vector3d::Zero();
    if (USE_GRAVITY_ALIGN == 3)
    {
        d_ba = gravity_realign::computeConsistentBa(acc_mean, Rs[frame_count], G.z()) -
               Bas[frame_count];
        for (int i = 0; i <= WINDOW_SIZE; i++)
            Bas[i] += d_ba;
        for (int i = 1; i <= WINDOW_SIZE; i++)
            if (pre_integrations[i])
                pre_integrations[i]->repropagate(Bas[i], Bgs[i]);
    }

    // --- marg prior 선형화점도 같은 변환 — 생략하면 다음 solve가 보정을 되돌린다(리스크의 핵심) ---
    //   keep_block_data[k]와 parameter_blocks[k]는 같은 루프에서 채워져 순서 정렬 보장
    //   (marginalization_factor.cpp getParameterBlocks). size 7이 pose/extrinsic 공용이라
    //   크기가 아닌 '주소 동일성'으로 블록을 판별. 야코비안은 소각 근사로 미회전(헤더 참조).
    if (last_marginalization_info)
    {
        for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++)
        {
            double *addr = last_marginalization_parameter_blocks[k];
            double *data = last_marginalization_info->keep_block_data[k];
            for (int j = 0; j <= WINDOW_SIZE; j++)
            {
                if (addr == para_Pose[j])
                {
                    gravity_realign::rotatePoseBlock(data, dR, pivot);
                    break;
                }
                if (addr == para_SpeedBias[j])
                {
                    gravity_realign::rotateSpeedBiasBlock(data, dR, d_ba);
                    break;
                }
            }
        }
    }

    grav_realign_done   = true;
    ba_at_realign       = Bas[frame_count];
    grav_realign_last_t = Headers[frame_count];
    RCLCPP_INFO(rclcpp::get_logger("vins_gravity_realign"),
                "[GRAV-REALIGN] apply t=%.3f err=%.2fdeg cap=%.1fdeg |dBa|=%.4f Ba=[%.4f %.4f %.4f]",
                Headers[frame_count], angle * 180.0 / M_PI,
                GRAVITY_ALIGN_MAX_ANGLE * 180.0 / M_PI, d_ba.norm(),
                Bas[frame_count].x(), Bas[frame_count].y(), Bas[frame_count].z());
}

// [SW1-1866] 게이지 슬라이드 가드 — 동적 장애물 오염 폭주(정지 중 연속 가짜 회전·활주) 차단.
//   기전·검출·처치 원리는 utility/yaw_slide_guard.h 헤더 주석 참조. 요지:
//   - 검출: '같은 물리 프레임'의 추정이 solve 사이에 이동한 양. 정상 재선형화는 yaw
//     0.0x°·위치 mm, 슬라이드는 yaw 13~16°·위치 0.03~0.15m(실측). 실제 운동(스핀·주행)은
//     과거 프레임 추정을 안 움직이므로 오탐 없음(v8·v9 A/B 발동 0회). 시간 상수 없음.
//   - 처치: 창 전체+marg prior 선형화점을 검출량만큼 역변환(역회전+역병진) —
//     gravityRealignWindow와 동일 기계(강체 변환이라 창 내 상대 pose·재투영 잔차 불변).
//     yaw만 막으면 압력이 병진 게이지로 전이(A/B: xy 0.26~1.5m/s 활주) → 둘 다 봉쇄해야
//     게이지 자유 방향이 밀폐된다.
//   - 재발동: 오염 prior가 남아 있으면 매 solve 다시 밀고 가드가 다시 되돌림(출력은
//     문턱 내 유계). 창이 전진하며 prior가 교정된 상태 기반으로 재구축되면 자연 소멸.
void Estimator::gaugeSlideGuard()
{
    const double stamp_now = Headers[frame_count];

    // [SW1-1866 probe] 강체성 판별용 — 직전 solve(보정 후)의 창 내 전 프레임 yaw.
    //   probe 전용(환경변수 게이트), 단일 estimator 전제의 함수 static.
    static const bool                slide_multi_log = (std::getenv("VINS_SLIDE_DIST_LOG") != nullptr);
    static std::map<double, double>  slide_multi_prev;  // stamp → yaw[deg]

    // 직전 solve의 최신 프레임을 현재 창에서 스탬프로 재탐색.
    //   MARGIN_SECOND_NEW로 그 프레임이 창에서 대체된 경우 부재 → 이번 solve는 검출
    //   불가(기록만 갱신). 폭주는 매 solve 지속되므로 다음 keyframe solve서 즉시 잡힌다.
    int ref = -1;
    if (yaw_guard_prev_stamp_ > 0.0)
        for (int i = frame_count; i >= 0; i--)
            if (Headers[i] == yaw_guard_prev_stamp_)
            {
                ref = i;
                break;
            }

    if (ref >= 0)
    {
        // 정지 확정 = 외부 앵커(휠 정지+gyro 정온+다리 이벤트 아님, 중력 재정렬과 동일
        //   3중 검사). 정지면 'Δpose=0'이라는 독립 관측이 생기므로 문턱을 조여 pose를
        //   사실상 고정 — 퇴화 장면(동적 장애물 점령)에서 문턱 이하 배회(3차 A/B: prior
        //   절제 후 yaw +76~222° 배회, 평균 0.9°/solve)를 막는 유일한 앵커.
        bool   still      = false;
        int    still_fail = 0;  // 진단: 0=합격 1=전제조건 2=leg게이트 3=휠v 4=휠w 5=gyro
        double diag_v = -1.0, diag_w = -1.0, diag_dt = -1.0;
        if (!(frame_count >= 1 && pre_integrations_wheel[frame_count] &&
              pre_integrations_wheel[frame_count]->sum_dt > 1e-3 &&
              pre_integrations[frame_count] &&
              !pre_integrations[frame_count]->gyr_buf.empty()))
            still_fail = 1;
        else if (isLegGated(Headers[frame_count - 1], Headers[frame_count]))
            still_fail = 2;
        else
        {
            const double dt_w  = pre_integrations_wheel[frame_count]->sum_dt;
            const double v_avg = pre_integrations_wheel[frame_count]->delta_p.norm() / dt_w;
            const double w_avg = 2.0 * std::acos(std::min(1.0, std::fabs(
                                     pre_integrations_wheel[frame_count]->delta_q.w()))) / dt_w;
            diag_v  = v_avg;
            diag_w  = w_avg;
            diag_dt = dt_w;
            Vector3d g_mean = Vector3d::Zero();
            for (const auto &g : pre_integrations[frame_count]->gyr_buf)
                g_mean += g;
            g_mean /= static_cast<double>(pre_integrations[frame_count]->gyr_buf.size());
            // 가드 전용 정지 임계 — GRAVITY_ALIGN_*는 use_gravity_align:1일 때만 로드되어
            //   0으로 남는 함정(4차 A/B서 still 항상 불합격의 범인). 물리값은 ZUPT·재정렬과
            //   동일한 0.02 m/s / 0.02 rad/s.
            constexpr double kStillVelMax = 0.02;
            constexpr double kStillGyrMax = 0.02;
            if (v_avg >= kStillVelMax)
                still_fail = 3;
            else if (w_avg >= kStillGyrMax)
                still_fail = 4;
            else if ((g_mean - Bgs[frame_count]).norm() >= 0.05)
                still_fail = 5;
            else
                still = true;
        }
        const double yaw_thresh_deg =
            (still ? YAW_SLIDE_GUARD_STILL_THRESH : YAW_SLIDE_GUARD_THRESH) * 180.0 / M_PI;
        const double pos_thresh_m =
            still ? POS_SLIDE_GUARD_STILL_THRESH : POS_SLIDE_GUARD_THRESH;

        const double yaw_slide = yaw_slide_guard::wrappedDeltaDeg(
            Utility::R2ypr(Rs[ref]).x(), yaw_guard_prev_yaw_deg_);
        const bool rot_hit = yaw_slide_guard::isSlide(yaw_slide, yaw_thresh_deg);

        // [SW1-1866] 강체성 판별 — 역변환 '전' 창 내 전 프레임의 slide 동시 측정.
        //   강체(진짜 게이지 슬라이드)면 전 프레임 동일 이동=산포 0. 비강체(산포 큼)는
        //   'prior가 특정 프레임 하나를 당기는 중'의 서명(온셋 실측: 1프레임만 −15.3°,
        //   나머지 9개 제자리, 산포=슬라이드 크기). 비강체에 강체 역변환을 쓰면 죄 없는
        //   프레임들을 돌려 +33° 온셋 과도를 '우리가' 주입했음이 8차 판별로 확정 → 분기.
        double spread = 0.0;
        int    n_spread = 0;
        if (!slide_multi_prev.empty())
        {
            double mn = 1e9, mx = -1e9, sum = 0;
            for (int i = 0; i <= frame_count; i++)
            {
                auto it = slide_multi_prev.find(Headers[i]);
                if (it == slide_multi_prev.end())
                    continue;
                const double d = yaw_slide_guard::wrappedDeltaDeg(
                    Utility::R2ypr(Rs[i]).x(), it->second);
                mn = std::min(mn, d);
                mx = std::max(mx, d);
                sum += d;
                n_spread++;
            }
            if (n_spread >= 2)
            {
                spread = mx - mn;
                if (slide_multi_log)
                    RCLCPP_INFO(rclcpp::get_logger("vins_gauge_guard"),
                                "[SLIDE-MULTI] t=%.3f n=%d mean=%+.3f min=%+.3f max=%+.3f "
                                "spread=%.3f",
                                stamp_now, n_spread, sum / n_spread, mn, mx, spread);
            }
        }
        // 강체 판정: 산포가 슬라이드 크기의 1/3 이하(+미세 바닥 0.5°)면 강체.
        //   비율 기준 = bag 편향 시간상수 없는 상태 조건(강체성 붕괴 그 자체)
        const bool rigid = spread <= std::max(0.3 * std::fabs(yaw_slide), 0.5);
        // '이상+비강체' = prior의 단일 프레임 견인 확정 → 강체 역변환은 부적용(자해),
        //   주범(prior)을 즉시 절제(연속 5회 대기 불필요 — 산포가 이미 신원을 입증)
        const bool nonrigid_takeover =
            !rigid && n_spread >= 2 &&
            yaw_slide_guard::isSlide(yaw_slide, YAW_SLIDE_GUARD_THRESH * 180.0 / M_PI);
        if (nonrigid_takeover)
        {
            if (last_marginalization_info)
            {
                delete last_marginalization_info;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
            yaw_guard_consec_ = 0;
            yaw_guard_trigger_cnt_++;
            if (stamp_now - yaw_guard_last_warn_t_ > 1.0)
            {
                yaw_guard_last_warn_t_ = stamp_now;
                RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                            "[GAUGE-GUARD] t=%.3f 비강체 슬라이드(ref %+.2fdeg, 산포 %.2fdeg)"
                            " → 역변환 생략+prior 즉시 절제", stamp_now, yaw_slide, spread);
            }
            // [SW1-1866 07-31] 정화 없는 연속 절제 집계 — 절제로 못 끊는 오염 판정 입력
            guard_amputate_streak_++;
            if (amputate_first_t_ < 0.0)
                amputate_first_t_ = stamp_now;  // [reboot-pose-seed] 오염 에피소드 시작
            if (!guard_escalation_fire_ &&
                yaw_slide_guard::escalationReached(guard_amputate_streak_, GUARD_ESCALATION_MAX))
            {
                guard_escalation_fire_ = true;
                RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                            "[GAUGE-GUARD] t=%.3f 정화 없는 연속 절제 %d회 — 절제로 못 끊는 "
                            "오염 → 조기 재초기화 요청", stamp_now, guard_amputate_streak_);
            }
        }

        // 회전 역변환 (pivot=현재 위치 — 현재 출력의 위치 연속성 유지)
        //   비강체 접수 시엔 부적용(강체 가정 위반 = 죄 없는 프레임 오염)
        const bool     do_rot = rot_hit && !nonrigid_takeover;
        const Matrix3d dR     = do_rot ? yaw_slide_guard::counterRotation(yaw_slide)
                                       : Matrix3d::Identity();
        const Vector3d pivot = Ps[frame_count];
        if (do_rot)
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                Ps[i] = pivot + dR * (Ps[i] - pivot);
                Rs[i] = dR * Rs[i];
                Vs[i] = dR * Vs[i];
            }

        // 병진 슬라이드는 회전 역변환 '후' 잔여로 측정 — ref 프레임을 직전 solve 위치로 복원
        const Vector3d pos_slide = Ps[ref] - yaw_guard_prev_pos_;
        const bool     pos_hit   = yaw_slide_guard::isPosSlide(pos_slide, pos_thresh_m) &&
                                   !nonrigid_takeover;
        if (pos_hit)
            for (int i = 0; i <= WINDOW_SIZE; i++)
                Ps[i] -= pos_slide;

        // [SW1-1866 07-30] 정지 창 누적 변위 가드 — per-solve 문턱(속도 제한)의 보완.
        //   obs_v2 실증: 사람이 30s 서 있는 조건서 문턱(5mm) 이하 병진 슬라이드 p50
        //   2~3.5mm가 방향 일관되게 누적 → 32s에 xy 0.32m('문턱=누설률'의 병진판).
        //   대응 = 총량 제한: 정지 연속 확인 후 앵커 래치, 앵커 대비 누적 xy가 상한
        //   (기본 0.03m = 정지 창 판정 기준)을 넘으면 초과분만 경계로 환원(게이지
        //   방향이라 무비용, 경계 안 대역은 치유용 자유). 앵커 래치를 15 solve 지연
        //   시키는 이유: 창 진입 직전 오염으로 틀리게 놓인 상태를 치유가 정당하게
        //   크게 교정하는 과도(수 solve 내 완료, p50 1.7mm 계보)와 싸우지 않기 위함.
        Vector3d cum_corr = Vector3d::Zero();
        if (USE_STILL_CUM_GUARD)
        {
            if (still)
            {
                still_cum_streak_++;
                constexpr int kCumAnchorSettleSolves = 15;
                if (!still_cum_valid_ && still_cum_streak_ >= kCumAnchorSettleSolves)
                {
                    // [SW1-1866 07-31] 앵커는 정지 창 간 '계승'이 원칙 — 재래치만 하면
                    //   다리 이벤트 중 스텝(v13 실증: z +20~50mm/이벤트, x도 동방향
                    //   크리프)이 새 앵커에 구워져 래칫을 못 지운다. 계승 조건(상태):
                    //   휠 병진 없음 + 다리각 복귀(자세가 바뀌면 진짜 높이 변화라 신규).
                    //   xy는 추가로 '물리' 무회전 필요 — Ps는 IMU 위치라 제자리 회전만으로도
                    //   레버암(0.1056m)만큼 원호로 '실제' 이동. 판정 신호는 gyro 순적분
                    //   (bias 보정) — VINS yaw 추정은 다리 이벤트가 미끄러뜨리는 오염
                    //   신호라 부적격(계측 실증: 무회전인데 dyaw>5° 재래치). |순회전|<5°면
                    //   레버암 잔여 ≤9mm(<상한)로 무시 가능.
                    constexpr double kLegMatchRad  = 0.035;  // 다리 엔코더 2-LSB(1°=0.0175)
                    constexpr double kNetYawMaxRad = 5.0 * M_PI / 180.0;
                    const double ll = latest_leg_l_.load(), lr = latest_leg_r_.load();
                    const double yaw_now = Utility::R2ypr(Rs[frame_count]).x();
                    const bool base_inherit =
                        anchor_history_valid_ && !z_anchor_wheel_moved_.load() &&
                        latest_leg_valid_.load() &&
                        std::fabs(ll - z_anchor_leg_l_) < kLegMatchRad &&
                        std::fabs(lr - z_anchor_leg_r_) < kLegMatchRad;
                    const bool xy_inherit =
                        base_inherit && std::fabs(anchor_net_yaw_rad_) < kNetYawMaxRad;
                    // 진단 캡처 — 아래 갱신·플래그 리셋 '전' 판정 시점 값(계승 미발동 수사)
                    const int    diag_wheel_mv = z_anchor_wheel_moved_.load() ? 1 : 0;
                    const double diag_dl   = std::fabs(ll - z_anchor_leg_l_);
                    const double diag_dr   = std::fabs(lr - z_anchor_leg_r_);
                    const double diag_dyaw = std::fabs(
                        yaw_slide_guard::wrappedDeltaDeg(yaw_now, anchor_yaw_deg_));
                    const int    diag_hist = anchor_history_valid_ ? 1 : 0;
                    const double diag_net  = anchor_net_yaw_rad_ * 180.0 / M_PI;
                    if (!xy_inherit)
                    {
                        still_cum_anchor_ = Ps[frame_count];
                        anchor_yaw_deg_   = yaw_now;
                    }
                    if (!base_inherit)
                    {
                        still_cum_z_anchor_ = Ps[frame_count].z();
                        z_anchor_leg_l_     = ll;
                        z_anchor_leg_r_     = lr;
                    }
                    still_cum_valid_      = true;
                    still_cum_z_valid_    = true;  // 클램프 자체는 STILL_CUM_Z_MAX>0 게이트
                    // [SW1-1866 08-04] yaw 앵커는 무조건 신규 — 주행 구간의 실회전 탓에
                    //   창 간 계승 불가(계승 허용오차 5°가 yaw 문턱 0.3°보다 커서 오염됨).
                    //   raw 적분·경과시간 리셋과 같은 순간이라 편차 기준점이 정확히 동기.
                    still_cum_yaw_anchor_deg_    = yaw_now;
                    still_cum_yaw_raw_net_rad_   = 0.0;
                    still_cum_yaw_elapsed_       = 0.0;
                    anchor_latch_t_       = stamp_now;  // [reboot-pose-seed] Q4 자격 판정용
                    anchor_history_valid_ = true;
                    z_anchor_wheel_moved_.store(false);
                    anchor_net_yaw_rad_   = 0.0;  // 물리 순회전 적분 재시작(계승 판정 기준점)
                    // 진단: 신규 판정 시 어느 조건이 깨졌는지(판정 '시점' 캡처값)
                    RCLCPP_INFO(rclcpp::get_logger("vins_gauge_guard"),
                                "[CUM-GUARD] t=%.3f 앵커 xy %s / z %s "
                                "(hist=%d wheel_mv=%d legv=%d dl=%.3f dr=%.3f dyaw=%.1f "
                                "net=%.1f)",
                                stamp_now, xy_inherit ? "계승" : "신규",
                                base_inherit ? "계승" : "신규", diag_hist, diag_wheel_mv,
                                latest_leg_valid_.load() ? 1 : 0, diag_dl, diag_dr, diag_dyaw,
                                diag_net);
                }
                if (still_cum_valid_)
                {
                    cum_corr = yaw_slide_guard::cumClampCorrection(
                        Ps[frame_count] - still_cum_anchor_, STILL_CUM_XY_MAX);
                    if (cum_corr.norm() > 0.0)
                    {
                        for (int i = 0; i <= WINDOW_SIZE; i++)
                            Ps[i] += cum_corr;
                        still_cum_trigger_cnt_++;
                        if (stamp_now - still_cum_last_warn_t_ > 2.0)
                        {
                            still_cum_last_warn_t_ = stamp_now;
                            RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                                        "[CUM-GUARD] t=%.3f 정지 창 누적 xy 상한 도달 — "
                                        "%.3fm 환원 (누적 %ld회)",
                                        stamp_now, cum_corr.norm(), still_cum_trigger_cnt_);
                        }
                    }
                }
                // [SW1-1866 07-31] z 래칫 환원 — 정지 창 참값은 Δz=0. z 병진도 게이지
                //   (비전·휠 상대 제약, IMU 중력 잔차는 병진 불변) — 유일 예외인 plane은
                //   zpw 동반 이동으로 무비용 완성. 창·prior 이동은 cum_corr.z 경유(공통 경로).
                if (STILL_CUM_Z_MAX > 0.0 && still_cum_z_valid_)
                {
                    const double zc = yaw_slide_guard::cumClampZCorrection(
                        Ps[frame_count].z() - still_cum_z_anchor_, STILL_CUM_Z_MAX);
                    if (zc != 0.0)
                    {
                        for (int i = 0; i <= WINDOW_SIZE; i++)
                            Ps[i].z() += zc;
                        zpw -= zc;  // zpw = -바퀴 고도(initPlane 규약) → 반대 부호로 동행
                        cum_corr.z() += zc;  // marg prior 병진은 아래 공통 경로가 수행
                        still_cum_z_trigger_cnt_++;
                        if (stamp_now - still_cum_z_last_warn_t_ > 2.0)
                        {
                            still_cum_z_last_warn_t_ = stamp_now;
                            RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                                        "[CUM-GUARD-Z] t=%.3f 정지 창 z 래칫 %+.3fm 환원 "
                                        "(누적 %ld회)",
                                        stamp_now, -zc, still_cum_z_trigger_cnt_);
                        }
                    }
                }

                // [SW1-1866 08-04] yaw 래칫 환원 — v14 굽힘 타임라인 실증: 정지 창에서
                //   창 전체 yaw가 per-solve 절제 문턱 이하로 미세하게 도는 게이지 슬라이드
                //   (+1.72°/s@|w|=0, "계단 2"의 정체). 정지 잠금은 상대 제약이라 구성상
                //   무력(8단 소거+힌지 probe로 확정) → 총량 유계가 처방. 편차에서 물리
                //   순회전(gyro 적분)을 차감해 문턱 이하 실제 크리프 회전은 오탐하지 않음.
                //   역회전은 states-only(피벗=현재 위치라 xy·z 앵커 불변) — per-solve
                //   역회전(위 do_rot)과 동일 선례. prior가 계속 밀면 매 solve 초과분만
                //   잘려 총량 유계, 지속 견인은 기존 절제·에스컬레이션 관할.
                // 물리 회전 기준 = raw 적분 − 정지 실측 bias(rest 중앙값)×경과시간.
                //   Bgs 기반이면 잠금 잔차(슬라이드의 원인)가 기준까지 오염하는 자기참조
                //   ([YAW-DEV] 진단 실증: yaw와 net이 동행해 dev≈0) — bgz_lock이 절대
                //   가드 한계를 '정지 실측 기준'으로 푼 계보 그대로. rest 미축적 시
                //   ready() 게이트로 우아하게 보류(bgz_rest는 정지 중 IMU 주기 축적이라
                //   앵커 래치(15 solve≈1.5s) 시점엔 통상 충족).
                // ※ready() 게이트는 크래시 방어이기도 함: RestBias::median()은 빈 이력에서
                //   UB(back()) — ready()가 비어있지 않음을 보증(08-04 진단 크래시 실증).
                if (STILL_CUM_YAW_MAX_DEG > 0.0 && still_cum_valid_ &&
                    bgz_rest_.ready(1.5))
                {
                    const double net_phys_deg =
                        (still_cum_yaw_raw_net_rad_ -
                         bgz_rest_.median(BGZ_RELOCK_WIN_SEC) * still_cum_yaw_elapsed_) *
                        180.0 / M_PI;
                    const double dev_deg = yaw_slide_guard::cumYawDeviationDeg(
                        Utility::R2ypr(Rs[frame_count]).x(), still_cum_yaw_anchor_deg_,
                        net_phys_deg);
                    const double yc =
                        yaw_slide_guard::cumClampZCorrection(dev_deg, STILL_CUM_YAW_MAX_DEG);
                    if (yc != 0.0)
                    {
                        const Matrix3d dRy = yaw_slide_guard::counterRotation(-yc);
                        const Vector3d pv  = Ps[frame_count];
                        for (int i = 0; i <= WINDOW_SIZE; i++)
                        {
                            Ps[i] = pv + dRy * (Ps[i] - pv);
                            Rs[i] = dRy * Rs[i];
                            Vs[i] = dRy * Vs[i];
                        }
                        still_cum_yaw_trigger_cnt_++;
                        if (stamp_now - still_cum_yaw_last_warn_t_ > 2.0)
                        {
                            still_cum_yaw_last_warn_t_ = stamp_now;
                            RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                                        "[CUM-GUARD-YAW] t=%.3f 정지 창 yaw 래칫 %+.2fdeg 환원 "
                                        "(누적 %ld회)",
                                        stamp_now, yc, still_cum_yaw_trigger_cnt_);
                        }
                    }
                }
            }
            else
            {
                still_cum_streak_ = 0;
                still_cum_valid_  = false;
                // xy·z 앵커 값은 의도적으로 유지 — 정지 복귀 시 계승 판정의 후보(래칫 삭제 핵심)
            }
        }
        const bool cum_hit = cum_corr.norm() > 0.0;

        // [SW1-1866 08-11] 정지 중 발산 가드 — 재부팅 '전' 폭주 차단.
        //   [고친 구멍] failureDetection의 Δp/Δz는 solve 간 델타라 '천천히 크게' 벗어나는
        //   누적을 원리적으로 못 본다(이 파일 failureDetection 주석 + v16 실측: 발행 pose가
        //   40m 이탈 중인데 미발동, 단발 5m 점프가 나서야 발동). 그 결과 재부팅 전 폭주가
        //   정지창 오차의 최대 성분이 된다(실측 런별 441mm ~ 37,660mm).
        //   [신호] 휠이 확정 정지인 동안 VINS pose가 창 안에서 얼마나 움직였나. 휠은 외부
        //   노드라 VIO 오염에 면역이므로 '움직이지 않았다'는 독립 관측이 되고, 그때 VINS의
        //   이동은 전부 오차다. ※휠 변위를 빼는 형태(‖ΔVINS − Δ휠‖)와 결과가 전 항목 동일해
        //   (24런 실측) 프레임 변환 없는 ‖ΔVINS‖로 단순화했다 — 확정 정지 중 Δ휠이 무시 가능.
        //   [실측] 큰 폭주(>1m) 4/4 · 중간(0.3~1m) 6/6 검출, v15·v14 오탐 0/3+0/3.
        //   [채택 08-12] v16_play 64런 A/B(28런+36런, 양쪽 시드 ON) → **기본 1**.
        //   ⚠️사전 등록 합격선은 **두 번 다 미달**(최대오차 p=0.077 / p=0.339)이고,
        //     상시 비용이 있다 — 정지창 오차 중앙 0.152→0.188m(+24%, p=0.0002).
        //     켜는 근거는 꼬리 하나뿐이다: >1m 폭주 합산 9/32 → 0/32 (p=0.002).
        //   ⚠️가드를 더 깎아도 폭발 구간은 안 줄어든다 — 27초간 1.8mm로 서 있다가
        //     0.14초 만에 1.1m 튀고, 가드는 그 안에 이미 반응한다. 수확 한계.
        //   근거 전문·철회된 진단 3건·다음 단서는 vio_edie.yaml의 use_still_drift_guard
        //   주석에 있다(여기서 중복 금지).
        //   ⚠️USE_REBOOT_POSE_SEED와 한 세트 — 시드가 꺼지면 채택 근거가 함께 무효다.
        //   문턱은 STILL_CUM_XY_MAX 와 같은 값(30mm) — 정지 창에서 클램프가 유지하기로 한
        //   경계를 넘었다는 뜻이라 배수가 필요 없다. 지속 정지 요구가 핵심이었다: 0.5~1.05s면
        //   주행 중 '일시정지'가 섞여 v14에서 3/3 오탐(t=117s, 그 20초간 로봇은 2.77m 주행).
        //   [발동] 새 실패 판정을 만들지 않고 기존 에스컬레이션을 세운다 — 저쪽은 "절제로
        //   못 끊는 오염", 이쪽은 "클램프로 못 끊는 오염"이라 결말(조기 재초기화)이 같다.
        if (USE_STILL_DRIFT_GUARD && !guard_escalation_fire_)
        {
            still_drift_hist_.emplace_back(stamp_now, Ps[frame_count],
                                   Vector2d(latest_wheel_x_.load(), latest_wheel_y_.load()),
                                   latest_wheel_yaw_.load() * 180.0 / M_PI);
            // 지속 정지 판정에 필요한 만큼만 보관. 1.5는 **문턱이 아니라 버퍼 여유**다 —
            //   1보다 크기만 하면 거동이 같아서 yaml 키로 뺄 이유가 없다(튜닝 축 아님).
            constexpr double kHistKeepFactor = 1.5;
            while (still_drift_hist_.size() > 1 &&
                   stamp_now - std::get<0>(still_drift_hist_.front()) >
                       STILL_CHECK_DURATION_SEC * kHistKeepFactor)
                still_drift_hist_.pop_front();

            // 지속 정지 = 기준 시간 이상 뒤 표본이 존재하고, 그 사이 휠 변위가 허용치 미만
            const Vector3d *p_win = nullptr;   // 창(WIN) 시작 pose
            double t_win = -1.0;               // 그 pose의 시각 = 오염 시작 후보
            bool sustained = false;
            for (auto it = still_drift_hist_.rbegin(); it != still_drift_hist_.rend(); ++it)
            {
                const double age = stamp_now - std::get<0>(*it);
                if (!p_win && age >= STILL_DRIFT_WINDOW_SEC)
                {
                    p_win = &std::get<1>(*it);
                    t_win = std::get<0>(*it);
                }
                if (age >= STILL_CHECK_DURATION_SEC)
                {
                    // [08-11 수정] 병진만으로 정지를 판정하면 **제자리 회전이 통과한다**.
                    //   Ps는 IMU 위치라 제자리 회전만으로도 레버암(0.1056m)만큼 원호로
                    //   '실제' 이동한다(이 파일 앵커 계승 주석의 같은 근거). 상한 30mm는
                    //   회전 30/105.6 = 0.284rad = 16.3°에서 그냥 넘는다.
                    //   실기 실증(dv1_r1): 회전 전환 구간에서 5회 과발동, 그때 다리 yaw가
                    //   ±45~90°였다. 휠 yaw도 함께 게이트해야 한다(휠 w는 외부 관측).
                    const double dyaw = std::fabs(yaw_slide_guard::wrappedDeltaDeg(
                        latest_wheel_yaw_.load() * 180.0 / M_PI, std::get<3>(*it)));
                    //   ≤0 이면 yaw 검사 안 함(A/B 대조군). 이 저장소 관행과 동일 —
                    //   still_cum_z_max_m(0=끔)·still_cum_yaw_max_deg(<=0 비활성).
                    //   ⚠️`dyaw < 0`으로 두면 항상 거짓이 되어 가드 '전체'가 죽으니 분기 필수.
                    const bool yaw_ok =
                        (STILL_CHECK_YAW_TOL_DEG <= 0.0) || (dyaw < STILL_CHECK_YAW_TOL_DEG);
                    sustained = (Vector2d(latest_wheel_x_.load(), latest_wheel_y_.load()) -
                                 std::get<2>(*it)).norm() < STILL_CHECK_XY_TOL &&
                                yaw_ok;
                    break;
                }
            }
            if (sustained && p_win)
            {
                const double div = (Ps[frame_count] - *p_win).head<2>().norm();
                if (div > STILL_DRIFT_MAX)
                {
                    // [08-12] 오염 시작 시각을 남긴다 — 시드 분기 선택자가 이 재부팅을
                    //   '무절제=건강'으로 오판해 오염된 정화pose를 물지 않게 하기 위함
                    //   (근거는 reboot_seed.h contaminationOnset 주석).
                    //   기준은 **창 시작 시각**이다: 그 시점 pose가 비교 기준이었으니
                    //   "여기까지는 건강했다"고 말할 수 있는 마지막 시각이다.
                    //   ⚠️**미설정일 때만** 기록한다(amputate_first_t_와 같은 관행).
                    //     해제는 정화 solve 한 곳에서만 — 근거는 recordOnsetOnce 주석.
                    reboot_seed::recordOnsetOnce(still_drift_first_t_, t_win);
                    ++still_drift_consec_;
                }
                else
                    still_drift_consec_ = 0;   // 오염 시작 시각은 정화 solve에서만 해제
                if (still_drift_consec_ >= STILL_DRIFT_CONSEC)
                {
                    guard_escalation_fire_ = true;
                    RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                                "[STILL-DRIFT] t=%.3f 정지 확정인데 VINS가 %.3fs 창에서 %.3fm "
                                "이동(연속 %d회, 상한 %.3fm) — 클램프로 못 끊는 폭주 → "
                                "조기 재초기화 요청",
                                stamp_now, STILL_DRIFT_WINDOW_SEC, div, still_drift_consec_,
                                STILL_DRIFT_MAX);
                }
            }
            else
                still_drift_consec_ = 0;
        }

        // 진단: 문턱 무관 slide 분포(정상 마진 실측용, 환경변수 게이트·read-only)
        static const bool dist_log = (std::getenv("VINS_SLIDE_DIST_LOG") != nullptr);
        if (dist_log)
            RCLCPP_INFO(rclcpp::get_logger("vins_gauge_guard"),
                        "[SLIDE-DIST] t=%.3f yaw=%+.4fdeg pos=%.4fm still=%d fail=%d "
                        "v=%.4f w=%.4f dt=%.3f",
                        stamp_now, yaw_slide, pos_slide.norm(), still ? 1 : 0, still_fail,
                        diag_v, diag_w, diag_dt);

        // '진짜 이상' 판정은 주행 문턱 기준 — 정지 전면 고정(pin)의 미세 카운터(mm·0.0x°)가
        //   절제/경고를 오발시키지 않도록 분리(5차 A/B: 정지 pin 없인 문턱만큼 새고,
        //   pin의 매 solve 발동이 consec에 잡히면 정상 정차서 prior 절제 사고)
        const bool anomaly =
            !nonrigid_takeover &&
            (yaw_slide_guard::isSlide(yaw_slide, YAW_SLIDE_GUARD_THRESH * 180.0 / M_PI) ||
             yaw_slide_guard::isPosSlide(pos_slide, POS_SLIDE_GUARD_THRESH));

        if (do_rot || pos_hit || cum_hit)
        {
            // marg prior 선형화점도 동일 변환 — 생략하면 다음 solve가 역변환을 되돌린다
            //   (gravityRealignWindow와 동일 관용구·동일 근거)
            if (last_marginalization_info)
            {
                for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++)
                {
                    double *addr = last_marginalization_parameter_blocks[k];
                    double *data = last_marginalization_info->keep_block_data[k];
                    for (int j = 0; j <= WINDOW_SIZE; j++)
                    {
                        if (addr == para_Pose[j])
                        {
                            if (do_rot)
                                gravity_realign::rotatePoseBlock(data, dR, pivot);
                            if (pos_hit)
                                yaw_slide_guard::translatePoseBlock(data, -pos_slide);
                            if (cum_hit)
                                yaw_slide_guard::translatePoseBlock(data, cum_corr);
                            break;
                        }
                        if (addr == para_SpeedBias[j])
                        {
                            if (do_rot)
                                gravity_realign::rotateSpeedBiasBlock(data, dR);
                            break;  // 병진은 속도·bias 불변
                        }
                    }
                }
            }
            yaw_guard_trigger_cnt_++;
            // WARN·절제 카운트는 '진짜 이상'(주행 문턱 초과)만 — pin 미세 카운터는 조용히
            if (anomaly)
            {
                yaw_guard_consec_++;
                if (stamp_now - yaw_guard_last_warn_t_ > 1.0)
                {
                    yaw_guard_last_warn_t_ = stamp_now;
                    RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                                "[GAUGE-GUARD] t=%.3f yaw_slide=%+.2fdeg pos_slide=%.3fm "
                                "역변환 적용 (누적 %ld회)",
                                stamp_now, yaw_slide, pos_slide.norm(), yaw_guard_trigger_cnt_);
                }

                // 에스컬레이션: 연속 5 solve 이상 발동 = 일회성 교란이 아니라 '오염된
                //   prior의 지속 압력'으로 확정(정상 주행은 이상 발동 0회 — v8·v9 실측)
                //   → 오염원인 marg prior를 절제. 역변환만으로는 문턱 이하 누설이 남아
                //   활주가 계속되기 때문(2차 A/B: 1.4cm/solve). prior 부재는 일시적
                //   (다음 solve 마진화가 건강한 현재 상태로 재구축).
                constexpr int kAmputateAfterConsec = 5;
                if (yaw_guard_consec_ >= kAmputateAfterConsec)
                {
                    if (last_marginalization_info)
                    {
                        delete last_marginalization_info;
                        last_marginalization_info = nullptr;
                        last_marginalization_parameter_blocks.clear();
                    }
                    yaw_guard_consec_ = 0;
                    RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                                "[GAUGE-GUARD] t=%.3f 오염 지속(연속 %d solve) → marg prior "
                                "절제(압력원 제거, 다음 solve서 재구축)",
                                stamp_now, kAmputateAfterConsec);
                    // [SW1-1866 07-31] 정화 없는 연속 절제 집계 (비강체 분기와 동일 판정)
                    guard_amputate_streak_++;
                    if (amputate_first_t_ < 0.0)
                        amputate_first_t_ = stamp_now;  // [reboot-pose-seed] 에피소드 시작
                    if (!guard_escalation_fire_ &&
                        yaw_slide_guard::escalationReached(guard_amputate_streak_,
                                                           GUARD_ESCALATION_MAX))
                    {
                        guard_escalation_fire_ = true;
                        RCLCPP_WARN(rclcpp::get_logger("vins_gauge_guard"),
                                    "[GAUGE-GUARD] t=%.3f 정화 없는 연속 절제 %d회 — 절제로 "
                                    "못 끊는 오염 → 조기 재초기화 요청",
                                    stamp_now, guard_amputate_streak_);
                    }
                }
            }
        }
        if (!anomaly)
        {
            yaw_guard_consec_ = 0;  // 이상 없는 solve = 에피소드 종료
            // [SW1-1866 07-31] 정화 판정: 비강체 접수 solve는 anomaly 정의상 false지만
            //   절제가 일어난 solve라 정화가 아님 — 자기 리셋으로 에스컬레이션을 못
            //   세게 되는 자충을 막는다. cum_hit도 동일: 누적 가드가 환원 중인 solve는
            //   per-solve 슬라이드가 주행 문턱 아래여도 '개입 불필요'가 아니므로 정화
            //   아님(문턱 넘나드는 중간 강도 폭주의 발동 회피 봉쇄; 평시 mm급 cum
            //   발동은 카운터 0이라 무영향)
            if (!nonrigid_takeover && !cum_hit)
            {
                guard_amputate_streak_ = 0;
                amputate_first_t_      = -1.0;  // 에피소드 종료 — 다음 절제가 새 시작
                // [08-13 수정] 발산 가드의 오염 실증은 **가드 미발동일 때만** 정화 리셋.
                //   옛 판(08-12)은 무조건 지웠는데, 누적 드리프트는 per-solve 델타가 작아
                //   **가드가 발동한 바로 그 solve가 '정화'로 판정**된다 → 같은 solve에서
                //   ①오염 실증 소거 ②드리프트된 pose를 clean_P_로 저장 → captureRebootSeed가
                //   "오염 없음"으로 보고 그 오염 pose를 시드로 물었다(DIAG 실측 dg1_r2:
                //   clean_t=발동 시각, 착지 오차 0.4m급 — +24% 비용의 주범).
                //   발동 후엔 재부팅 → clearState가 전부 리셋하므로 들고 있어도 안전.
                if (!guard_escalation_fire_)
                    still_drift_first_t_ = -1.0;
                // [reboot-pose-seed] 2순위 시드 재료: 정화 solve의 pose 스냅샷.
                //   정화 정의는 179807f 수술본(자기참조 제거) 그대로 재사용 — 이
                //   순간의 상태는 '가드 개입이 불필요했던 건강한 solve'다.
                clean_pose_t_ = stamp_now;
                clean_P_      = Ps[frame_count];
                clean_yaw_    = Utility::R2ypr(Rs[frame_count]).x() * M_PI / 180.0;
            }
        }
    }

    yaw_guard_prev_stamp_   = stamp_now;
    yaw_guard_prev_yaw_deg_ = Utility::R2ypr(Rs[frame_count]).x();
    yaw_guard_prev_pos_     = Ps[frame_count];

    // [SW1-1866] 강체성 판별용 전 프레임 yaw 저장(보정 후 값 — 다음 solve 비교 기준).
    //   비강체 접수 분기의 판정 입력이라 상시 유지(11개 R2ypr/solve — 비용 무시 가능)
    slide_multi_prev.clear();
    for (int i = 0; i <= frame_count; i++)
        slide_multi_prev[Headers[i]] = Utility::R2ypr(Rs[i]).x();
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    // 휠 extrinsic(T_io)/intrinsic/td → ceres double 배열
    if (USE_WHEEL)
    {
        para_Ex_Pose_wheel[0][0] = tio.x();
        para_Ex_Pose_wheel[0][1] = tio.y();
        para_Ex_Pose_wheel[0][2] = tio.z();
        Quaterniond qio{rio};
        para_Ex_Pose_wheel[0][3] = qio.x();
        para_Ex_Pose_wheel[0][4] = qio.y();
        para_Ex_Pose_wheel[0][5] = qio.z();
        para_Ex_Pose_wheel[0][6] = qio.w();

        para_Ix_sx_wheel[0][0] = sx;
        para_Ix_sy_wheel[0][0] = sy;
        para_Ix_sw_wheel[0][0] = sw;

        para_Td_wheel[0][0] = td_wheel;
    }

    // 지면평면 파라미터 → ceres (SW1-1837)
    if (USE_PLANE)
    {
        Eigen::Quaterniond q_pw(rpw);
        para_plane_R[0][0] = q_pw.x();
        para_plane_R[0][1] = q_pw.y();
        para_plane_R[0][2] = q_pw.z();
        para_plane_R[0][3] = q_pw.w();
        para_plane_Z[0][0] = zpw;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

// 数据转换，vector2double的相反过程
// 同时这里为防止优化结果往零空间变化，会根据优化前后第一帧的位姿差进行修正。
void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0     = Utility::R2ypr(last_R0);
        origin_P0     = last_P0;
        failure_occur = false;
    }
    if (USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(
            Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5])
                .toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        // TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4],
                                           para_Pose[0][5])
                                   .toRotationMatrix()
                                   .transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = rot_diff *
                    Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
                        .normalized()
                        .toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                        para_Pose[i][1] - para_Pose[0][1],
                                        para_Pose[i][2] - para_Pose[0][2]) +
                    origin_P0;

            Vs[i] = rot_diff *
                    Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8]);
        }

        // relative info between two loop frame
        if (relocalization_info)
        {
            Matrix3d relo_r;
            Vector3d relo_t;
            relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5])
                                    .normalized()
                                    .toRotationMatrix();
            relo_t =
                rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0], relo_Pose[1] - para_Pose[0][1],
                                    relo_Pose[2] - para_Pose[0][2]) +
                origin_P0;
            double drift_correct_yaw;
            drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
            drift_correct_r   = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
            drift_correct_t   = prev_relo_t - drift_correct_r * relo_t;
            relo_relative_t   = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
            relo_relative_q   = relo_r.transpose() * Rs[relo_frame_local_index];
            relo_relative_yaw = Utility::normalizeAngle(
                Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
            // cout << "vins relo " << endl;
            // cout << "vins relative_t " << relo_relative_t.transpose() << endl;
            // cout << "vins relative_yaw " <<relo_relative_yaw << endl;
            relocalization_info = 0;
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
                        .normalized()
                        .toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }

        // relative info between two loop frame
        if (relocalization_info)
        {
            Matrix3d relo_r;
            Vector3d relo_t;
            relo_r = Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5])
                         .normalized()
                         .toRotationMatrix();
            relo_t = Vector3d(relo_Pose[0], relo_Pose[1], relo_Pose[2]);
            double drift_correct_yaw;
            drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
            drift_correct_r   = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
            drift_correct_t   = prev_relo_t - drift_correct_r * relo_t;
            relo_relative_t   = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
            relo_relative_q   = relo_r.transpose() * Rs[relo_frame_local_index];
            relo_relative_yaw = Utility::normalizeAngle(
                Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
            // cout << "vins relo " << endl;
            // cout << "vins relative_t " << relo_relative_t.transpose() << endl;
            // cout << "vins relative_yaw " <<relo_relative_yaw << endl;
            relocalization_info = false;
        }
    }
    if (USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3], para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5])
                         .normalized()
                         .toRotationMatrix();
        }
    }

    // 휠 extrinsic/intrinsic/td 읽기 (Step1 고정이면 값 불변 → no-op, Step2 추정 시 갱신)
    if (USE_WHEEL)
    {
        tio = Vector3d(para_Ex_Pose_wheel[0][0], para_Ex_Pose_wheel[0][1],
                       para_Ex_Pose_wheel[0][2]);
        rio = Quaterniond(para_Ex_Pose_wheel[0][6], para_Ex_Pose_wheel[0][3],
                          para_Ex_Pose_wheel[0][4], para_Ex_Pose_wheel[0][5])
                  .normalized()
                  .toRotationMatrix();
        sx       = para_Ix_sx_wheel[0][0];
        sy       = para_Ix_sy_wheel[0][0];
        sw       = para_Ix_sw_wheel[0][0];
        td_wheel = para_Td_wheel[0][0];
    }

    // ceres → 지면평면 파라미터 (SW1-1837)
    if (USE_PLANE)
    {
        rpw = Eigen::Quaterniond(para_plane_R[0][3], para_plane_R[0][0],
                                 para_plane_R[0][1], para_plane_R[0][2])
                  .normalized()
                  .toRotationMatrix();
        zpw = para_plane_Z[0][0];
        // // [SW1-1837] zpw 추적 로그 — 자유변수 zpw가 드리프트한 궤적 고도를 쫓아가는지 관측용.
        // //   평면 잔차가 0이려면 zpw ≈ -z_wheel(최신 프레임 바퀴 고도) → 두 값이 나란히 내려가면
        // //   'zpw 윈도우 재앵커'(전역 z 앵커 부재)의 직접 증거. 분석: 로그에서 [ZPW] grep 후 plot.
        // RCLCPP_INFO(rclcpp::get_logger("vins_plane"), "[ZPW] t=%.3f zpw=%.4f z_wheel=%.4f",
        //             Headers[frame_count], zpw,
        //             (Ps[frame_count] + Rs[frame_count] * tio)[2]);
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD && USE_IMU)
        td = para_Td[0][0];
}

// [SW1-1837] 지면평면 초기화 — 윈도 pose들 평균으로 평면 방향(rpw)·높이(zpw) 추정.
//   rpw_i = (Rs[i]·rio)^T = world→바퀴평면. 쿼터니언 평균(Markley) 후 yaw 제거(평면 yaw 미관측).
//   zpw = 바퀴 고도(-(rpw·wheel_pos).z) 평균. 지면 관측 없이 pose만으로 = 순수 상태 prior.
void Estimator::initPlane()
{
    const int cnt = frame_count;
    if (cnt <= 0)
        return;
    // ★[SW1-1837 수정] 평면 법선을 편향된 pose 자세에서 뽑지 않는다.
    //   VINS world z축은 이미 중력정렬됨 → 평평한 바닥의 법선 = world-up = e3.
    //   법선을 편향 자세((Rs·rio)^T) 평균으로 잡으면 그 tilt(실측 14°→18°)를 전 구간 강제해
    //   z-drift를 오히려 키웠다(순환참조: 병(편향 자세)으로 병을 고치려 함). 실측 검증:
    //   자유 법선일 때 전역평면 기울기 OFF 14.4°→PLANE 18.1°로 악화.
    //   → 법선은 world-up(Identity)으로 고정하고 optimization에서 상수 처리(SetParameterBlockConstant).
    //     그러면 roll/pitch 잔차가 자세를 '절대 수평'으로 당겨 pitch bias를 근본 교정한다.
    rpw = Eigen::Matrix3d::Identity();
    // 높이 zpw만 데이터에서 추정(법선=Identity이므로 pw = P_wheel, zpw = -mean(P_wheel.z)).
    double sum_zpw = 0.0;
    for (int i = 0; i < cnt; ++i)
        sum_zpw += -(Ps[i] + Rs[i] * tio)[2];
    zpw = sum_zpw / cnt;
    RCLCPP_INFO(rclcpp::get_logger("vins_plane"),
                "[PLANE] init: normal=world-up(fixed), zpw=%.4f (frames=%d)", zpw, cnt);
}

// [SW1-1866 reboot-pose-seed] failure 확정 직후·clearState 직전 호출 — 시드 캡처.
//   계층: ①정지 창 앵커(Q4: 래치 < 첫 절제 시각일 때만 — 오염 후 래치 기각)
//        ②마지막 정화 solve pose(179807f 수술본 정화 정의)
//        ③포기 → 원점 폴백(기존 T_seed 유지, 억지 계승 금지)
void Estimator::captureRebootSeed(double stamp)
{
    Vector3d p_session;
    double   yaw_session_rad;
    const char *src;
    // 앵커 1순위의 전제 = "현재 상태가 오염됐다"(절제 실증 존재). 절제가 없는
    //   failure(big bias 등)는 직전 상태가 건강하므로 신선한 정화 pose가 우월 —
    //   강제 reboot A/B 실증: 무절제 상태서 앵커 시드는 0.56m 낡아 GT 오차 2.2배.
    //   ⚠️[08-12] 위 전제("절제 없음 = 건강")는 **발산 가드 발동 케이스에서 거짓**이다.
    //     가드는 '정지 확정인데 VINS가 계속 움직인' 상태에서 재부팅시키므로 오염됐는데
    //     절제는 없다. 그래서 오염 실증을 절제 시각과 가드 감지 시각의 **합집합**으로
    //     본다(reboot_seed.h contaminationOnset).
    const double contam_t =
        reboot_seed::contaminationOnset(amputate_first_t_, still_drift_first_t_);
    const bool contamination_evidenced = contam_t >= 0.0;
    if (contamination_evidenced && anchor_history_valid_ &&
        reboot_seed::anchorSeedEligible(anchor_latch_t_, contam_t))
    {
        p_session      = still_cum_anchor_;
        p_session.z()  = still_cum_z_anchor_;
        yaw_session_rad = anchor_yaw_deg_ * M_PI / 180.0;
        src = "앵커";
    }
    else if (clean_pose_t_ >= 0.0)
    {
        p_session      = clean_P_;
        yaw_session_rad = clean_yaw_;
        src = "정화pose";
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("vins_reboot_seed"),
                    "[REBOOT-SEED] t=%.3f 시드 재료 없음 — 원점 폴백", stamp);
        return;
    }
    // 발행 프레임으로 변환(기존 T_seed 합성) — 연속성의 기준은 '발행됐던' pose
    const double prev_seed_yaw =
        seed_active_ ? std::atan2(seed_R_(1, 0), seed_R_(0, 0)) : 0.0;
    seed_cap_P_   = seed_active_ ? Vector3d(seed_R_ * p_session + seed_P_) : p_session;
    seed_cap_yaw_ = yaw_session_rad + prev_seed_yaw;
    seed_cap_gyro_yaw_  = bridge_gyro_yaw_rad_;
    seed_cap_wheel_x_   = latest_wheel_x_.load();
    seed_cap_wheel_y_   = latest_wheel_y_.load();
    seed_cap_wheel_yaw_ = latest_wheel_yaw_.load();
    seed_pending_ = true;
    RCLCPP_WARN(rclcpp::get_logger("vins_reboot_seed"),
                "[REBOOT-SEED] t=%.3f 시드 캡처(%s): (%.3f, %.3f, %.3f) yaw=%.1fdeg",
                stamp, src, seed_cap_P_.x(), seed_cap_P_.y(), seed_cap_P_.z(),
                seed_cap_yaw_ * 180.0 / M_PI);
    // [SW1-1866 08-13 계측] 분기 판정에 실제로 쓰인 내부 상태 4값 — 08-12 A/B에서
    //   외부 타임라인이 동일한 런들(r2·r5 vs r13)이 다른 분기로 갈렸는데, 로그 재구성으로는
    //   앵커 자격이 성립해 보여 판별이 막혔다(가드 이벤트 로그는 스로틀·계승 갱신 탓에
    //   내부 상태의 대리 지표가 못 된다는 게 그 수사의 결론). 판정 '입력'을 직접 남긴다.
    RCLCPP_WARN(rclcpp::get_logger("vins_reboot_seed"),
                "[REBOOT-SEED-DIAG] latch_t=%.3f amputate_t=%.3f drift_t=%.3f "
                "hist=%d clean_t=%.3f",
                anchor_latch_t_, amputate_first_t_, still_drift_first_t_,
                anchor_history_valid_ ? 1 : 0, clean_pose_t_);
}

// 재init 완료 후 첫 solve에서 호출 — 캡처~지금 사이 이동(휠 병진+gyro yaw)을 얹어
// T_seed 확정. 주 시나리오(정지 폭주)에선 다리≈0.
void Estimator::finalizeRebootSeed()
{
    // [vins-output-map-anchor] init 완료 공용 훅: seed 비활성 init = 이 세션이 발행
    //   기준 프레임 → T(odom←세션) 스냅샷. 시드 계승 시엔 원 세션 스냅샷 유지.
    if (USE_OUTPUT_MAP_ANCHOR && !seed_active_ && !seed_pending_)
        snapshotOutputAnchor();
    if (!seed_pending_)
        return;
    const double d_yaw = bridge_gyro_yaw_rad_ - seed_cap_gyro_yaw_;
    const Eigen::Vector2d wheel_delta(latest_wheel_x_.load() - seed_cap_wheel_x_,
                                      latest_wheel_y_.load() - seed_cap_wheel_y_);
    const Vector3d d_p =
        reboot_seed::bridgeTranslation(seed_cap_yaw_, seed_cap_wheel_yaw_, wheel_delta);
    reboot_seed::finalizeSeed(seed_cap_P_, seed_cap_yaw_, d_p, d_yaw, seed_R_, seed_P_);
    seed_active_  = true;
    seed_pending_ = false;
    seed_apply_cnt_++;
    RCLCPP_WARN(rclcpp::get_logger("vins_reboot_seed"),
                "[REBOOT-SEED] T_seed 확정(누적 %ld회): (%.3f, %.3f, %.3f) yaw=%.1fdeg "
                "(다리 %.3fm / %.2fdeg)",
                seed_apply_cnt_, seed_P_.x(), seed_P_.y(), seed_P_.z(),
                std::atan2(seed_R_(1, 0), seed_R_(0, 0)) * 180.0 / M_PI,
                d_p.norm(), d_yaw * 180.0 / M_PI);
}

void Estimator::seedTransform(Vector3d &p, Matrix3d &R) const
{
    if (!seed_active_)
        return;
    reboot_seed::compose(seed_R_, seed_P_, p, R);
}

// [SW1-1866 vins-output-map-anchor] 발행단 종합 변환. 사슬:
//   published = T(map→odom) ∘ T(odom←세션) ∘ (T_seed ∘ session)
//   T(odom←세션)은 init 순간 휠 pose 스냅샷이라 VINS를 언제 켰든 정확(부팅 정렬
//   가정 불요). 핀/스냅샷이 없으면 그 단계를 생략 = 현행 동작(우아한 퇴화).
void Estimator::displayTransform(Vector3d &p, Matrix3d &R) const
{
    seedTransform(p, R);
    if (!USE_OUTPUT_MAP_ANCHOR)
        return;
    if (!display_anchor_valid_)
    {
        if (!map_pin_wait_logged_)
        {
            map_pin_wait_logged_ = true;
            RCLCPP_INFO(rclcpp::get_logger("vins_output_anchor"),
                        "[OUTPUT-ANCHOR] 핀 대기 중(map→odom static %s) — "
                        "수신 전까지 세션 프레임 발행",
                        map_odom_pin_valid_.load() ? "수신" : "미수신");
        }
        return;
    }
    // 단일 변환 1회 적용. 구 2단 합성(init 스냅샷 × 나중 핀)의 시각 불일치 제거.
    reboot_seed::composeYawXYZ(disp_yaw_, disp_t_, p, R);
}

// [SW1-1866 08-09] 표시 앵커 확정 — 핀과 세션 pose가 '동시에' 유효한 첫 시점에 1회.
//   추정기 스레드 전용(세션 pose를 창 상태에서 읽으므로). 핀이 init보다 먼저 오면
//   init 직후 여기서 잡히고(S≈I → 구 동작과 동일), 늦게 오면 그 시점 S로 잡혀
//   그 사이 휠 드리프트가 오프셋으로 굳지 않는다.
void Estimator::maybeCaptureDisplayAnchor()
{
    if (!USE_OUTPUT_MAP_ANCHOR || display_anchor_valid_)
        return;
    if (solver_flag != NON_LINEAR || !map_odom_pin_valid_.load())
        return;

    // S = 발행 직전 pose = T_seed ∘ (창 최신 상태)
    Vector3d p_s = Ps[WINDOW_SIZE];
    Matrix3d R_s = Rs[WINDOW_SIZE];
    seedTransform(p_s, R_s);

    // W = T(map→odom) ∘ T(odom←base)(지금) — 휠은 평면이라 z=0
    const Vector3d wheel_t(latest_wheel_x_.load(), latest_wheel_y_.load(), 0.0);
    reboot_seed::computeDisplayAnchor(
        map_odom_yaw_, Eigen::Vector3d(map_odom_x_, map_odom_y_, map_odom_z_),
        latest_wheel_yaw_.load(), wheel_t, p_s, R_s, disp_yaw_, disp_t_);
    display_anchor_valid_ = true;
    RCLCPP_INFO(rclcpp::get_logger("vins_output_anchor"),
                "[OUTPUT-ANCHOR] 표시 앵커 확정: yaw=%.2fdeg t=(%.3f, %.3f, %.3f) "
                "[핀 yaw=%.2f + 휠 yaw=%.2f − 세션 yaw=%.2f]",
                disp_yaw_ * 180.0 / M_PI, disp_t_.x(), disp_t_.y(), disp_t_.z(),
                map_odom_yaw_ * 180.0 / M_PI, latest_wheel_yaw_.load() * 180.0 / M_PI,
                std::atan2(R_s(1, 0), R_s(0, 0)) * 180.0 / M_PI);
}

void Estimator::setMapOdomPin(double x, double y, double z, double yaw)
{
    map_odom_x_ = x; map_odom_y_ = y; map_odom_z_ = z; map_odom_yaw_ = yaw;
    map_odom_pin_valid_.store(true);  // 값 기록 후 플래그(수신 스레드→발행 스레드 공개 순서)
    RCLCPP_INFO(rclcpp::get_logger("vins_output_anchor"),
                "[OUTPUT-ANCHOR] map→odom 핀 수신: (%.3f, %.3f, %.3f) yaw=%.2fdeg",
                x, y, z, yaw * 180.0 / M_PI);
}

// 비시드 init 완료 훅 — 새 세션 프레임이 태어났으므로 표시 앵커를 무효화해 재캡처를
// 예약한다. 값 자체는 maybeCaptureDisplayAnchor()가 '핀과 세션 pose가 동시에 유효한'
// 시점에 잡는다(구현은 init 시점 휠 스냅샷을 그대로 썼고, 그게 시각 불일치의 원인).
// ※시드 계승 중(seed_active_/pending_)에는 호출되지 않는다 — T_seed가 연속성을 잇고
//   원 세션의 표시 앵커를 그대로 유지해야 하기 때문.
void Estimator::snapshotOutputAnchor()
{
    display_anchor_valid_ = false;
    map_pin_wait_logged_  = false;
    RCLCPP_INFO(rclcpp::get_logger("vins_output_anchor"),
                "[OUTPUT-ANCHOR] 새 세션 — 표시 앵커 재캡처 예약(휠 pose=(%.3f, %.3f) "
                "yaw=%.2fdeg)",
                latest_wheel_x_.load(), latest_wheel_y_.load(),
                latest_wheel_yaw_.load() * 180.0 / M_PI);
}

bool Estimator::failureDetection()
{
    // [SW1-1866 07-31] 가드 에스컬레이션 — 정화 없는 연속 prior 절제 상한 도달.
    //   절제로 못 끊는 오염의 결말은 어차피 reboot(실기 2건: 11~13m 폭주 후 big
    //   translation)이므로, 폭주가 발행되기 전에 같은 결말을 앞당긴다.
    if (guard_escalation_fire_)
    {
        ROS_INFO(" guard escalation: prior amputation x%d without recovery",
                 GUARD_ESCALATION_MAX);
        return true;
    }
    // [SW1-1866 08-10 R3] 아래 조건은 로그만 남기고 발동하지 않는다(원본 VINS부터 주석 처리).
    //   현행 유지 근거: 실측상 이 판정이 켜져 있었으면 정상 구간에서도 재초기화가 났다 —
    //   v16 4런서 last_track_num<2가 사고 없이 여러 번 관측됨(사람 통과 외 구간 포함).
    //   되살리려면 '지속 시간' 조건이 함께 필요하다(순간값은 평시에도 바닥을 침).
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        // return true;  // ← 의도적 비활성(위 근거). 복원 시 지속 조건 동반 필수
    }
    if (Bas[WINDOW_SIZE].norm() > FAILURE_BA_MAX)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > FAILURE_BG_MAX)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    // ⚠️아래 두 판정은 last_P/last_R 기준 = **solve 간 델타**다(last_P는 매 solve 갱신,
    //   이 파일 processImage 말미). 따라서 '천천히 크게' 벗어나는 누적 드리프트는 원리적으로
    //   못 본다 — v16 실측: 발행 pose가 40m 이탈 중인데 미발동, 단발 5m 점프가 나서야 발동.
    //   누적 판정은 별도 경로가 담당한다(R2).
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > FAILURE_DP_MAX)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > FAILURE_DZ_MAX)
    {
        ROS_INFO(" big z translation");
        return true;
    }
    Matrix3d    tmp_R   = Rs[WINDOW_SIZE];
    Matrix3d    delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double      delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        // return true;  // ← 의도적 비활성(R3). 이 로봇은 제자리 스핀이 상시 안무라
        //   solve 간 50° 회전이 정상 주행에서도 발생한다(v14 스핀 |w|1.4~1.8 rad/s).
        //   복원하려면 gyro 예측 대비 '잔차' 기준으로 바꿔야 한다(절대 각도 아님).
    }
    return false;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem       problem;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    /*######优化参数：q、p；v、Ba、Bg#######*/
    //添加ceres参数块
    //因为ceres用的是double数组，所以在下面用vector2double做类型装换
    // Ps、Rs转变成para_Pose，Vs、Bas、Bgs转变成para_SpeedBias
    // [SW1-1837] Bg_z 잠금 — 수렴 후 gyro z-bias를 상수로 고정해 'Bg_z 도피' 경로 차단.
    //   근거: 최적화기가 yaw 불일치(휠 타이밍·저품질 장면)를 Bg_z로 도피시켜 참값의
    //   15~40배로 과대추정하는 것이 yaw 드리프트의 단일 지배 원인(인과 봉인 probe:
    //   고정 시 v7 yaw −24°→−2.7°, xy 개선, z/tilt 무비용).
    //   발동은 상태 기반(정지 지속 + 추정 안정 + 크기 가드) — 시동 직후 바로 조작하는
    //   B2C 사용에서도 init 직후의 자연 정지 꼬리(~4-6s)에 자동 발동. 조건 상세와
    //   각 조건이 막는 사고는 bgz_lock.h 참조.
    if (USE_BGZ_LOCK)
    {
        // 정지 판정: 회전(gyro, bias 보정)·병진(추정 속도) 모두 문턱 이하.
        //   실측 표본은 processIMU가 IMU 주기(~380Hz)로 공급(주기 근거는 그쪽 주석).
        const double now      = Headers[frame_count];
        const bool still_now  = (gyr_0 - Bgs[WINDOW_SIZE]).norm() < bgz_lock::kStillGyrRadps &&
                               Vs[WINDOW_SIZE].norm() < bgz_lock::kStillVelMps;
        const bgz_lock::Params lock_params{BGZ_LOCK_DELAY,        BGZ_LOCK_STILL_SEC,
                                           BGZ_LOCK_STAB_MAX,     BGZ_LOCK_FALLBACK_SEC,
                                           BGZ_LOCK_MAX,          BGZ_RELOCK_DELTA,
                                           BGZ_RELOCK_WIN_SEC,    BGZ_RELOCK_COOLDOWN};
        const bool   rest_ready  = bgz_rest_.ready(BGZ_LOCK_STILL_SEC);
        const double rest_median = rest_ready ? bgz_rest_.median(BGZ_LOCK_STILL_SEC) : 0.0;

        if (!bgz_locked_)
        {
            if (bgz_lock_tracker_.update(now, Bgs[WINDOW_SIZE].z(), still_now, rest_ready,
                                         rest_median, lock_params))
            {
                bgz_locked_     = true;
                bgz_locked_val_ = Bgs[WINDOW_SIZE].z();
                RCLCPP_INFO(rclcpp::get_logger("vins_bgz_lock"),
                            "[BGZ-LOCK] engaged t=%.3f bgz=%.6f rad/s (rest=%.6f)", now,
                            bgz_locked_val_, rest_median);
            }
        }
        // 재잠금은 "연속 정지 10s + 10s 창 중앙값"으로만 — 온도 표류는 분 단위 현상.
        //   주행 중 준정지(1~2s)는 ready(10s)가 구조적으로 배제(회귀 오발동 처방).
        else if (still_now && bgz_rest_.ready(BGZ_RELOCK_WIN_SEC) &&
                 bgz_lock::shouldRelock(bgz_rest_.median(BGZ_RELOCK_WIN_SEC),
                                        bgz_locked_val_, now, bgz_last_relock_t_,
                                        lock_params))
        {
            // 재잠금(온도 표류 추종): 잠긴 값이 10s 정지 실측과 유의미하게 벌어짐 →
            //   실측값을 주입하고 그 값으로 재고정. Bg_z 한 축의 ~1e-3급 변화는
            //   preintegration 1차 bias 보정(dq_dbg) 범위라 상태 수술 위험 없음.
            //   vector2double가 이미 실행됐으므로 para 쪽도 함께 갱신한다.
            const double relock_val = bgz_rest_.median(BGZ_RELOCK_WIN_SEC);
            for (int i = 0; i <= frame_count; i++)
            {
                Bgs[i].z()           = relock_val;
                para_SpeedBias[i][8] = relock_val;
            }
            RCLCPP_INFO(rclcpp::get_logger("vins_bgz_lock"),
                        "[BGZ-RELOCK] t=%.3f %.6f -> %.6f rad/s (drift %.6f)", now,
                        bgz_locked_val_, relock_val, relock_val - bgz_locked_val_);
            bgz_locked_val_    = relock_val;
            bgz_last_relock_t_ = now;
        }
    }
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
        {
            if (bgz_locked_)
            {
                // para_SpeedBias 레이아웃 [V(0-2), Ba(3-5), Bg(6-8)] — Bg_z=인덱스 8만 고정
                ceres::SubsetParameterization *sb_lock =
                    new ceres::SubsetParameterization(SIZE_SPEEDBIAS, {8});
                problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS, sb_lock);
            }
            else
                problem.AddParameterBlock(para_SpeedBias[i],
                                          SIZE_SPEEDBIAS);  // v、Ba、Bg参数
        }
    }
    if (!USE_IMU)
    {
        problem.SetParameterBlockConstant(para_Pose[0]);
    }
    /*######优化参数：imu与camera外参#######*/
    for (auto &i : para_Ex_Pose)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(i, SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) ||
            openExEstimation)
        {
            // ROS_DEBUG("estimate extinsic param");
            openExEstimation = true;
        }
        else
        {
            // ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(i);
        }
    }
    /*######优化参数：imu与camera之间的time offset#######*/
    if (USE_IMU)
    {
        problem.AddParameterBlock(para_Td[0], 1);
        //速度过低时，不估计td
        if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        {
            problem.SetParameterBlockConstant(para_Td[0]);
        }
    }

    // ===== 휠 extrinsic/intrinsic/td 파라미터 블록 (Step1: 전부 고정) =====
    if (USE_WHEEL)
    {
        // 휠-body extrinsic (T_io)
        ceres::LocalParameterization *wheel_ex_param = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose_wheel[0], SIZE_POSE, wheel_ex_param);
        if ((ESTIMATE_EXTRINSIC_WHEEL && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) ||
            openExWheelEstimation)
            openExWheelEstimation = true;
        else
            problem.SetParameterBlockConstant(para_Ex_Pose_wheel[0]);

        // ===== 지면평면 제약 (SW1-1837): z 위치 + roll/pitch 자세를 추정 평면에 묶음 =====
        //   vz-only(VerticalVelocityFactor)가 xy로 오차 전가한 것과 달리 근본 pitch까지 교정.
        //   ★법선(para_plane_R)은 world-up으로 '고정'(SetParameterBlockConstant) — 자유변수로 두면
        //     {평면+궤적}이 통째로 기우는 gauge 자유도가 편향 방향으로 흘러 tilt를 키운다(실측 14°→18°).
        //     중력정렬된 world z가 절대 수평 기준. 높이 zpw만 최적화. VI 초기화 완료 후 활성.
        if (USE_PLANE && openPlaneEstimation)
        {
            ceres::LocalParameterization *plane_r_param = new ceres::EigenQuaternionParameterization();
            problem.AddParameterBlock(para_plane_R[0], 4, plane_r_param);
            problem.SetParameterBlockConstant(para_plane_R[0]);   // 법선 = world-up 고정
            problem.AddParameterBlock(para_plane_Z[0], 1);
            for (int i = 0; i <= frame_count; i++)
            {
                // [SW1-1837] 다리 이벤트 중엔 몸체 pitch가 실제로 변함 → 수평/고도 강제가 오차 주입 → skip
                if (isLegGated(Headers[i], Headers[i]))
                    continue;
                ceres::CostFunction *plane_factor =
                    PlaneFactor::Create(PITCH_N_INV, ROLL_N_INV, ZPW_N_INV);
                problem.AddResidualBlock(plane_factor, NULL, para_Pose[i],
                                         para_Ex_Pose_wheel[0], para_plane_R[0], para_plane_Z[0]);
            }
        }

        // 휠 intrinsic 스케일 sx/sy/sw
        problem.AddParameterBlock(para_Ix_sx_wheel[0], 1);
        problem.AddParameterBlock(para_Ix_sy_wheel[0], 1);
        problem.AddParameterBlock(para_Ix_sw_wheel[0], 1);
        if ((ESTIMATE_INTRINSIC_WHEEL && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) ||
            openIxEstimation)
            openIxEstimation = true;
        else
        {
            problem.SetParameterBlockConstant(para_Ix_sx_wheel[0]);
            problem.SetParameterBlockConstant(para_Ix_sy_wheel[0]);
            problem.SetParameterBlockConstant(para_Ix_sw_wheel[0]);
        }

        // 휠 td
        problem.AddParameterBlock(para_Td_wheel[0], 1);
        if (!ESTIMATE_TD_WHEEL || Vs[0].norm() < 0.2)
            problem.SetParameterBlockConstant(para_Td_wheel[0]);
    }

    //构建残差
    /*******先验残差*******/
    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor =
            new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    /*******预积分残差*******/
    if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)  //预积分残差，总数目为frame_count
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt >
                10.0)  //两图像帧之间时间过长，不使用中间的预积分 tzhang
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            //添加残差格式：残差因子，鲁棒核函数，优化变量（i时刻位姿，i时刻速度与偏置，i+1时刻位姿，i+1时刻速度与偏置）
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i],
                                     para_Pose[j], para_SpeedBias[j]);
        }
    }

    /*******휠 preintegration 잔차 (WheelFactor<6,7,7,7,1,1,1,1>)*******/
    if (USE_WHEEL)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations_wheel[j]->sum_dt > 10.0)  // 간격 과대 시 미사용
                continue;

            // [SW1-1837] factor-구간 게이팅: 이 키프레임 구간에 비물리 속도 글리치 샘플이
            //   하나라도 있으면 휠 factor를 추가하지 않는다(그 구간은 IMU/비전이 받침).
            //   preintegration은 그대로 두고 factor만 빼므로 적분 일관성을 깨지 않는다.
            //   임계는 조이스틱 실측 물리한계 기반(선 0.6, 각 3.3). [[edie-wheel-odom-glitch]]
            if (USE_WHEEL_VEL_GATE)
            {
                bool glitch = false;
                for (size_t k = 0; k < linear_velocity_buf_wheel[j].size(); k++)
                {
                    if (linear_velocity_buf_wheel[j][k].norm() > WHEEL_VEL_MAX ||
                        angular_velocity_buf_wheel[j][k].norm() > WHEEL_GYR_MAX)
                    {
                        glitch = true;
                        break;
                    }
                }
                if (glitch)
                    continue;  // 글리치 구간 휠 factor skip
            }

            // [SW1-1837] 다리 이벤트 게이팅: 다리가 움직인 구간은 휠이 몸체 운동을 못 봄(v=w=0 주장)
            //   → 틀린 앵커가 자세로 전가(07-13 실측: 자세 오차 1.6~3.4° 주입) → 그 구간 factor skip.
            //   발동 확인용 INFO는 1s 스로틀(A/B 검증 시 육안 확인, 평시 이벤트 없으면 무출력).
            // [SW1-1866 08-03] 다리 이벤트 중에도 휠 factor '유지'가 기본.
            //   3-bag 판별(v13/v4/v5): skip은 키프레임 '구간' 단위라 정지·저속(키프레임
            //   성김, 실측 최장 7.1s)에선 0.2s 이벤트가 수 초 휠 앵커를 통째 제거(v13
            //   146~158s +0.72m 도약의 주인 — 유지 시 xy RMS 0.30→0.11, v4/v5 동등).
            //   이벤트 중 엔코더 실측은 '이동 작음'이 근사 참(실제 x·y 소이동 존재),
            //   회전 과소보고는 gyr_n_wheel 0.3으로 이미 무해. 0 = 하드 skip 롤백.
            if (!KEEP_WHEEL_IN_LEG_EVENT && isLegGated(Headers[i], Headers[j]))
            {
                static rclcpp::Clock gate_clk;
                RCLCPP_INFO_THROTTLE(rclcpp::get_logger("vins_event_gate"), gate_clk, 1000,
                                     "[EVENT-GATE] leg event: wheel factor skip (seg %.3f~%.3f)",
                                     Headers[i], Headers[j]);
                continue;
            }

            WheelFactor *wheel_factor = new WheelFactor(pre_integrations_wheel[j]);
            // 블록: pose_i, pose_j, T_io(extrinsic), sx, sy, sw, td_wheel
            problem.AddResidualBlock(wheel_factor, NULL, para_Pose[i], para_Pose[j],
                                     para_Ex_Pose_wheel[0], para_Ix_sx_wheel[0],
                                     para_Ix_sy_wheel[0], para_Ix_sw_wheel[0], para_Td_wheel[0]);
        }
    }

    /*******[SW1-1837] Zero-velocity Update (ZUPT): 정지 프레임 속도 0 제약 → z drift 완화*******/
    //   휠 preintegration의 평균 선속도/각속도로 프레임별 정지 판정.
    //   정지면 para_SpeedBias[i]의 속도(0:3)를 0으로 당김(휠이 못 잡는 z velocity까지 제약).
    if (USE_ZUPT && USE_WHEEL)
    {
        // 디버그용 named logger — optimization()엔 노드 핸들이 없어 get_logger 사용.
        // DEBUG 레벨은 ROS2 기본 비활성 → 실행 시 `--ros-args --log-level vins_zupt:=debug` 필요.
        auto zupt_logger = rclcpp::get_logger("vins_zupt");
        int zupt_cnt = 0;
        for (int i = 1; i <= frame_count; i++)
        {
            if (!pre_integrations_wheel[i] || pre_integrations_wheel[i]->sum_dt < 1e-3)
                continue;
            // [SW1-1837] 다리 이벤트 중엔 '휠 정지'가 몸체 정지를 의미하지 않음 → ZUPT 오적용 방지
            if (isLegGated(Headers[i - 1], Headers[i]))
                continue;
            double dt    = pre_integrations_wheel[i]->sum_dt;
            double v_avg = pre_integrations_wheel[i]->delta_p.norm() / dt;  // 평균 선속도 [m/s]
            double w_avg = 2.0 * std::acos(std::min(1.0, std::fabs(
                               pre_integrations_wheel[i]->delta_q.w()))) / dt;  // 평균 각속도 [rad/s]
            // 두 평균이 모두 임계 미만이면 정지로 판정 (AND 조건)
            bool is_zupt = (v_avg < ZUPT_VEL_THRESH && w_avg < ZUPT_GYR_THRESH);
            // 프레임별 실측값 vs 임계값 + 판정 결과 (정지로 판정 시 STOP, 아니면 move)
            RCLCPP_DEBUG(zupt_logger,
                         "[ZUPT] frame=%d v_avg=%.4f(th %.3f) w_avg=%.4f(th %.3f) -> %s",
                         i, v_avg, ZUPT_VEL_THRESH, w_avg, ZUPT_GYR_THRESH,
                         is_zupt ? "STOP" : "move");
            if (is_zupt)
            {
                ZeroVelocityFactor *zupt_factor = new ZeroVelocityFactor(ZUPT_WEIGHT);
                problem.AddResidualBlock(zupt_factor, NULL, para_SpeedBias[i]);
                zupt_cnt++;
            }
        }
        // 이번 최적화에서 정지 제약이 걸린 프레임 수 요약
        RCLCPP_DEBUG(zupt_logger, "[ZUPT] applied %d / %d frames", zupt_cnt, frame_count);
    }

    // [SW1-1866] 정지 상대운동 잠금 — 오염 비전의 '새 프레임 배치 오차' 차단.
    //   정지 확정(휠 정지+다리 게이트 아님+gyro 정온 — 게이지 가드와 동일 3중 검사) 구간의
    //   인접 프레임에 상대 위치·yaw=0 관측 주입. 배경·과제약 회피는 still_motion_factor.h.
    if (USE_STILL_MOTION_LOCK && USE_WHEEL)
    {
        for (int i = 1; i <= frame_count; i++)
        {
            if (!pre_integrations_wheel[i] || pre_integrations_wheel[i]->sum_dt < 1e-3 ||
                !pre_integrations[i] || pre_integrations[i]->gyr_buf.empty())
                continue;
            if (isLegGated(Headers[i - 1], Headers[i]))
                continue;
            const double dt_w  = pre_integrations_wheel[i]->sum_dt;
            const double v_avg = pre_integrations_wheel[i]->delta_p.norm() / dt_w;
            const double w_avg = 2.0 * std::acos(std::min(1.0, std::fabs(
                                     pre_integrations_wheel[i]->delta_q.w()))) / dt_w;
            Vector3d g_mean = Vector3d::Zero();
            for (const auto &g : pre_integrations[i]->gyr_buf)
                g_mean += g;
            g_mean /= static_cast<double>(pre_integrations[i]->gyr_buf.size());
            if (v_avg < 0.02 && w_avg < 0.02 && (g_mean - Bgs[i]).norm() < 0.05)
                problem.AddResidualBlock(
                    StillMotionFactor::Create(STILL_LOCK_POS_W, STILL_LOCK_YAW_W), NULL,
                    para_Pose[i - 1], para_Pose[i]);
        }
    }

    /*******[SW1-1837] 정지 시 중력 재정렬: 정지 프레임 acc 평균(≈중력 방향)으로 잔여 roll/pitch 교정*******/
    //   이벤트 게이팅(예방)이 못 막은 잔여 자세 주입의 사후 교정(감쇠≠차단, doc/EVENT_GATING.md 판정 절).
    //   정지 중 acc는 중력만 감지 = 절대 roll/pitch 관측. 부팅 imu_offsets 교정으로 Ba≈0 전제,
    //   잔여 Ba는 현재 추정치를 '상수'로 차감(bias를 파라미터로 안 넣는 이유는 factor 헤더 참조).
    //   정지 판정 = 휠(ZUPT와 동일식, 전용 임계 — use_zupt:0이어도 동작) AND IMU 자체 검증
    //   (자이로 평균·acc 흔들림) AND 다리 이벤트 게이팅 구간 제외(휠 정지 ≠ 몸체 정지).
    if (USE_GRAVITY_ALIGN == 1)  // 모드 1 = soft factor (w200/w1000 A/B 기각 — 실험 기록용 봉인)
    {
        // IMU 자체 정지 검증 임계 — 자이로: 회전 부재, acc 표준편차: 진동/운동 부재.
        //   acc 상한은 28~29Hz 구조 공진의 정지 시 진폭을 흡수하도록 여유(실측 기반 재조정 여지).
        constexpr double kGaGyrQuiet  = 0.05;  // [rad/s] Bg 차감 후 자이로 평균 노름 상한
        constexpr double kGaAccStdMax = 0.5;   // [m/s^2] acc 편차 RMS 상한
        int ga_cnt = 0;
        for (int i = 1; i <= frame_count; i++)
        {
            if (!pre_integrations_wheel[i] || pre_integrations_wheel[i]->sum_dt < 1e-3)
                continue;
            if (!pre_integrations[i] || pre_integrations[i]->acc_buf.size() < 5)
                continue;
            // 다리 이벤트 중 '휠 정지'는 몸체 정지가 아님 → 재정렬 금지
            if (isLegGated(Headers[i - 1], Headers[i]))
                continue;
            const double ga_dt    = pre_integrations_wheel[i]->sum_dt;
            const double ga_v_avg = pre_integrations_wheel[i]->delta_p.norm() / ga_dt;
            const double ga_w_avg = 2.0 * std::acos(std::min(1.0, std::fabs(
                                        pre_integrations_wheel[i]->delta_q.w()))) / ga_dt;
            if (ga_v_avg >= GRAVITY_ALIGN_VEL_THRESH || ga_w_avg >= GRAVITY_ALIGN_GYR_THRESH)
                continue;
            // IMU 창 통계 (프레임 i 구간의 acc/gyr 원시 샘플)
            const auto &ga_accs = pre_integrations[i]->acc_buf;
            const auto &ga_gyrs = pre_integrations[i]->gyr_buf;
            Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
            for (const auto &a : ga_accs) acc_mean += a;
            for (const auto &g : ga_gyrs) gyr_mean += g;
            acc_mean /= static_cast<double>(ga_accs.size());
            gyr_mean /= static_cast<double>(ga_gyrs.size());
            double acc_var = 0.0;
            for (const auto &a : ga_accs) acc_var += (a - acc_mean).squaredNorm();
            const double acc_std = std::sqrt(acc_var / static_cast<double>(ga_accs.size()));
            if ((gyr_mean - Bgs[i]).norm() > kGaGyrQuiet || acc_std > kGaAccStdMax)
                continue;
            // 실측 중력(위) 방향 = acc 평균 − 현재 Ba 추정(상수 취급, 매 최적화마다 최신값으로 갱신됨)
            const Eigen::Vector3d g_body = (acc_mean - Bas[i]).normalized();
            problem.AddResidualBlock(GravityAlignFactor::Create(g_body, GRAVITY_ALIGN_WEIGHT),
                                     NULL, para_Pose[i]);
            ga_cnt++;
        }
        if (ga_cnt > 0)
        {
            static rclcpp::Clock ga_clk;
            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("vins_gravity_align"), ga_clk, 1000,
                                 "[GRAV-ALIGN] applied %d / %d frames", ga_cnt, frame_count);
        }
    }

    /*******[SW1-1836] Accelerometer-bias prior: 수평 acc bias 과대추정 억제 → z drift 완화*******/
    //   평지 observability 약점으로 vins가 수평 acc bias를 과대추정 → pitch bias → z 누설.
    //   부팅 시 IMU 드라이버가 bias를 ~0 보정하므로, 윈도 각 프레임의 acc bias를 0으로 약하게 당긴다.
    //   az는 이미 정확 → w_z 기본 0(수평 ax/ay만 제약). 검증은 live 연속주행 A/B(doc/ACC_BIAS_PRIOR.md).
    if (USE_ACC_BIAS_PRIOR)
    {
        const Eigen::Vector3d ab_w(ACC_BIAS_PRIOR_W_XY, ACC_BIAS_PRIOR_W_XY, ACC_BIAS_PRIOR_W_Z);
        const Eigen::Vector3d ab_target(0.0, 0.0, 0.0);  // 부팅 calibration이 bias를 ~0으로 만듦
        for (int i = 0; i <= frame_count; i++)
        {
            AccBiasPriorFactor *ab_factor = new AccBiasPriorFactor(ab_w, ab_target);
            problem.AddResidualBlock(ab_factor, NULL, para_SpeedBias[i]);
        }
    }

    /*******[SW1-1837] Vertical-velocity soft constraint (planar-motion Level1): 주행중 Vz→0 → z drift 억제*******/
    //   지면 로봇은 평지서 월드 수직속도≈0. ZUPT(정지만)·휠(평면만)이 못 잡는 z를 주행 중에도 상시 약하게 억제.
    //   ⚠️ 평지 가정 → 경사 미대응(소프트). 지형 게이팅·바디NHC·plane_factor(자세까지)는 후속(Level2/이식).
    if (USE_VERTICAL_VEL)
    {
        for (int i = 0; i <= frame_count; i++)
        {
            // [SW1-1837] 다리 이벤트 중엔 몸체가 실제로 수직 운동(들어올림) → vz=0 강제가 오차 주입 → skip
            if (isLegGated(Headers[i], Headers[i]))
                continue;
            VerticalVelocityFactor *vz_factor = new VerticalVelocityFactor(VERTICAL_VEL_WEIGHT);
            problem.AddResidualBlock(vz_factor, NULL, para_SpeedBias[i]);
        }
    }

    /*******[SW1-1837] Body-frame NHC (planar-motion Level2, 레버암 보상 v2): 바퀴원점 vy·vz→0*******/
    //   월드 vz 제약과의 차이 = '같은 속도를 어느 축으로 재느냐'. 경사(pitch θ)에선 월드 vz=|v|sinθ≠0
    //   (0 강제는 오차 주입)이지만 바디 vz는 여전히 0(바닥을 뚫거나 뜨지 않음) → 게이팅 없이 참.
    //   ★레버암(07-02 실증): 상태 V는 IMU 위치 속도라 제자리회전 중 실제 횡속도 ω×t_io(≈0.32m/s)가
    //   있음 — 보상 없이 vy=0 강제 시 xy +80% 악화. 프레임별 gyro 측정(angular_velocity_buf 마지막
    //   샘플 ≈ 그 프레임 시각의 ω)을 상수로 넘기고 bias는 상태 Bg로 잔차 안에서 보정.
    //   rio·tio는 상수 전달 — 파라미터 블록로 넣으면 use_wheel:0일 때 자유 gauge 위험(07-01 실증).
    if (USE_BODY_NHC)
    {
        const Eigen::Quaterniond qio(rio);
        for (int i = 0; i <= frame_count; i++)
        {
            // 프레임 i 시각의 각속도: buf[i]의 마지막 IMU 샘플. 비어 있으면(슬라이드 직후 등)
            // 다음 프레임 buf의 첫 샘플로 대체, 그것도 없으면 레버암 항 생략(0) — 소프트라 허용.
            Eigen::Vector3d gyr_i = Eigen::Vector3d::Zero();
            if (!angular_velocity_buf[i].empty())
                gyr_i = angular_velocity_buf[i].back();
            else if (i + 1 <= frame_count && !angular_velocity_buf[i + 1].empty())
                gyr_i = angular_velocity_buf[i + 1].front();
            ceres::CostFunction *nhc_factor =
                BodyNhcFactor::Create(qio, tio, gyr_i, NHC_Y_WEIGHT, NHC_Z_WEIGHT);
            problem.AddResidualBlock(nhc_factor, NULL, para_Pose[i], para_SpeedBias[i]);
        }
    }

    /*******重投影残差*******/
    //重投影残差相关，此时使用了Huber损失核函数
    int f_m_cnt       = 0;
    int feature_index = -1;
    // [SW1-1837] 프레임별 고속 회전 플래그 사전계산 — 관측마다 재계산 방지.
    //   게이트는 끝점(imu_j)만 검사한다. 앵커(imu_i)까지 확장하는 변형은 A/B로 기각됨:
    //   v7 yaw엔 무효과였고 v5에서 z범위를 2배 이상 악화시켰다(게이트량 2배의 비용만 확인).
    bool fast_rot[WINDOW_SIZE + 1] = {false};
    if (USE_YAW_GATING)
        for (int fi = 0; fi <= frame_count; fi++)
            fast_rot[fi] = yaw_gating::isFastRotation(angular_velocity_buf[fi], Bgs[fi],
                                                      YAW_GATE_GYR_THRESH);

    for (auto &it_per_id : f_manager.feature)
    {
        if (it_per_id.is_dynamic)
        {
            continue;
        }
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        // if (it_per_id.used_num < 4)
        //     continue;
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)  //遍历观测到路标点的图像帧
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            // [SW1-1837] 고속 회전 프레임의 관측은 특징 추적 불신 → 재투영 factor skip.
            //   그 구간의 상대 pose(yaw 포함)는 IMU preintegration(gyro, GT −0.11%)이 담당.
            if (USE_YAW_GATING && fast_rot[imu_j])
            {
                yaw_gated_obs_++;
                continue;
            }
            Vector3d pts_j = it_per_frame.point;  //测量值
            if (ESTIMATE_TD)
            {
                ProjectionTdFactor *f_td = new ProjectionTdFactor(
                    pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                    it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j],
                                         para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                if (it_per_id.estimate_flag == 1 && FIX_DEPTH)
                    problem.SetParameterBlockConstant(para_Feature[feature_index]);
                else if (it_per_id.estimate_flag == 2)
                {
                    problem.SetParameterUpperBound(para_Feature[feature_index], 0,
                                                   2 / DEPTH_MAX_DIST);
                }
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j],
                                         para_Ex_Pose[0], para_Feature[feature_index]);
                if (it_per_id.estimate_flag == 1 && FIX_DEPTH)
                    problem.SetParameterBlockConstant(para_Feature[feature_index]);
                else if (it_per_id.estimate_flag == 2)
                {
                    // 防止远处的点深度过小
                    problem.SetParameterUpperBound(para_Feature[feature_index], 0,
                                                   2 / DEPTH_MAX_DIST);
                }
            }
            f_m_cnt++;
        }
    }
    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());
    // [SW1-1837] 고속 회전 게이팅 진단 — 이번 최적화 채택 관측 vs 누적 skip 관측.
    //   A/B 분석용이라 환경변수로만 활성(기본 off, BG 로그와 동일 방식): 최적화마다
    //   찍혀 운영 콘솔을 덮고 [BGZ-LOCK] 같은 중요 로그를 가린다.
    static const bool yaw_gate_log = (std::getenv("VINS_YAW_GATE_LOG") != nullptr);
    if (USE_YAW_GATING && yaw_gate_log)
        RCLCPP_INFO(rclcpp::get_logger("vins_yaw_gating"),
                    "[YAW-GATE] used=%d gated_total=%ld", f_m_cnt, yaw_gated_obs_);

    // [SW1-1837] Bg 시계열 진단 로그 — Bg_z 인플레 검증/포렌식용, 환경변수로만 활성(기본 off)
    static const bool bg_log = (std::getenv("VINS_BG_LOG") != nullptr);
    if (bg_log)
        RCLCPP_INFO(rclcpp::get_logger("vins_bg_probe"),
                    "[BG] t=%.3f bgx=%.6f bgy=%.6f bgz=%.6f",
                    Headers[WINDOW_SIZE], Bgs[WINDOW_SIZE].x(), Bgs[WINDOW_SIZE].y(),
                    Bgs[WINDOW_SIZE].z());

    // [SW1-1866] 동적 장애물 yaw 폭주 포렌식 — 최적화별 특징점 구성(환경변수 게이트, 기본 off).
    //   폭주 중 비전이 '무엇으로' 최적화에 참여하는지: 채택 관측 수(used), 관리 중 특징 수,
    //   is_dynamic 낙인 수. 낙인 0 + used 다수면 방어 회피 확정, used≈0이면 게이지 무저항 확정.
    static const bool dynobs_log = (std::getenv("VINS_DYNOBS_LOG") != nullptr);
    if (dynobs_log)
    {
        int n_total = 0, n_dyn = 0, n_depth = 0;
        for (auto &it_per_id : f_manager.feature)
        {
            n_total++;
            if (it_per_id.is_dynamic)
                n_dyn++;
            if (it_per_id.estimated_depth > 0)
                n_depth++;
        }
        RCLCPP_INFO(rclcpp::get_logger("vins_dynobs_probe"),
                    "[DYNOBS] t=%.3f used=%d total=%d dyn=%d depth=%d",
                    Headers[WINDOW_SIZE], f_m_cnt, n_total, n_dyn, n_depth);
    }

    //添加闭环检测残差，计算滑动窗口中与每一个闭环关键帧的相对位姿，这个相对位置是为后面的图优化准备
    if (relocalization_info)
    {
        // printf("set relocalization factor! \n");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index         = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            if (it_per_id.is_dynamic)
            {
                continue;
            }
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            // if (it_per_id.used_num < 4)
            //     continue;
            ++feature_index;
            int start = it_per_id.start_frame;
            if (start <= relo_frame_local_index)
            {
                while ((int)match_points[retrive_feature_index].z() < it_per_id.feature_id)
                {
                    retrive_feature_index++;
                }
                if ((int)match_points[retrive_feature_index].z() == it_per_id.feature_id)
                {
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(),
                                              match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose,
                                             para_Ex_Pose[0], para_Feature[feature_index]);
                    retrive_feature_index++;
                }
            }
        }
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //    options.num_threads = 4;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations         = NUM_ITERATIONS;
    //    options.use_explicit_schur_complement = true;
    //    options.minimizer_progress_to_stdout = false;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc                 t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));

    // 防止优化结果在零空间变化，通过固定第一帧的位姿
    double2vector();

    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    //边缘化处理
    //如果次新帧是关键帧，将边缘化最老帧，及其看到的路标点和IMU数据，将其转化为先验：
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        // 先验部分，基于先验残差，边缘化滑窗中第0帧时刻的状态向量
        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor =
                new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (USE_IMU)
        {
            // imu
            // 预积分部分，基于第0帧与第1帧之间的预积分残差，边缘化第0帧状态向量
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor         *imu_factor          = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                    imu_factor, NULL,
                    vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1],
                                     para_SpeedBias[1]},
                    vector<int>{0, 1});  //边缘化 para_Pose[0], para_SpeedBias[0]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // 휠 preintegration 잔차를 marg에 추가 → para_Pose[0]만 drop (extrinsic/intrinsic/td는 보존)
        if (USE_WHEEL)
        {
            if (pre_integrations_wheel[1]->sum_dt < 10.0)
            {
                WheelFactor       *wheel_factor        = new WheelFactor(pre_integrations_wheel[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                    wheel_factor, NULL,
                    vector<double *>{para_Pose[0], para_Pose[1], para_Ex_Pose_wheel[0],
                                     para_Ix_sx_wheel[0], para_Ix_sy_wheel[0], para_Ix_sw_wheel[0],
                                     para_Td_wheel[0]},
                    vector<int>{0});  // para_Pose[0] 만 边缘化
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        //图像部分，基于与第0帧相关的图像残差，边缘化第一次观测的图像帧为第0帧的路标点和第0帧
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                if (it_per_id.is_dynamic)
                    continue;
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                // if (it_per_id.used_num < 4)
                //     continue;
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)  //仅处理第一次观测的图像帧为第0帧的情形
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame :
                     it_per_id.feature_per_frame)  //对观测到路标点的图像帧的遍历
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    // [SW1-1837] 최적화와 동일 게이트 — marg prior도 제외해
                    //   최적화/marg 일관성 유지(불일치 시 prior가 skip한 factor를 되살림).
                    //   fast_rot는 위 최적화 단계서 사전계산된 배열 재사용(같은 함수 스코프).
                    if (USE_YAW_GATING && fast_rot[imu_j])
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD)
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(
                            pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
                            it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td,
                            it_per_frame.cur_td, it_per_id.feature_per_frame[0].uv.y(),
                            it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                            f_td, loss_function,
                            vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                             para_Feature[feature_index], para_Td[0]},
                            vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor  *f                   = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                            f, loss_function,
                            vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                             para_Feature[feature_index]},
                            vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        //仅仅改变滑窗double部分地址映射，具体值的通过slideWindow和vector2double函数完成；记住边缘化仅仅改变A和b，不改变状态向量
        //由于第0帧观测到的路标点全被边缘化，即边缘化后保存的状态向量中没有路标点;因此addr_shift无需添加路标点
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)  //最老图像帧数据丢弃，从i=1开始遍历
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] =
                para_Pose[i - 1];  // i数据保存到1-1指向的地址，滑窗向前移动一格
            if (USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (auto &i : para_Ex_Pose)
            addr_shift[reinterpret_cast<long>(i)] = i;
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        // 휠 extrinsic/intrinsic/td는 슬라이딩과 무관하게 자기 자신으로 유지
        if (USE_WHEEL)
        {
            addr_shift[reinterpret_cast<long>(para_Ex_Pose_wheel[0])] = para_Ex_Pose_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sx_wheel[0])]   = para_Ix_sx_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sy_wheel[0])]   = para_Ix_sy_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sw_wheel[0])]   = para_Ix_sw_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Td_wheel[0])]      = para_Td_wheel[0];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        delete last_marginalization_info;
        last_marginalization_info             = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else  //将次新的图像帧数据边缘化； tzhang
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks),
                       std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int>
                    drop_set;  //记录需要丢弃的变量在last_marginalization_parameter_blocks中的索引
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size());
                     i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] !=
                               para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor =
                    new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                    marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            //            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            //            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)  // WINDOW_SIZE - 1会被边缘化，不保存
                    continue;
                else if (i == WINDOW_SIZE)  // WINDOW_SIZE数据保存到WINDOW_SIZE-1指向的地址
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];

                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];

                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (auto &i : para_Ex_Pose)
                addr_shift[reinterpret_cast<long>(i)] = i;
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
            // 휠 extrinsic/intrinsic/td 유지
            if (USE_WHEEL)
            {
                addr_shift[reinterpret_cast<long>(para_Ex_Pose_wheel[0])] = para_Ex_Pose_wheel[0];
                addr_shift[reinterpret_cast<long>(para_Ix_sx_wheel[0])]   = para_Ix_sx_wheel[0];
                addr_shift[reinterpret_cast<long>(para_Ix_sy_wheel[0])]   = para_Ix_sy_wheel[0];
                addr_shift[reinterpret_cast<long>(para_Ix_sw_wheel[0])]   = para_Ix_sw_wheel[0];
                addr_shift[reinterpret_cast<long>(para_Td_wheel[0])]      = para_Td_wheel[0];
            }

            vector<double *> parameter_blocks =
                marginalization_info->getParameterBlocks(addr_shift);

            delete last_marginalization_info;
            last_marginalization_info             = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)  // 边缘化最老的图像帧，即次新的图像帧为关键帧
    {
        double t_0 = Headers[0];
        back_R0    = Rs[0];
        back_P0    = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            // 1、滑窗中的数据往前移动一帧；运行结果就是WINDOW_SIZE位置的状态为之前0位置对应的状态
            //  0,1,2...WINDOW_SIZE——>1,2...WINDOW_SIZE,0
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Rs[i].swap(Rs[i + 1]);
                if (USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
                if (USE_WHEEL)
                {
                    std::swap(pre_integrations_wheel[i], pre_integrations_wheel[i + 1]);

                    dt_buf_wheel[i].swap(dt_buf_wheel[i + 1]);
                    linear_velocity_buf_wheel[i].swap(linear_velocity_buf_wheel[i + 1]);
                    angular_velocity_buf_wheel[i].swap(angular_velocity_buf_wheel[i + 1]);
                }
                ++find_solved[i + 1];
                find_solved[i] = find_solved[i + 1];
            }
            // 2、处理前，WINDOW_SIZE位置的状态为之前0位置对应的状态；处理后，WINDOW_SIZE位置的状态为之前WINDOW_SIZE位置对应的状态;之前0位置对应的状态被剔除
            //  0,1,2...WINDOW_SIZE——>1,2...WINDOW_SIZE,WINDOW_SIZE
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE]      = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE]      = Rs[WINDOW_SIZE - 1];
            if (USE_IMU)
            {
                Vs[WINDOW_SIZE]  = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] =
                    new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            if (USE_WHEEL)
            {
                delete pre_integrations_wheel[WINDOW_SIZE];
                pre_integrations_wheel[WINDOW_SIZE] =
                    new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel};

                dt_buf_wheel[WINDOW_SIZE].clear();
                linear_velocity_buf_wheel[WINDOW_SIZE].clear();
                angular_velocity_buf_wheel[WINDOW_SIZE].clear();
            }
            find_solved[WINDOW_SIZE] = 0;

            // 3、对时刻t_0(对应滑窗第0帧)之前的所有数据进行剔除；即all_image_frame中仅保留滑窗中图像帧0与图像帧WINDOW_SIZE之间的数据
            map<double, ImageFrame>::iterator it_0;
            it_0 = all_image_frame.find(t_0);
            delete it_0->second.pre_integration;
            it_0->second.pre_integration = nullptr;
            for (auto it = all_image_frame.begin(); it != it_0; ++it)
            {
                delete it->second.pre_integration;
                it->second.pre_integration = NULL;
            }
            all_image_frame.erase(all_image_frame.begin(), it_0);
            all_image_frame.erase(t_0);
            slideWindowOld();
        }
    }
    else  //边缘化次新的图像帧，主要完成的工作是数据衔接 tzhang
    {     // 0,1,2...WINDOW_SIZE-2, WINDOW_SIZE-1,
        // WINDOW_SIZE——>0,,1,2...WINDOW_SIZE-2,WINDOW_SIZE, WINDOW_SIZE
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1]      = Ps[frame_count];
            Rs[frame_count - 1]      = Rs[frame_count];

            find_solved[WINDOW_SIZE] = 0;

            if (USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double   tmp_dt                  = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity    = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration,
                                                                 tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }
                Vs[frame_count - 1]  = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] =
                    new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            if (USE_WHEEL)  // 휠 데이터 연결 (제거되는 차차신 프레임의 적분을 이전 프레임에 흡수)
            {
                for (unsigned int i = 0; i < dt_buf_wheel[frame_count].size(); i++)
                {
                    double   tmp_dt               = dt_buf_wheel[frame_count][i];
                    Vector3d tmp_linear_velocity  = linear_velocity_buf_wheel[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf_wheel[frame_count][i];

                    pre_integrations_wheel[frame_count - 1]->push_back(
                        tmp_dt, tmp_linear_velocity, tmp_angular_velocity);

                    dt_buf_wheel[frame_count - 1].push_back(tmp_dt);
                    linear_velocity_buf_wheel[frame_count - 1].push_back(tmp_linear_velocity);
                    angular_velocity_buf_wheel[frame_count - 1].push_back(tmp_angular_velocity);
                }

                delete pre_integrations_wheel[WINDOW_SIZE];
                pre_integrations_wheel[WINDOW_SIZE] =
                    new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel};

                dt_buf_wheel[WINDOW_SIZE].clear();
                linear_velocity_buf_wheel[WINDOW_SIZE].clear();
                angular_velocity_buf_wheel[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

/**
 * @brief   进行重定位
 * @optional
 * @param[in]   _frame_stamp    重定位帧时间戳
 * @param[in]   _frame_index    重定位帧索引值
 * @param[in]   _match_points   重定位帧的所有匹配点
 * @param[in]   _relo_t     重定位帧平移向量
 * @param[in]   _relo_r     重定位帧旋转矩阵
 * @return      void
 */
void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points,
                             Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t  = std::move(_relo_t);
    prev_relo_r  = std::move(_relo_r);
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        if (relo_frame_stamp == Headers[i])
        {
            relo_frame_local_index = i;
            relocalization_info    = true;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration,
                         const Vector3d &angularVelocity)
{
    m_imu.lock();
    imu_buf.push(make_pair(t, make_pair(linearAcceleration, angularVelocity)));
    // imu_predict_buf.push(
    //     make_pair(t, make_pair(linearAcceleration, angularVelocity)));
    m_imu.unlock();

    if (solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        // predict imu (no residual error)
        m_propagate.lock();
        predict(t, linearAcceleration, angularVelocity);
        m_propagate.unlock();
        // [reboot-pose-seed] 고주기 경로에도 동일 시드 합성(저주기 발행과 프레임 일치)
        Vector3d pub_P = latest_P;
        Matrix3d pub_R = latest_Q.toRotationMatrix();
        displayTransform(pub_P, pub_R);
        Vector3d net_p = Vector3d::Zero();
        Matrix3d net_R = Matrix3d::Identity();
        displayTransform(net_p, net_R);
        pubLatestOdometry(pub_P, Quaterniond(pub_R), Vector3d(net_R * latest_V), t);
    }
}

// 휠 오도메트리 입력 — 비동기 버퍼에 적재 (inputIMU 미러, SW1-1829)
// linearVelocity = odom twist.linear, angularVelocity = odom twist.angular
void Estimator::inputWheel(double t, const Vector3d &linearVelocity,
                           const Vector3d &angularVelocity)
{
    m_wheel.lock();
    wheelVelBuf.push(make_pair(t, linearVelocity));
    wheelGyrBuf.push(make_pair(t, angularVelocity));
    m_wheel.unlock();
    // [SW1-1837] Bg_z 잠금 정지 판별자 — 엔코더가 돌면 '준정지'(느린 잔여 회전)도
    //   정지가 아님을 직접 알 수 있음(gyro 문턱만으로는 구분 불가, 회귀 실측).
    last_wheel_speed_.store(std::max(linearVelocity.norm(), angularVelocity.norm()));
    // [SW1-1866 07-31] z 앵커 계승 판정 입력 — 휠 '병진'이 있었으면 바닥 기준 이동
    //   가능성 → 계승 차단. 제자리 회전은 z 불변이라 병진만 본다. 임계 0.05 = 저속
    //   데드밴드 실측 상한(그 이하는 엔코더가 물리적으로 정지와 구분 못 함).
    if (linearVelocity.norm() > 0.05)
        z_anchor_wheel_moved_.store(true);
    // Step1: fast-predict/퍼블리시 생략 (VINS 출력은 vision+IMU 기반 그대로 유지)
}

// [SW1-1837] 다리 실측 각도 입력 (joint_states) — 이벤트 게이팅 detector에 전달
void Estimator::inputLegState(double t, double theta_l, double theta_r)
{
    std::lock_guard<std::mutex> lk(m_leg_gate);
    leg_gate.onMeasurement(t, theta_l, theta_r);
    // [SW1-1866 07-31] z 앵커 계승 판정 입력 — 최신 다리각(앵커 시점 대비 복귀 여부)
    latest_leg_l_.store(theta_l);
    latest_leg_r_.store(theta_r);
    latest_leg_valid_.store(true);
    // [SW1-1866] 워밍업 게이트 표본 입력 — 다리 이벤트 진행 중 여부(기존 detector 재사용)
    leg_gate_active_now_.store(leg_gate.overlaps(t, t));
}

// [SW1-1837] 다리 위치 명령 입력 — 실측 반응 전에 게이트를 선행 개시
void Estimator::inputLegCommand(double t, double target, bool left)
{
    std::lock_guard<std::mutex> lk(m_leg_gate);
    leg_gate.onCommand(t, target, left);
}

// [SW1-1837] [t0,t1]이 다리 이벤트 구간(마진 포함)과 겹치는가 — optimization()의 factor skip 판정
bool Estimator::isLegGated(double t0, double t1)
{
    if (!USE_EVENT_GATING || !GATE_LEG)
        return false;
    std::lock_guard<std::mutex> lk(m_leg_gate);
    return leg_gate.overlaps(t0, t1);
}

void Estimator::updateLatestStates()
{
    m_propagate.lock();
    // [SW1-1872] 재기저 점프 계측 — HF 예측 경로 결함 수정 전/후 A/B의 근거 지표.
    // '직전' = 예측 경로가 IMU마다 이어온 상태, '직후' = 최적 해 재기저 + 잔여 IMU 재적분 상태.
    // 두 상태 모두 imu_buf의 최신 샘플 시각에서 끝나므로 직접 비교 가능(t_pre/t_post 로그로 검증).
    // 자세한 배경: doc/HF_TF_PREDICT_DEFECTS.md §6-1
    const bool             diag_valid = !init_imu; // 예측이 한 번도 안 돌았으면 비교 무의미
    const Eigen::Vector3d  diag_P_pre = latest_P;
    const Eigen::Quaterniond diag_Q_pre = latest_Q;
    const double           diag_t_pre = latest_time;

    latest_time = Headers[frame_count] + td;
    latest_P    = Ps[frame_count];
    latest_Q    = Rs[frame_count];
    latest_V    = Vs[frame_count];
    latest_Ba   = Bas[frame_count];
    latest_Bg   = Bgs[frame_count];

    size_t diag_n_imu = 0;
    if (USE_IMU)
    {
        m_imu.lock();
        diag_n_imu = imu_buf.size();
        // [SW1-1872 결함② 수정] 재기저 시 예측 경로의 '직전 샘플'을 창 경로가 마지막으로
        // 소비한 샘플로 재장전 — 재적분 첫 스텝의 중점 적분 이전 값 (VINS-Fusion 준용)
        latest_acc_0 = acc_0;
        latest_gyr_0 = gyr_0;
        // [SW1-1872 결함① 수정] 시각과 값을 "같은 큐 원소"에서 꺼내 재적분한다. 기존 코드는
        // 값만 imu_buf.front()로 고정되어(upstream 39dcb63d 오타) 지연 창 전체를 IMU 한
        // 샘플로 zero-order-hold 적분했다 — doc/HF_TF_PREDICT_DEFECTS.md §3
        hf_predict::forEachSample(
            imu_buf, [this](double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
            { predict(t, acc, gyr); });
        m_imu.unlock();
    }

    if (diag_valid)
    {
        // 점프 크기: 위치 노름 + 상대회전의 yaw 성분. dt_cmp≠0이면 비교 시각이 어긋난
        // 표본이므로 분석 단계에서 걸러낸다(스레드 경합 등 드문 케이스 방어).
        const double jump_p  = (latest_P - diag_P_pre).norm();
        const Eigen::AngleAxisd rel(diag_Q_pre.inverse() * latest_Q);
        const double jump_ang = rel.angle() * 180.0 / M_PI;
        RCLCPP_INFO(rclcpp::get_logger("vins_hf_rebase"),
                    "[HF-REBASE-DIAG] t=%.6f jp=%.5f jang=%.4f n=%zu dt_cmp=%.6f",
                    latest_time, jump_p, jump_ang, diag_n_imu, latest_time - diag_t_pre);
    }
    m_propagate.unlock();
}

Matrix3d Estimator::predictMotion(double t0, double t1)
{
    Matrix3d relative_R = Matrix3d::Identity();
    if (imu_buf.empty())
        return relative_R;

    bool            first_imu = true;
    double          prev_imu_time;
    Eigen::Vector3d prev_gyr;

    m_imu.lock();
    queue<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_predict_buf = imu_buf;
    m_imu.unlock();
    if (t1 <= imu_predict_buf.back().first)
    {
        while (imu_predict_buf.front().first <= t0)
        {
            imu_predict_buf.pop();
        }
        while (imu_predict_buf.front().first <= t1 && !imu_predict_buf.empty())
        {
            double t = imu_predict_buf.front().first;

            Eigen::Vector3d angular_velocity = imu_predict_buf.front().second.second;

            // Eigen::Vector3d linear_acceleration{
            //     imu_predict_buf.front()->linear_acceleration.x,
            //     imu_predict_buf.front()->linear_acceleration.y,
            //     imu_predict_buf.front()->linear_acceleration.z};

            imu_predict_buf.pop();

            if (first_imu)
            {
                prev_imu_time = t;
                first_imu     = false;
                prev_gyr      = angular_velocity;
                continue;
            }
            double dt     = t - prev_imu_time;
            prev_imu_time = t;

            // Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

            Eigen::Vector3d un_gyr = 0.5 * (prev_gyr + angular_velocity) - latest_Bg;
            // tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

            // Eigen::Vector3d un_acc_1 =
            //     tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

            // un_acc = 0.5 * (un_acc_0 + un_acc_1);

            // tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
            // tmp_V = tmp_V + dt * un_acc;

            // acc_0 = linear_acceleration;
            prev_gyr = angular_velocity;

            // cl
            // Transform the mean angular velocity from the IMU
            // frame to the cam0 frames.
            // Compute the relative rotation.
            Vector3d cam0_angle_axisd = RIC.back().transpose() * un_gyr * dt;
            relative_R *= AngleAxisd(cam0_angle_axisd.norm(), cam0_angle_axisd.normalized())
                              .toRotationMatrix()
                              .transpose();
        }
    }

    return relative_R;
}

void Estimator::predict(double t, const Vector3d &linearAcceleration,
                        const Vector3d &angularVelocity)
{
    if (init_imu)
    {
        latest_time  = t;
        init_imu     = false;
        // [SW1-1872] 첫 샘플은 적분 없이 '직전 샘플'로만 장전 (0벡터로 첫 중점을 만들지 않도록)
        latest_acc_0 = linearAcceleration;
        latest_gyr_0 = angularVelocity;
        return;
    }
    double dt   = t - latest_time;
    latest_time = t;
    // [SW1-1872 결함② 수정] 중점 적분의 '이전 값'을 예측 경로 전용 멤버(latest_acc_0/gyr_0)
    // 에서 읽는다. 기존 코드는 창 처리용 acc_0/gyr_0를 자물쇠 없이 빌려 써(다른 박자의
    // 스레드 B가 갱신) 이전 값이 실제 직전 IMU가 아닐 수 있었다 — HF_TF_PREDICT_DEFECTS.md §4
    hf_predict::midpointStep(latest_P, latest_Q, latest_V, dt, latest_acc_0, latest_gyr_0,
                             linearAcceleration, angularVelocity, latest_Ba, latest_Bg, g);
    latest_acc_0 = linearAcceleration;
    latest_gyr_0 = angularVelocity;
}

bool Estimator::IMUAvailable(double t)
{
    if (!imu_buf.empty() && t <= imu_buf.back().first)
        return true;
    else
        return false;
}

bool Estimator::WheelAvailable(double t)
{
    if (!wheelVelBuf.empty() && t <= wheelVelBuf.back().first)
        return true;
    else
        return false;
}

// [t0, t1] 구간 휠 샘플을 (t, (vel, gyr)) 형태로 추출 (getIMUInterval 미러)
bool Estimator::getWheelInterval(
    double t0, double t1,
    std::vector<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> &wheel_vector)
{
    m_wheel.lock();
    if (wheelVelBuf.empty())
    {
        printf("not receive wheel\n");
        m_wheel.unlock();
        return false;
    }
    if (t1 <= wheelVelBuf.back().first)
    {
        while (wheelVelBuf.front().first <= t0)
        {
            wheelVelBuf.pop();
            wheelGyrBuf.pop();
        }
        while (wheelVelBuf.front().first < t1)
        {
            wheel_vector.emplace_back(
                wheelVelBuf.front().first,
                make_pair(wheelVelBuf.front().second, wheelGyrBuf.front().second));
            wheelVelBuf.pop();
            wheelGyrBuf.pop();
        }
        wheel_vector.emplace_back(
            wheelVelBuf.front().first,
            make_pair(wheelVelBuf.front().second, wheelGyrBuf.front().second));
        m_wheel.unlock();
        return true;
    }
    else
    {
        printf("wait for wheel\n");
        m_wheel.unlock();
        return false;
    }
}

void Estimator::initFirstIMUPose(
    std::vector<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> &imu_vector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    // return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int             n = (int)imu_vector.size();
    for (auto &i : imu_vector)
    {
        averAcc = averAcc + i.second.first;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0  = Utility::g2R(averAcc);
    double   yaw = Utility::R2ypr(R0).x();
    R0           = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0]        = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
}
bool Estimator::getIMUInterval(
    double t0, double t1,
    std::vector<pair<double, pair<Eigen::Vector3d, Eigen::Vector3d>>> &imu_vector)
{
    m_imu.lock();
    if (imu_buf.empty())
    {
        printf("not receive imu\n");
        m_imu.unlock();
        return false;
    }
    if (t1 <= imu_buf.back().first)
    {
        while (imu_buf.front().first <= t0)
        {
            imu_buf.pop();
        }
        while (imu_buf.front().first < t1)
        {
            imu_vector.emplace_back(std::move(imu_buf.front()));
            imu_buf.pop();
        }
        imu_vector.emplace_back(imu_buf.front());
        m_imu.unlock();
        return true;
    }
    else
    {
        printf("wait for imu\n");
        m_imu.unlock();
        return false;
    }
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                    Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w    = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj   = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double   rx       = residual.x();
    double   ry       = residual.y();
    return sqrt(rx * rx + ry * ry);
}

double Estimator::reprojectionError3D(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                      Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                      double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w  = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    return (pts_cj - uvj).norm() / depth;
}

void Estimator::movingConsistencyCheck(set<int> &removeIndex)
{
    // [SW1-1866] 동적 장애물 포렌식(환경변수 게이트, 기본 off) — 판정 오차 분포 집계.
    //   폭주 중 오차가 10px 문턱 '아래'로 기는지(방어 회피), 전부 초과인지(전면 기각) 구분용.
    static const bool dynobs_log = (std::getenv("VINS_DYNOBS_LOG") != nullptr);
    int    mcc_checked = 0, mcc_flagged = 0;
    double mcc_sum_px = 0, mcc_max_px = 0;

    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        double depth = it_per_id.estimated_depth;
        if (depth < 0)
            continue;

        double   err    = 0;
        double   err3D  = 0;
        int      errCnt = 0;
        int      imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                err += reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j],
                                         ric[0], tic[0], depth, pts_i, pts_j);
                // for bleeding points
                err3D += reprojectionError3D(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j],
                                             Ps[imu_j], ric[0], tic[0], depth, pts_i, pts_j);
                errCnt++;
            }
        }
        if (errCnt > 0)
        {
            const double err_px = FOCAL_LENGTH * err / errCnt;
            if (err_px > 10 || err3D / errCnt > 2.0)
            {
                removeIndex.insert(it_per_id.feature_id);
                it_per_id.is_dynamic = true;
            }
            else
            {
                it_per_id.is_dynamic = false;
            }
            if (dynobs_log)
            {
                mcc_checked++;
                if (it_per_id.is_dynamic)
                    mcc_flagged++;
                mcc_sum_px += err_px;
                mcc_max_px = std::max(mcc_max_px, err_px);
            }
        }
    }

    if (dynobs_log && mcc_checked > 0)
        RCLCPP_INFO(rclcpp::get_logger("vins_dynobs_probe"),
                    "[DYNOBS-MCC] t=%.3f checked=%d flagged=%d mean_px=%.2f max_px=%.2f",
                    Headers[WINDOW_SIZE], mcc_checked, mcc_flagged,
                    mcc_sum_px / mcc_checked, mcc_max_px);
}
