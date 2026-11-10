// [SW1-1959 도킹 위치 나이] 연결부 — hf_image_timeout_s 읽기(기본값·끔·음수·시작 로그)와,
// 실물 Estimator::inputIMU 가 영상이 끊기면 predict 는 계속하고 imu_propagate 발행만 건너뛰는지.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include "scoped_temp_dir.h"
#include "../src/estimator/estimator.h"
#include "../src/utility/parameters.h"
#include "../src/utility/visualization.h"

namespace
{

const std::string kVinsRoot = VINS_SOURCE_ROOT;  // CMake 가 주입 (VINS-RGBD-FAST 루트)
const std::string kVioYaml  = kVinsRoot + "/config/elp_stereo_edie/vio_edie.yaml";
const char       *kKey      = "hf_image_timeout_s:";

std::unique_ptr<vins_test::ScopedTempDir> g_tmp;  // main 이 만들고 지운다

std::mutex  g_log_mutex;
std::string g_startup_line;

void CaptureLog(const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
                const char *format, va_list *args)
{
    char    buf[4096];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(buf, sizeof(buf), format, copy);
    va_end(copy);
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (std::string(buf).rfind("HF_IMAGE_TIMEOUT_S", 0) == 0)
        g_startup_line = buf;
}

// 배포 yaml 에서 키 줄을 지운 사본을 만들고, value 가 있으면 그 값으로 다시 붙인다.
std::string ConfigWith(const std::string &name, const std::string &value)
{
    std::ifstream     in(kVioYaml);
    std::ostringstream out;
    std::string       line;
    while (std::getline(in, line))
        if (line.rfind(kKey, 0) != 0)
            out << line << "\n";
    if (!value.empty())
        out << kKey << " " << value << "\n";
    const std::string path = g_tmp->path() + name;
    std::ofstream(path) << out.str();
    return path;
}

void ReadConfig(const std::string &config_file)
{
    // readParameters 는 전역에 쌓는다 — 시험마다 비우고 다시 읽는다. 앞 값이 남아 통과하지 않게 일부러 어긋난 값을 넣어 둔다.
    RIC.clear();
    TIC.clear();
    HF_IMAGE_TIMEOUT_S = 123.0;
    g_startup_line.clear();
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("config_file", config_file),
                              rclcpp::Parameter("vins_folder", kVinsRoot + "/"),
                              rclcpp::Parameter("extrinsic_file", ""),
                              rclcpp::Parameter("camera_calib_file", ""),
                              rclcpp::Parameter("camera_calib_from_robot", false),
                              rclcpp::Parameter("camera_calib_default_file", "")});
    auto node = std::make_shared<rclcpp::Node>("vins_hf_gate_param_test", opts);
    readParameters(node.get());
}

}  // namespace

TEST(HfTimeoutParam, DeployedConfigIsOneSecond)
{
    ReadConfig(kVioYaml);
    EXPECT_DOUBLE_EQ(HF_IMAGE_TIMEOUT_S, 1.0);
    EXPECT_NE(g_startup_line.find("활성"), std::string::npos) << g_startup_line;
}

TEST(HfTimeoutParam, MissingKeyDefaultsToOn)
{
    // 로봇 /opt 사본 yaml 에는 키가 없을 수 있다 — 그래도 켜져 있어야 한다.
    ReadConfig(ConfigWith("no_key.yaml", ""));
    EXPECT_DOUBLE_EQ(HF_IMAGE_TIMEOUT_S, 1.0);
    EXPECT_NE(g_startup_line.find("활성"), std::string::npos) << g_startup_line;
}

TEST(HfTimeoutParam, ZeroMeansOff)
{
    ReadConfig(ConfigWith("zero.yaml", "0.0"));
    EXPECT_DOUBLE_EQ(HF_IMAGE_TIMEOUT_S, 0.0);
    EXPECT_NE(g_startup_line.find("끔"), std::string::npos) << g_startup_line;
}

TEST(HfTimeoutParam, NegativeIsTreatedAsOff)
{
    ReadConfig(ConfigWith("negative.yaml", "-2.5"));
    EXPECT_DOUBLE_EQ(HF_IMAGE_TIMEOUT_S, 0.0);
    EXPECT_NE(g_startup_line.find("끔"), std::string::npos) << g_startup_line;
}

TEST(HfGateWiring, InputImuSkipsOnlyPublishWhileCut)
{
    ReadConfig(kVioYaml);
    auto node = std::make_shared<rclcpp::Node>("vins_hf_gate_wiring_test");
    registerPub(node.get());

    long received = 0;
    auto sub      = node->create_subscription<nav_msgs::msg::Odometry>(
        "/vins_estimator/imu_propagate", 50,
        [&received](nav_msgs::msg::Odometry::ConstSharedPtr) { ++received; });

    Estimator est;
    est.setParameter();
    est.solver_flag = Estimator::NON_LINEAR;

    double          t = 100.0;
    const Vector3d  acc(0.1, 0.0, 9.8);
    const Vector3d  gyr(0.0, 0.0, 0.01);
    const auto feed = [&](int n)
    {
        for (int i = 0; i < n; ++i, t += 0.005)
            est.inputIMU(t, acc, gyr);
    };
    const auto spinFor = [&](std::chrono::milliseconds d)
    {
        const auto end = std::chrono::steady_clock::now() + d;
        while (std::chrono::steady_clock::now() < end)
        {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    };
    // 발행이 살아 있으면 구독 연결(디스커버리)을 기다리는 동안 언젠가 받는다.
    const auto feedUntilReceived = [&]()
    {
        received = 0;
        for (int i = 0; i < 1000 && received == 0; ++i)
        {
            feed(1);
            spinFor(std::chrono::milliseconds(10));
        }
        return received > 0;
    };

    // 영상을 한 번도 못 받았으면 막지 않는다.
    ASSERT_TRUE(feedUntilReceived()) << "아직 영상 없음: 발행돼야 한다";
    EXPECT_FALSE(est.hf_cut_.isCut());

    // 방금 받았으면 발행.
    est.last_image_arrival_.store(hf_image_gate::Clock::now());
    EXPECT_TRUE(feedUntilReceived()) << "신선: 발행돼야 한다";
    EXPECT_FALSE(est.hf_cut_.isCut());

    // 5 s 끊김 → 발행은 0, predict 는 계속.
    est.last_image_arrival_.store(hf_image_gate::Clock::now() - std::chrono::seconds(5));
    feed(1);
    spinFor(std::chrono::milliseconds(300));  // 앞서 나간 메시지를 비운다
    received                  = 0;
    const double   time_before = est.latest_time;
    const Vector3d p_before    = est.latest_P;
    feed(20);
    spinFor(std::chrono::milliseconds(300));
    EXPECT_EQ(received, 0) << "끊긴 동안 imu_propagate 가 나가면 안 된다";
    EXPECT_GT(est.latest_time, time_before) << "predict 는 계속 돌아야 한다";
    EXPECT_NE((est.latest_P - p_before).norm(), 0.0) << "predict 는 계속 돌아야 한다";
    EXPECT_TRUE(est.hf_cut_.isCut());
    EXPECT_EQ(est.hf_cut_.cutCount(), 1);
    EXPECT_EQ(est.hf_cut_.blockedSamples(), 21);

    // 복귀 → 다시 발행.
    est.last_image_arrival_.store(hf_image_gate::Clock::now());
    EXPECT_TRUE(feedUntilReceived()) << "복귀: 발행이 재개돼야 한다";
    EXPECT_FALSE(est.hf_cut_.isCut());

    // 재시작 중(INITIAL)엔 predict·발행 모두 없다 — 기존 게이트 유지.
    est.solver_flag = Estimator::INITIAL;
    spinFor(std::chrono::milliseconds(300));
    received = 0;
    feed(5);
    spinFor(std::chrono::milliseconds(300));
    EXPECT_EQ(received, 0);
    est.solver_flag = Estimator::NON_LINEAR;

    // clearState 는 수신 시각을 지우지 않는다 — 지우면 재시작 직후 "아직 못 받음"으로 돌아가 차단이 풀린다.
    const hf_image_gate::TimePoint arrival = hf_image_gate::Clock::now() - std::chrono::seconds(5);
    est.last_image_arrival_.store(arrival);
    est.clearState();
    EXPECT_TRUE(est.last_image_arrival_.load() == arrival);
    est.setParameter();
    est.solver_flag = Estimator::NON_LINEAR;

    // 끔(0): 오래 끊겨도 기존처럼 발행.
    HF_IMAGE_TIMEOUT_S = 0.0;
    EXPECT_TRUE(feedUntilReceived()) << "끔: 발행돼야 한다";
    EXPECT_FALSE(est.hf_cut_.isCut());
}

int main(int argc, char **argv)
{
    // 기록 폴더를 시험용 임시 폴더로 — 홈(~/.edie/logs/vins)·/tmp 에 파일을 남기지 않게(끝나면 폴더째 지움).
    g_tmp = std::make_unique<vins_test::ScopedTempDir>();
    setenv("EDIE_VINS_OUTPUT_DIR", g_tmp->path().c_str(), 1);
    rclcpp::init(argc, argv);
    rcutils_logging_set_output_handler(CaptureLog);
    ::testing::InitGoogleTest(&argc, argv);
    const int rc = RUN_ALL_TESTS();
    rclcpp::shutdown();
    g_tmp.reset();
    // registerPub 가 채운 visualization.cpp 전역 publisher 는 DDS 전역 정리 뒤에 소멸하며 abort 한다(시험 결과와 무관한
    //   종료 순서 문제). 이 시험만 registerPub 를 부르므로 전역 소멸을 건너뛰고 끝낸다.
    std::fflush(nullptr);
    std::_Exit(rc);
}
