#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <thread>
#include <cassert>

#include "estimator/estimator.h"
#include "feature_tracker/feature_tracker.h"
#include "sensor_msgs/image_encodings.hpp"
#include "utility/parameters.h"
#include "utility/tic_toc.h"
#include "utility/visualization.h"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point32.hpp>

class EstimatorNode : public rclcpp::Node
{
public:
    EstimatorNode() : Node("vins_estimator")
    {
        readParameters(this);
        estimator.setParameter();

        RCLCPP_WARN(get_logger(), "waiting for image, semantic and imu...");

        registerPub(this);

        // 카메라/IMU 센서는 BEST_EFFORT 로 발행된다(edie_vision, ros2 bag 동일).
        // 기본 QoS(RELIABLE) 로 구독하면 호환 불가로 메시지를 한 건도 못 받으므로
        // BEST_EFFORT + 깊은 큐(KeepLast 1000) 로 맞춘다.
        const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort();

        sub_image = create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC, sensor_qos,
            std::bind(&EstimatorNode::image_callback, this, std::placeholders::_1));
        sub_depth = create_subscription<sensor_msgs::msg::Image>(
            DEPTH_TOPIC, sensor_qos,
            std::bind(&EstimatorNode::depth_callback, this, std::placeholders::_1));

        if (USE_IMU)
            sub_imu = create_subscription<sensor_msgs::msg::Imu>(
                IMU_TOPIC, sensor_qos,
                std::bind(&EstimatorNode::imu_callback, this, std::placeholders::_1));

        sub_relo_points = create_subscription<sensor_msgs::msg::PointCloud>(
            "/pose_graph/match_points", 10,
            std::bind(&EstimatorNode::relocalization_callback, this, std::placeholders::_1));

        dura = std::chrono::milliseconds(2);

        trackThread   = std::thread(&EstimatorNode::process_tracker, this);
        processThread = std::thread(&EstimatorNode::process, this);
    }

    ~EstimatorNode()
    {
        if (trackThread.joinable())   trackThread.detach();
        if (processThread.joinable()) processThread.detach();
    }

private:
    Estimator estimator;

    std::thread trackThread, processThread;
    std::chrono::milliseconds dura;
    std::condition_variable   con_tracker;
    std::condition_variable   con_estimator;
    std::mutex                m_feature;
    std::mutex                m_backend;
    std::mutex                m_buf;
    std::mutex                m_vis;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr     sub_relo_points;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          sub_image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          sub_depth;

    queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;
    queue<sensor_msgs::msg::Image::ConstSharedPtr> depth_buf;
    queue<std::pair<std::pair<std_msgs::msg::Header, sensor_msgs::msg::Image::ConstSharedPtr>,
                    std::map<int, Eigen::Matrix<double, 7, 1>>>>
        feature_buf;
    queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> relo_buf;
    queue<std::pair<std_msgs::msg::Header, cv::Mat>> vis_img_buf;

    bool init_feature = false;
    bool init_pub     = false;

    bool   first_image_flag = true;
    double first_image_time = 0;
    double last_image_time  = 0;
    int    pub_count        = 1;
    int    input_count      = 0;

    double last_imu_t = 0;

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {
        if (!imu_msg) return;
        double t = rclcpp::Time(imu_msg->header.stamp).seconds();
        if (t <= last_imu_t)
        {
            RCLCPP_WARN(get_logger(), "imu message in disorder! %f", t);
            return;
        }
        last_imu_t = t;
        Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                            imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z);
        Eigen::Vector3d gyr(imu_msg->angular_velocity.x,
                            imu_msg->angular_velocity.y,
                            imu_msg->angular_velocity.z);
        estimator.inputIMU(last_imu_t, acc, gyr);
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &color_msg)
    {
        m_buf.lock();
        img_buf.emplace(color_msg);
        m_buf.unlock();
        con_tracker.notify_one();
    }

    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
    {
        m_buf.lock();
        depth_buf.emplace(depth_msg);
        m_buf.unlock();
        con_tracker.notify_one();
    }

    void relocalization_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &points_msg)
    {
        m_buf.lock();
        relo_buf.push(points_msg);
        m_buf.unlock();
    }

    void visualizeFeatureFilter(const std::map<int, Eigen::Matrix<double, 7, 1>> &features,
                                double feature_time)
    {
        cv::Mat vis_img;
        m_vis.lock();
        while (!vis_img_buf.empty())
        {
            double buf_stamp = rclcpp::Time(vis_img_buf.front().first.stamp).seconds();
            if (buf_stamp == feature_time)
            {
                vis_img = vis_img_buf.front().second;
                vis_img_buf.pop();
                break;
            }
            else if (buf_stamp < feature_time)
                vis_img_buf.pop();
            else
            {
                m_vis.unlock();
                return;
            }
        }
        m_vis.unlock();

        for (auto &feature : features)
            cv::circle(vis_img, cv::Point(feature.second[3], feature.second[4]), 5,
                       cv::Scalar(0, 255, 255), 2);
        pubTrackImg(vis_img);
    }

    [[noreturn]] void process_tracker()
    {
        while (1)
        {
            {
                sensor_msgs::msg::Image::ConstSharedPtr color_msg = nullptr;
                sensor_msgs::msg::Image::ConstSharedPtr depth_msg = nullptr;

                std::unique_lock<std::mutex> locker(m_buf);
                while (img_buf.empty() || depth_buf.empty())
                    con_tracker.wait(locker);

                double time_color = rclcpp::Time(img_buf.front()->header.stamp).seconds();
                double time_depth = rclcpp::Time(depth_buf.front()->header.stamp).seconds();

                if (time_color < time_depth - 0.003)
                {
                    img_buf.pop();
                    RCLCPP_DEBUG(get_logger(), "throw color");
                }
                else if (time_color > time_depth + 0.003)
                {
                    depth_buf.pop();
                    RCLCPP_DEBUG(get_logger(), "throw depth");
                }
                else
                {
                    color_msg = img_buf.front(); img_buf.pop();
                    depth_msg = depth_buf.front(); depth_buf.pop();
                }
                locker.unlock();

                if (color_msg == nullptr || depth_msg == nullptr)
                {
                    RCLCPP_DEBUG(get_logger(), "time_color = %f, time_depth = %f", time_color, time_depth);
                    continue;
                }

                if (first_image_flag)
                {
                    first_image_flag = false;
                    first_image_time = time_color;
                    last_image_time  = time_color;
                    continue;
                }

                if (time_color - last_image_time > 1.0 || time_color < last_image_time)
                {
                    RCLCPP_WARN(get_logger(), "image discontinue! reset the feature tracker!");
                    first_image_flag = true;
                    last_image_time  = 0;
                    pub_count        = 1;

                    RCLCPP_WARN(get_logger(), "restart the estimator!");
                    m_feature.lock();
                    while (!feature_buf.empty()) feature_buf.pop();
                    m_feature.unlock();
                    m_backend.lock();
                    estimator.clearState();
                    estimator.setParameter();
                    m_backend.unlock();
                    last_imu_t = 0;
                    continue;
                }

                if (round(1.0 * input_count / (time_color - first_image_time)) > FRONTEND_FREQ)
                {
                    RCLCPP_DEBUG(get_logger(), "Skip this frame.%f",
                                 1.0 * input_count / (time_color - first_image_time));
                    continue;
                }
                ++input_count;

                if (round(1.0 * pub_count / (time_color - first_image_time)) <= FREQ)
                {
                    PUB_THIS_FRAME = true;
                    if (abs(1.0 * pub_count / (time_color - first_image_time) - FREQ) < 0.01 * FREQ)
                    {
                        first_image_time = time_color;
                        pub_count        = 0;
                        input_count      = 0;
                    }
                }
                else
                    PUB_THIS_FRAME = false;

                TicToc t_r;
                cv_bridge::CvImageConstPtr ptr;
                if (color_msg->encoding == "8UC1")
                {
                    sensor_msgs::msg::Image img;
                    img.header       = color_msg->header;
                    img.height       = color_msg->height;
                    img.width        = color_msg->width;
                    img.is_bigendian = color_msg->is_bigendian;
                    img.step         = color_msg->step;
                    img.data         = color_msg->data;
                    img.encoding     = "mono8";
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
                }
                else
                    ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::MONO8);

                if (USE_IMU)
                {
                    Eigen::Matrix3d &&relative_R =
                        estimator.predictMotion(last_image_time, time_color + estimator.td);
                    estimator.featureTracker.readImage(ptr->image, time_color, relative_R);
                }
                else
                    estimator.featureTracker.readImage(ptr->image, time_color);

                last_image_time = time_color;

                for (unsigned int i = 0;; i++)
                {
                    bool completed = false;
                    completed |= estimator.featureTracker.updateID(i);
                    if (!completed) break;
                }

                if (PUB_THIS_FRAME)
                {
                    pub_count++;

                    std_msgs::msg::Header feature_header = color_msg->header;
                    std::map<int, Eigen::Matrix<double, 7, 1>> image;
                    auto &un_pts       = estimator.featureTracker.cur_un_pts;
                    auto &cur_pts      = estimator.featureTracker.cur_pts;
                    auto &ids          = estimator.featureTracker.ids;
                    auto &pts_velocity = estimator.featureTracker.pts_velocity;

                    for (unsigned int j = 0; j < ids.size(); j++)
                    {
                        if (estimator.featureTracker.track_cnt[j] > 1)
                        {
                            int    p_id       = ids[j];
                            double x          = un_pts[j].x;
                            double y          = un_pts[j].y;
                            double z          = 1;
                            int    v          = p_id * NUM_OF_CAM + 0.5;
                            int    feature_id = v / NUM_OF_CAM;
                            double p_u        = cur_pts[j].x;
                            double p_v        = cur_pts[j].y;
                            double velocity_x = pts_velocity[j].x;
                            double velocity_y = pts_velocity[j].y;

                            assert(z == 1);
                            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                            image[feature_id] = xyz_uv_velocity;
                        }
                    }

                    if (!init_pub)
                    {
                        init_pub = true;
                    }
                    else
                    {
                        if (!init_feature)
                        {
                            init_feature = true;
                            continue;
                        }
                        if (!image.empty())
                        {
                            m_feature.lock();
                            feature_buf.push(
                                std::make_pair(std::make_pair(feature_header, depth_msg), std::move(image)));
                            m_feature.unlock();
                            con_estimator.notify_one();
                        }
                        else
                        {
                            first_image_time = time_color;
                            pub_count        = 0;
                            input_count      = 0;
                            continue;
                        }
                    }

                    if (SHOW_TRACK)
                    {
                        cv::Mat show_img = ptr->image;
                        ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                        cv::Mat stereo_img = ptr->image;
                        cv::Mat tmp_img    = stereo_img.rowRange(0, ROW);
                        cv::cvtColor(show_img, tmp_img, cv::COLOR_GRAY2RGB);

                        for (unsigned int j = 0; j < estimator.featureTracker.cur_pts.size(); j++)
                        {
                            if (estimator.featureTracker.track_cnt[j] > 1)
                            {
                                double len = std::min(
                                    1.0, 1.0 * estimator.featureTracker.track_cnt[j] / WINDOW_SIZE);
                                cv::circle(tmp_img, estimator.featureTracker.cur_pts[j], 5,
                                           cv::Scalar(255 * (1 - len), 0, 255 * len), -1);
                            }
                        }
                        if (USE_IMU)
                        {
                            for (auto &predict_pt : estimator.featureTracker.predict_pts)
                                cv::circle(tmp_img, predict_pt, 2, cv::Scalar(0, 255, 0), -1);
                        }

                        m_vis.lock();
                        vis_img_buf.push(std::make_pair(feature_header, tmp_img));
                        m_vis.unlock();
                    }
                }
                static double whole_process_time = 0;
                static size_t cnt_frame          = 0;
                ++cnt_frame;
                whole_process_time += t_r.toc();
                RCLCPP_DEBUG(get_logger(), "average feature tracking costs: %f", whole_process_time / cnt_frame);
            }
            std::this_thread::sleep_for(dura);
        }
    }

    [[noreturn]] void process()
    {
        while (true)
        {
            std::unique_lock<std::mutex> locker(m_feature);
            while (feature_buf.empty())
                con_estimator.wait(locker);

            auto feature_msg = std::move(feature_buf.front());
            feature_buf.pop();
            locker.unlock();

            TicToc t_backend;
            m_backend.lock();

            sensor_msgs::msg::PointCloud::ConstSharedPtr relo_msg = nullptr;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }

            if (relo_msg != nullptr)
            {
                std::vector<Eigen::Vector3d> match_points;
                double frame_stamp = rclcpp::Time(relo_msg->header.stamp).seconds();
                for (auto point : relo_msg->points)
                {
                    Eigen::Vector3d u_v_id;
                    u_v_id.x() = point.x;
                    u_v_id.y() = point.y;
                    u_v_id.z() = point.z;
                    match_points.push_back(u_v_id);
                }
                Eigen::Vector3d    relo_t(relo_msg->channels[0].values[0],
                                          relo_msg->channels[0].values[1],
                                          relo_msg->channels[0].values[2]);
                Eigen::Quaterniond relo_q(relo_msg->channels[0].values[3],
                                          relo_msg->channels[0].values[4],
                                          relo_msg->channels[0].values[5],
                                          relo_msg->channels[0].values[6]);
                Eigen::Matrix3d    relo_r = relo_q.toRotationMatrix();
                int frame_index           = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            cv::Mat depth_img;
            if (feature_msg.first.second == nullptr)
            {
                depth_img = cv::Mat(ROW, COL, CV_16UC1, cv::Scalar(0));
            }
            else
            {
                const auto &enc = feature_msg.first.second->encoding;
                if (enc == "mono16" || enc == "16UC1")
                    depth_img = cv_bridge::toCvShare(feature_msg.first.second)->image;
                else if (enc == "32FC1")
                {
                    cv::Mat depth_32fc1 = cv_bridge::toCvShare(feature_msg.first.second)->image;
                    depth_32fc1.convertTo(depth_img, CV_16UC1, 1000);
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "Unknown depth encoding: %s", enc.c_str());
                    depth_img = cv::Mat(ROW, COL, CV_16UC1, cv::Scalar(0));
                }
            }
            estimator.f_manager.inputDepth(depth_img);

            double feature_time = rclcpp::Time(feature_msg.first.first.stamp).seconds();

            TicToc t_processImage;
            estimator.processImage(feature_msg.second, feature_msg.first.first);

            std_msgs::msg::Header header = feature_msg.first.first;
            header.frame_id              = "map";
            pubOdometry(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != nullptr)
                pubRelocalization(estimator);

            m_backend.unlock();

            if (SHOW_TRACK)
            {
                pubKeyPoses(estimator, header);
                pubCameraPose(estimator, header);
                pubPointCloud(estimator, header);
                visualizeFeatureFilter(feature_msg.second, feature_time);
            }

            static int    cnt_frame          = 0;
            static double whole_process_time = 0;
            double        per_process_time   = t_backend.toc();
            cnt_frame++;
            whole_process_time += per_process_time;
            printStatistics(estimator, per_process_time);
            RCLCPP_DEBUG(get_logger(), "average backend costs: %f", whole_process_time / cnt_frame);
            std::this_thread::sleep_for(dura);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
