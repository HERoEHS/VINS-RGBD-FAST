#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "keyframe/keyframe.h"
#include "pose_graph/pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"

#define SKIP_FIRST_CNT 10

// Global definitions for extern vars declared in utility/parameters.h
camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
Eigen::Matrix<double, 3, 1> ti_d;
Eigen::Matrix<double, 3, 3> qi_d;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_match_img;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
int         VISUALIZATION_SHIFT_X = 0;
int         VISUALIZATION_SHIFT_Y = 0;
std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
int         ROW = 0;
int         COL = 0;
std::string VINS_RESULT_PATH;
int         DEBUG_IMAGE       = 0;
int         FAST_RELOCALIZATION = 0;

class PoseGraphNode : public rclcpp::Node
{
public:
    PoseGraphNode() : Node("pose_graph")
    {
        frame_index    = 0;
        sequence       = 1;
        skip_first_cnt = 0;
        skip_cnt       = 0;
        load_flag      = 0;
        start_flag     = 0;
        SKIP_DIS       = 0;
        last_image_time = -1;
        last_t = Eigen::Vector3d(-100, -100, -100);
        cameraposevisual = new CameraPoseVisualization(1, 0, 0, 1);

        declare_parameter("visualization_shift_x", 0);
        declare_parameter("visualization_shift_y", 0);
        declare_parameter("skip_cnt", 0);
        declare_parameter("skip_dis", 0.0);
        declare_parameter("config_file", std::string(""));

        get_parameter("visualization_shift_x", VISUALIZATION_SHIFT_X);
        get_parameter("visualization_shift_y", VISUALIZATION_SHIFT_Y);
        get_parameter("skip_cnt", SKIP_CNT);
        get_parameter("skip_dis", SKIP_DIS);
        std::string config_file;
        get_parameter("config_file", config_file);

        if (config_file.empty())
            throw std::runtime_error("The required ROS parameter 'config_file' is empty");

        posegraph.registerPub(this);

        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
            throw std::runtime_error("Cannot open pose graph config file: " + config_file);

        double camera_visual_size = fsSettings["visualize_camera_size"];
        cameraposevisual->setScale(camera_visual_size);
        cameraposevisual->setLineWidth(camera_visual_size / 10.0);

        LOOP_CLOSURE = fsSettings["loop_closure"];
        std::string IMAGE_TOPIC;
        int LOAD_PREVIOUS_POSE_GRAPH = 0;

        if (LOOP_CLOSURE)
        {
            ROW = fsSettings["image_height"];
            COL = fsSettings["image_width"];

            std::string pkg_path = ament_index_cpp::get_package_share_directory("pose_graph");
            std::string vocabulary_file = pkg_path + "/support_files/brief_k10L6.bin";
            std::cout << "vocabulary_file: " << vocabulary_file << std::endl;
            posegraph.loadVocabulary(vocabulary_file);

            BRIEF_PATTERN_FILE = pkg_path + "/support_files/brief_pattern.yml";
            std::cout << "BRIEF_PATTERN_FILE: " << BRIEF_PATTERN_FILE << std::endl;
            m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

            fsSettings["image_topic"]          >> IMAGE_TOPIC;
            fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
            fsSettings["output_path"]          >> VINS_RESULT_PATH;
            fsSettings["save_image"]           >> DEBUG_IMAGE;

            cv::Mat cv_qid, cv_tid;
            fsSettings["extrinsicRotation"]    >> cv_qid;
            fsSettings["extrinsicTranslation"] >> cv_tid;
            cv::cv2eigen(cv_qid, qi_d);
            cv::cv2eigen(cv_tid, ti_d);

            int USE_IMU = fsSettings["imu"];
            posegraph.setIMUFlag(USE_IMU);

            VISUALIZE_IMU_FORWARD  = fsSettings["visualize_imu_forward"];
            LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
            FAST_RELOCALIZATION    = fsSettings["fast_relocalization"];
            VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
            std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
            fout.close();

            if (LOAD_PREVIOUS_POSE_GRAPH)
            {
                printf("load pose graph\n");
                m_process.lock();
                posegraph.loadPoseGraph();
                m_process.unlock();
                printf("load pose graph finish\n");
            }
            load_flag = true;
        }

        fsSettings.release();

        pub_match_img = create_publisher<sensor_msgs::msg::Image>("match_image", 100);
        pub_camera_pose_visual = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pose_graph/camera_pose_visual", 100);
        pub_key_odometrys = create_publisher<visualization_msgs::msg::Marker>(
            "/pose_graph/key_odometrys", 100);
        pub_vio_path  = create_publisher<nav_msgs::msg::Path>("/pose_graph/no_loop_path", 100);
        pub_match_points = create_publisher<sensor_msgs::msg::PointCloud>("/pose_graph/match_points", 100);

        sub_imu_forward = create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/imu_propagate", 100,
            std::bind(&PoseGraphNode::imu_forward_callback, this, std::placeholders::_1));
        sub_vio = create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/odometry", 100,
            std::bind(&PoseGraphNode::vio_callback, this, std::placeholders::_1));
        sub_image = create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC, rclcpp::QoS(100).best_effort(), //100,
            std::bind(&PoseGraphNode::image_callback, this, std::placeholders::_1));
        sub_pose = create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/keyframe_pose", 100,
            std::bind(&PoseGraphNode::pose_callback, this, std::placeholders::_1));
        sub_extrinsic = create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/extrinsic", 100,
            std::bind(&PoseGraphNode::extrinsic_callback, this, std::placeholders::_1));
        sub_point = create_subscription<sensor_msgs::msg::PointCloud>(
            "/vins_estimator/keyframe_point", 100,
            std::bind(&PoseGraphNode::point_callback, this, std::placeholders::_1));
        sub_relo_relative_pose = create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/relo_relative_pose", 100,
            std::bind(&PoseGraphNode::relo_relative_pose_callback, this, std::placeholders::_1));

        measurement_process    = std::thread(&PoseGraphNode::process, this);
        keyboard_command_process = std::thread(&PoseGraphNode::command, this);
    }

    ~PoseGraphNode()
    {
        if (measurement_process.joinable())    measurement_process.detach();
        if (keyboard_command_process.joinable()) keyboard_command_process.detach();
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     sub_imu_forward, sub_vio;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_image;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     sub_pose, sub_extrinsic, sub_relo_relative_pose;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_point;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      pub_key_odometrys;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  pub_vio_path;

    std::thread measurement_process, keyboard_command_process;

    std::queue<sensor_msgs::msg::Image::ConstSharedPtr>    image_buf;
    std::queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> point_buf;
    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr>    pose_buf;
    std::queue<Eigen::Vector3d>                             odometry_buf;
    std::mutex m_buf;
    std::mutex m_process;

    int    frame_index;
    int    sequence;
    PoseGraph posegraph;
    int    skip_first_cnt;
    int    SKIP_CNT{};
    int    skip_cnt;
    bool   load_flag;
    bool   start_flag;
    double SKIP_DIS;
    int    VISUALIZE_IMU_FORWARD{};
    int    LOOP_CLOSURE{};

    nav_msgs::msg::Path      no_loop_path;
    CameraPoseVisualization *cameraposevisual;
    Eigen::Vector3d          last_t;
    double                   last_image_time;

    void new_sequence()
    {
        printf("new sequence\n");
        sequence++;
        printf("sequence cnt %d\n", sequence);
        if (sequence > 5)
        {
            RCLCPP_WARN(get_logger(), "only support 5 sequences.");
            rclcpp::shutdown();
        }
        posegraph.posegraph_visualization->reset();
        posegraph.publish();
        m_buf.lock();
        while (!image_buf.empty()) image_buf.pop();
        while (!point_buf.empty()) point_buf.pop();
        while (!pose_buf.empty())  pose_buf.pop();
        while (!odometry_buf.empty()) odometry_buf.pop();
        m_buf.unlock();
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
    {
        if (!LOOP_CLOSURE) return;
        m_buf.lock();
        image_buf.push(image_msg);
        m_buf.unlock();

        double t = rclcpp::Time(image_msg->header.stamp).seconds();
        if (last_image_time == -1)
            last_image_time = t;
        else if (t - last_image_time > 1.0 || t < last_image_time)
        {
            RCLCPP_WARN(get_logger(), "image discontinue! detect a new sequence!");
            new_sequence();
        }
        last_image_time = t;
    }

    void point_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &point_msg)
    {
        if (!LOOP_CLOSURE) return;
        m_buf.lock();
        point_buf.push(point_msg);
        m_buf.unlock();
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
    {
        if (!LOOP_CLOSURE) return;
        m_buf.lock();
        pose_buf.push(pose_msg);
        m_buf.unlock();
    }

    void imu_forward_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &forward_msg)
    {
        if (!VISUALIZE_IMU_FORWARD) return;
        Eigen::Vector3d  vio_t(forward_msg->pose.pose.position.x,
                               forward_msg->pose.pose.position.y,
                               forward_msg->pose.pose.position.z);
        Eigen::Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio * vio_q;
        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        Eigen::Vector3d    vio_t_cam = vio_t + vio_q * tic;
        Eigen::Quaterniond vio_q_cam = vio_q * Eigen::Quaterniond(qic);

        cameraposevisual->reset();
        cameraposevisual->add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual->publish_by(pub_camera_pose_visual, forward_msg->header);
    }

    void relo_relative_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
    {
        Eigen::Vector3d relative_t(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
        Eigen::Quaterniond relative_q;
        relative_q.w() = pose_msg->pose.pose.orientation.w;
        relative_q.x() = pose_msg->pose.pose.orientation.x;
        relative_q.y() = pose_msg->pose.pose.orientation.y;
        relative_q.z() = pose_msg->pose.pose.orientation.z;
        double relative_yaw = pose_msg->twist.twist.linear.x;
        int    index        = pose_msg->twist.twist.linear.y;

        Eigen::Matrix<double, 8, 1> loop_info;
        loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
            relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(), relative_yaw;
        posegraph.updateKeyFrameLoop(index, loop_info);
    }

    void vio_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
    {
        Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x,
                              pose_msg->pose.pose.position.y,
                              pose_msg->pose.pose.position.z);
        Eigen::Quaterniond vio_q;
        vio_q.w() = pose_msg->pose.pose.orientation.w;
        vio_q.x() = pose_msg->pose.pose.orientation.x;
        vio_q.y() = pose_msg->pose.pose.orientation.y;
        vio_q.z() = pose_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio * vio_q;
        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        Eigen::Vector3d    vio_t_cam = vio_t + vio_q * tic;
        Eigen::Quaterniond vio_q_cam = vio_q * Eigen::Quaterniond(qic);

        if (!VISUALIZE_IMU_FORWARD)
        {
            cameraposevisual->reset();
            cameraposevisual->add_pose(vio_t_cam, vio_q_cam);
            cameraposevisual->publish_by(pub_camera_pose_visual, pose_msg->header);
        }

        odometry_buf.push(vio_t_cam);
        if (odometry_buf.size() > 10)
            odometry_buf.pop();

        visualization_msgs::msg::Marker key_odometrys;
        key_odometrys.header          = pose_msg->header;
        key_odometrys.header.frame_id = "map";
        key_odometrys.ns              = "key_odometrys";
        key_odometrys.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
        key_odometrys.action          = visualization_msgs::msg::Marker::ADD;
        key_odometrys.pose.orientation.w = 1.0;
        key_odometrys.lifetime        = rclcpp::Duration(0, 0);
        key_odometrys.id              = 0;
        key_odometrys.scale.x         = 0.1;
        key_odometrys.scale.y         = 0.1;
        key_odometrys.scale.z         = 0.1;
        key_odometrys.color.r         = 1.0;
        key_odometrys.color.a         = 1.0;

        for (unsigned int i = 0; i < odometry_buf.size(); i++)
        {
            geometry_msgs::msg::Point pose_marker;
            Eigen::Vector3d           vio_t_tmp = odometry_buf.front();
            odometry_buf.pop();
            pose_marker.x = vio_t_tmp.x();
            pose_marker.y = vio_t_tmp.y();
            pose_marker.z = vio_t_tmp.z();
            key_odometrys.points.push_back(pose_marker);
            odometry_buf.push(vio_t_tmp);
        }
        pub_key_odometrys->publish(key_odometrys);

        if (!LOOP_CLOSURE)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header          = pose_msg->header;
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = vio_t.x();
            pose_stamped.pose.position.y = vio_t.y();
            pose_stamped.pose.position.z = vio_t.z();
            no_loop_path.header          = pose_msg->header;
            no_loop_path.header.frame_id = "map";
            no_loop_path.poses.push_back(pose_stamped);
            pub_vio_path->publish(no_loop_path);
        }
    }

    void extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
    {
        m_process.lock();
        tic = Eigen::Vector3d(pose_msg->pose.pose.position.x,
                              pose_msg->pose.pose.position.y,
                              pose_msg->pose.pose.position.z);
        qic = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                                 pose_msg->pose.pose.orientation.x,
                                 pose_msg->pose.pose.orientation.y,
                                 pose_msg->pose.pose.orientation.z)
                  .toRotationMatrix();
        m_process.unlock();
    }

    [[noreturn]] void process()
    {
        if (!LOOP_CLOSURE)
            while (true) std::this_thread::sleep_for(std::chrono::seconds(1));

        while (true)
        {
            sensor_msgs::msg::Image::ConstSharedPtr    image_msg = nullptr;
            sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg = nullptr;
            nav_msgs::msg::Odometry::ConstSharedPtr    pose_msg  = nullptr;

            m_buf.lock();
            if (!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
            {
                double t_image = rclcpp::Time(image_buf.front()->header.stamp).seconds();
                double t_point = rclcpp::Time(point_buf.front()->header.stamp).seconds();
                double t_pose  = rclcpp::Time(pose_buf.front()->header.stamp).seconds();

                if (t_image > t_pose)
                {
                    pose_buf.pop();
                    printf("throw pose at beginning\n");
                }
                else if (t_image > t_point)
                {
                    point_buf.pop();
                    printf("throw point at beginning\n");
                }
                else if (rclcpp::Time(image_buf.back()->header.stamp).seconds() >= t_pose &&
                         rclcpp::Time(point_buf.back()->header.stamp).seconds() >= t_pose)
                {
                    pose_msg = pose_buf.front();
                    pose_buf.pop();
                    while (!pose_buf.empty()) pose_buf.pop();

                    while (rclcpp::Time(image_buf.front()->header.stamp).seconds() < t_pose)
                        image_buf.pop();
                    image_msg = image_buf.front();
                    image_buf.pop();

                    while (rclcpp::Time(point_buf.front()->header.stamp).seconds() < t_pose)
                        point_buf.pop();
                    point_msg = point_buf.front();
                    point_buf.pop();
                }
            }
            m_buf.unlock();

            if (pose_msg != nullptr)
            {
                if (skip_first_cnt < SKIP_FIRST_CNT) { skip_first_cnt++; continue; }
                if (skip_cnt < SKIP_CNT)              { skip_cnt++;       continue; }
                skip_cnt = 0;

                cv::Mat image =
                    cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8)->image;

                Eigen::Vector3d T(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
                Eigen::Matrix3d R = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z)
                                        .toRotationMatrix();

                if ((T - last_t).norm() > SKIP_DIS)
                {
                    std::vector<cv::Point3f> point_3d;
                    std::vector<cv::Point2f> point_2d_uv, point_2d_normal;
                    std::vector<double>      point_id;

                    for (unsigned int i = 0; i < point_msg->points.size(); i++)
                    {
                        cv::Point3f p_3d;
                        p_3d.x = point_msg->points[i].x;
                        p_3d.y = point_msg->points[i].y;
                        p_3d.z = point_msg->points[i].z;
                        point_3d.push_back(p_3d);

                        cv::Point2f p_2d_uv, p_2d_normal;
                        p_2d_normal.x = point_msg->channels[i].values[0];
                        p_2d_normal.y = point_msg->channels[i].values[1];
                        p_2d_uv.x     = point_msg->channels[i].values[2];
                        p_2d_uv.y     = point_msg->channels[i].values[3];
                        double p_id   = point_msg->channels[i].values[4];
                        point_2d_normal.push_back(p_2d_normal);
                        point_2d_uv.push_back(p_2d_uv);
                        point_id.push_back(p_id);
                    }

                    KeyFrame *keyframe = new KeyFrame(
                        rclcpp::Time(pose_msg->header.stamp).seconds(),
                        frame_index, T, R, image,
                        point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                    m_process.lock();
                    start_flag = 1;
                    posegraph.addKeyFrame(keyframe, 1);
                    m_process.unlock();
                    frame_index++;
                    last_t = T;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    [[noreturn]] void command()
    {
        if (!LOOP_CLOSURE)
            while (true) std::this_thread::sleep_for(std::chrono::seconds(1));

        while (1)
        {
            char c = getchar();
            if (c == 's')
            {
                m_process.lock();
                posegraph.savePoseGraph();
                m_process.unlock();
                printf("save pose graph finish\n");
                rclcpp::shutdown();
            }
            if (c == 'n')
                new_sequence();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseGraphNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
