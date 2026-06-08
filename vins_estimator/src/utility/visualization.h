#pragma once

#include "../estimator/estimator.h"
#include "CameraPoseVisualization.h"
#include "parameters.h"
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <cv_bridge/cv_bridge.h>

extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
extern rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_cloud, pub_map;
extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ref_pose, pub_cur_pose;
extern nav_msgs::msg::Path path;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose_graph;
extern int IMAGE_ROW, IMAGE_COL;

void registerPub(rclcpp::Node* node);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q,
                       const Eigen::Vector3d &V, const double &t);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubIMUPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubTrackImg(const cv_bridge::CvImageConstPtr &img_ptr);

void pubTrackImg(const sensor_msgs::msg::Image::SharedPtr &img_msg);

void pubTrackImg(const cv::Mat &img);
