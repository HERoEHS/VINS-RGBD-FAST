#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

extern camodocal::CameraPtr m_camera;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;
extern Eigen::Matrix<double, 3, 1> ti_d;
extern Eigen::Matrix<double, 3, 3> qi_d;
extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_match_img;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;
extern std::string BRIEF_PATTERN_FILE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern int ROW;
extern int COL;
extern std::string VINS_RESULT_PATH;
extern int DEBUG_IMAGE;
extern int FAST_RELOCALIZATION;
