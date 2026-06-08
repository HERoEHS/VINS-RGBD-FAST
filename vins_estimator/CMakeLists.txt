cmake_minimum_required(VERSION 3.8)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_model REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
  src
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /home/higony/Sophus
)

set(VINS_LIB_SRCS
  src/utility/parameters.cpp
  src/estimator/estimator.cpp
  src/feature_manager/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
  src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
  src/feature_tracker/feature_tracker.cpp
)

add_library(vins_lib ${VINS_LIB_SRCS})
ament_target_dependencies(vins_lib
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  tf2 tf2_ros cv_bridge image_transport camera_model ament_index_cpp
)
target_link_libraries(vins_lib
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Boost::filesystem Boost::program_options Boost::system
)

add_executable(vins_estimator_node src/estimator_nodelet.cpp)
ament_target_dependencies(vins_estimator_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  tf2 tf2_ros cv_bridge image_transport camera_model ament_index_cpp
)
target_link_libraries(vins_estimator_node vins_lib)

install(TARGETS vins_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

ament_package()
