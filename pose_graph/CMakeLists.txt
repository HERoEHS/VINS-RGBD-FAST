cmake_minimum_required(VERSION 3.8)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# === OpenCV 선점 힌트(SW1-1837) — ★반드시 모든 find_package 보다 먼저, 교차 빌드에선 자동으로 끔(SW1-1959) ===
#   Ubuntu 에선 시스템 OpenCV 를 OpenCV_DIR 로 먼저 박아 /usr/local 커스텀 판과 두 판 로드(ABI 충돌)를 막고,
#   Yocto 교차 빌드에선 호스트 /usr/lib 를 읽지 않고 sysroot 에 맡긴다. 자세한 이유는 cmake/OpenCvSystemHint.cmake.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/OpenCvSystemHint.cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

# OpenCV_DIR은 위(모든 find_package 이전)에서 시스템 경로로 선점됨(SW1-1837). 여기선 그대로 사용.
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
  src
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(POSE_GRAPH_LIB_SRCS
  src/pose_graph/pose_graph.cpp
  src/keyframe/keyframe.cpp
  src/utility/CameraPoseVisualization.cpp
  src/ThirdParty/DBoW/BowVector.cpp
  src/ThirdParty/DBoW/FBrief.cpp
  src/ThirdParty/DBoW/FeatureVector.cpp
  src/ThirdParty/DBoW/QueryResults.cpp
  src/ThirdParty/DBoW/ScoringObject.cpp
  src/ThirdParty/DUtils/Random.cpp
  src/ThirdParty/DUtils/Timestamp.cpp
  src/ThirdParty/DVision/BRIEF.cpp
  src/ThirdParty/VocabularyBinary.cpp
)

add_library(pose_graph_lib ${POSE_GRAPH_LIB_SRCS})
ament_target_dependencies(pose_graph_lib
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  cv_bridge camera_model ament_index_cpp
)
target_link_libraries(pose_graph_lib
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Boost::filesystem Boost::program_options Boost::system
)

add_executable(pose_graph_node src/pose_graph_nodelet.cpp)
ament_target_dependencies(pose_graph_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  cv_bridge camera_model ament_index_cpp
)
target_link_libraries(pose_graph_node pose_graph_lib)

install(TARGETS pose_graph_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../support_files/
  DESTINATION share/${PROJECT_NAME}/support_files
)

ament_package()
