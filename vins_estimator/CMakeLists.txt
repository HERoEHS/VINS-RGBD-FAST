cmake_minimum_required(VERSION 3.8)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# === OpenCV ABI 고정 (SW1-1837) — ★반드시 모든 find_package 보다 먼저 ===
#   cv_bridge/camera_model config가 transitive 하게 find_package(OpenCV)를 호출해 OpenCV_DIR을
#   /usr/local 커스텀(4.14)으로 먼저 박으면 아래 가드가 무력화 → cv_bridge(시스템 4.5d)와 2버전
#   로드 → cv::Mat ABI 충돌·크래시(277TB 할당). 그래서 어떤 find_package 보다 먼저 시스템 OpenCV 선점.
#   외부 -DOpenCV_DIR 지정 시 존중(아키텍처 무관 glob=이식성).
if(NOT DEFINED OpenCV_DIR)
  file(GLOB _sys_opencv_dirs "/usr/lib/*/cmake/opencv4")
  if(_sys_opencv_dirs)
    list(GET _sys_opencv_dirs 0 OpenCV_DIR)
    message(STATUS "OpenCV_DIR 힌트(시스템 apt): ${OpenCV_DIR}")
  endif()
endif()

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

# OpenCV_DIR은 위(모든 find_package 이전)에서 시스템 경로로 선점됨(SW1-1837). 여기선 그대로 사용.
find_package(OpenCV REQUIRED)
message(STATUS "Using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Sophus (헤더 전용 라이브러리) 경로 — 이식성 확보.
#   기존엔 /home/higony/Sophus 로 하드코딩돼 있어 다른 사용자/로봇(예: /home/edie)에서 빌드가 깨졌다.
#   기본값은 홈 디렉토리(~/Sophus). 다른 위치면 빌드 시 -DSOPHUS_INCLUDE_DIR=<경로> 로 덮어쓴다.
set(SOPHUS_INCLUDE_DIR "$ENV{HOME}/Sophus" CACHE PATH "Path to Sophus header-only library")
if(NOT EXISTS "${SOPHUS_INCLUDE_DIR}/sophus")
  message(WARNING "Sophus 헤더를 '${SOPHUS_INCLUDE_DIR}' 에서 못 찾음. "
                  "git clone https://github.com/strasdat/Sophus 후 "
                  "-DSOPHUS_INCLUDE_DIR=<경로> 로 지정하세요.")
endif()

include_directories(
  src
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${SOPHUS_INCLUDE_DIR}
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

# launch 파일을 share 에 설치해야 `ros2 launch vins_estimator <file>` 로 실행 가능
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# ===== 테스트 (SW1-1837 ZUPT 회귀) =====
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ZeroVelocityFactor는 헤더-only(ceres+Eigen 의존)라 vins_lib 링크 없이 단독 검증 가능
  ament_add_gtest(test_zero_velocity_factor test/test_zero_velocity_factor.cpp)
  target_include_directories(test_zero_velocity_factor PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_zero_velocity_factor ${CERES_LIBRARIES})

  # AccBiasPriorFactor도 헤더-only(ceres+Eigen)라 vins_lib 링크 없이 단독 검증 가능
  ament_add_gtest(test_acc_bias_prior_factor test/test_acc_bias_prior_factor.cpp)
  target_include_directories(test_acc_bias_prior_factor PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_acc_bias_prior_factor ${CERES_LIBRARIES})

  # VerticalVelocityFactor도 헤더-only(ceres+Eigen)라 vins_lib 링크 없이 단독 검증 가능
  ament_add_gtest(test_vertical_velocity_factor test/test_vertical_velocity_factor.cpp)
  target_include_directories(test_vertical_velocity_factor PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_vertical_velocity_factor ${CERES_LIBRARIES})

  # PlaneFactor(AutoDiff, ceres+Eigen 헤더-only) 단독 검증
  ament_add_gtest(test_plane_factor test/test_plane_factor.cpp)
  target_include_directories(test_plane_factor PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_plane_factor ${CERES_LIBRARIES})
endif()

ament_package()
