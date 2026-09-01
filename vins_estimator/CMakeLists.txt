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

  # [SW1-1837] LegEventDetector(순수 STL 헤더-only) — 이벤트 게이팅 구간 마킹 단위검증
  ament_add_gtest(test_leg_event_detector test/test_leg_event_detector.cpp)

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

  # BodyNhcFactor(AutoDiff, ceres+Eigen 헤더-only) 단독 검증
  ament_add_gtest(test_body_nhc_factor test/test_body_nhc_factor.cpp)
  target_include_directories(test_body_nhc_factor PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_body_nhc_factor ${CERES_LIBRARIES})

  # GravityAlignFactor(AutoDiff, ceres+Eigen 헤더-only) 단독 검증
  ament_add_gtest(test_gravity_align_factor test/test_gravity_align_factor.cpp)
  target_include_directories(test_gravity_align_factor PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_gravity_align_factor ${CERES_LIBRARIES})

  # [SW1-1866] yaw 게이지 슬라이드 가드 핵심 수학(Eigen 헤더-only) 단독 검증
  ament_add_gtest(test_yaw_slide_guard test/test_yaw_slide_guard.cpp)
  target_include_directories(test_yaw_slide_guard PRIVATE
    ${EIGEN3_INCLUDE_DIR}
  )

  # [SW1-1866 reboot-pose-seed] 재초기화 시드 계승 순수 수학(Eigen 헤더-only) 단독 검증
  ament_add_gtest(test_reboot_seed test/test_reboot_seed.cpp)
  target_include_directories(test_reboot_seed PRIVATE
    ${EIGEN3_INCLUDE_DIR}
  )

  # [SW1-1872] HF 예측 순수 수학부(hf_predict.h, 헤더-only) — 결함①·② 회귀 방지
  ament_add_gtest(test_hf_predict test/test_hf_predict.cpp)
  target_include_directories(test_hf_predict PRIVATE
    ${EIGEN3_INCLUDE_DIR}
  )

  # [SW1-1866] 정지 상대운동 잠금 factor(AutoDiff, ceres+Eigen 헤더-only) 단독 검증
  ament_add_gtest(test_still_motion_factor test/test_still_motion_factor.cpp)
  target_include_directories(test_still_motion_factor PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_still_motion_factor ${CERES_LIBRARIES})

  # 중력 재정렬 v2(창 전체 자세 보정) 핵심 수학 — Eigen 헤더-only 단독 검증
  ament_add_gtest(test_gravity_window_realign test/test_gravity_window_realign.cpp)
  target_include_directories(test_gravity_window_realign PRIVATE
    ${EIGEN3_INCLUDE_DIR}
  )

  # 고속 회전 비전 게이팅 판정 로직 — Eigen 헤더-only 단독 검증
  ament_add_gtest(test_yaw_gating test/test_yaw_gating.cpp)
  target_include_directories(test_yaw_gating PRIVATE
    ${EIGEN3_INCLUDE_DIR}
  )

  # [docking] odom→base_vins 합성 수학(Eigen 헤더-only) 단독 검증
  ament_add_gtest(test_odom_base_vins_tf test/test_odom_base_vins_tf.cpp)
  target_include_directories(test_odom_base_vins_tf PRIVATE
    ${EIGEN3_INCLUDE_DIR}
  )

  # Bg_z 잠금 발동 판정 로직 — 헤더-only 단독 검증
  ament_add_gtest(test_bgz_lock test/test_bgz_lock.cpp)

  # 스핀 중 accel 신뢰 강등 판정 로직 — 헤더-only 단독 검증
  ament_add_gtest(test_spin_acc_deweight test/test_spin_acc_deweight.cpp)

  # [SW1-1866] init/출발 워밍업 게이트 판정 로직 — 헤더-only 단독 검증
  ament_add_gtest(test_warmup_init_gate test/test_warmup_init_gate.cpp)

  # [08-31] all_image_frame 정지 누수 가지치기 — Estimator 실물 링크 회귀 테스트
  ament_add_gtest(test_all_image_frame_prune test/test_all_image_frame_prune.cpp)
  target_link_libraries(test_all_image_frame_prune vins_lib)

  # [SW1-1880] static init 자세 부여(장부>창 결함) — Estimator 실물 링크 회귀 테스트
  ament_add_gtest(test_static_init_pose_assign test/test_static_init_pose_assign.cpp)
  target_link_libraries(test_static_init_pose_assign vins_lib)
endif()

ament_package()
