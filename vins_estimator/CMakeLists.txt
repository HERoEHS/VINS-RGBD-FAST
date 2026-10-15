cmake_minimum_required(VERSION 3.8)
project(vins_estimator)

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
find_package(yaml-cpp REQUIRED)   # [SW1-1962] edie_vision 캘리브(ROS camera_info 형식) 읽기
# yaml-cpp 0.8.0(Yocto Scarthgap)은 정식 타깃 yaml-cpp::yaml-cpp 만 export 한다 — 옛 이름 yaml-cpp 타깃이 없어서
# 그대로 링크하면 맨 -lyaml-cpp 로 넘어가 include 경로·정의가 빠진다(옛 이름 호환 타깃+경고는 0.8.0 뒤 master 에서 추가).
# Ubuntu 22.04 의 0.7·.219 의 0.6(정적)은 옛 이름뿐이다 → 있는 쪽을 쓴다 [SW1-1959].
if(TARGET yaml-cpp::yaml-cpp)
  set(VINS_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(VINS_YAML_CPP_TARGET yaml-cpp)
endif()
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Sophus(헤더 전용) — 판을 1.24.6 으로 못 박는다 [SW1-1959]. 찾는 순서·판 검사·안내는 cmake/SophusPin.cmake.
#   ① 설치본 SophusConfig.cmake(sysroot·/usr/local, Yocto 정본) ② 교차 빌드가 아닐 때만 소스 트리
#   (-DSOPHUS_INCLUDE_DIR, 기본 ~/Sophus, SOPHUS_VERSION 파일로 판 확인) ③ 없거나 판이 다르면 FATAL.
#   예전엔 ~/Sophus 를 판 확인 없이 그대로 include 해 2020 판/1.22(meta-ros)가 조용히 섞일 수 있었다.
include(SophusPin)

include_directories(
  src
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${VINS_SOPHUS_INCLUDE_DIRS}
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
  src/utility/pinhole_calib_override.cpp
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
  ${VINS_YAML_CPP_TARGET}
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

# [SW1-1948] 설정(vio_edie.yaml 등)은 저장소 루트 config/ 에 있다 — vins_estimator/config/ 는 없다.
# 예전엔 없는 폴더를 OPTIONAL 로 설치해 share/config 가 빈 채로 배포됐고, 소스 체크아웃 없이 설치본만 있는
# 로봇(/opt/edie, root 계정)에서 launch 기본 경로가 깨져 기동 즉시 abort 했다. OPTIONAL 을 빼서
# 폴더가 없으면 빌드에서 바로 실패하게 한다(조용히 빈 설치 금지).
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../config/
  DESTINATION share/${PROJECT_NAME}/config
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

  # [SW1-1889] depth 채택 프로브(순수 STL 헤더-only) — 분류·행 형식·중복 억제 단위검증
  ament_add_gtest(test_depth_adopt_probe test/test_depth_adopt_probe.cpp)

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

  # [SW1-1922] STILL-DRIFT 가드 몸체 운동 판정(still_drift_guard.h) + 다리 게이트 구간 연장 — 헤더-only
  ament_add_gtest(test_still_drift_guard test/test_still_drift_guard.cpp)
  target_include_directories(test_still_drift_guard PRIVATE
    ${EIGEN3_INCLUDE_DIR}
  )

  # [SW1-1866 reboot-pose-seed] 재초기화 시드 계승 순수 수학(Eigen 헤더-only) 단독 검증
  ament_add_gtest(test_reboot_seed test/test_reboot_seed.cpp)
  target_include_directories(test_reboot_seed PRIVATE
    ${EIGEN3_INCLUDE_DIR}
  )
  # [SW1-1936] vpr4 실기 IMU 발췌(앵커 pose → 시드 캡처)로 다리 회전 재현
  target_compile_definitions(test_reboot_seed PRIVATE
    VINS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

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

  # [SW1-1883] 마지널라이즈 빈 행렬 가드 + 창 분단 판정 — 회귀 테스트
  ament_add_gtest(test_marginalization_empty test/test_marginalization_empty.cpp)
  target_link_libraries(test_marginalization_empty vins_lib)

  # [SW1-1883 후속] 슬롯 적분 상한 강제 키프레임 — 회귀 테스트
  ament_add_gtest(test_preint_slot_cap test/test_preint_slot_cap.cpp)
  target_link_libraries(test_preint_slot_cap vins_lib)

  # [SW1-1883 후속] IMU/휠 스탬프 간극 판정 — 헤더 단독 순수 함수 테스트
  ament_add_gtest(test_imu_gap test/test_imu_gap.cpp)

  # [SW1-1964] 영상 속도 제한기 — 재시작 때 시간 기준·받은 사진 수 함께 리셋(재시작 뒤 영구 정지 재발 방지)
  ament_add_gtest(test_frontend_rate_limiter test/test_frontend_rate_limiter.cpp)
  target_compile_definitions(test_frontend_rate_limiter PRIVATE
    VINS_SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_include_directories(test_frontend_rate_limiter PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_frontend_rate_limiter ${OpenCV_LIBS})

  # [SW1-1828] vio_edie.yaml 카메라 모델 ↔ 정류 영상(edie_vision P) 일치 — 이중 보정 재발 방지
  ament_add_gtest(test_camera_intrinsics_consistency test/test_camera_intrinsics_consistency.cpp)
  target_compile_definitions(test_camera_intrinsics_consistency PRIVATE
    VINS_SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_include_directories(test_camera_intrinsics_consistency PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_camera_intrinsics_consistency ${OpenCV_LIBS})

  # [SW1-1962] VINS 가 edie_vision 캘리브(정류 P)를 직접 읽어 만든 카메라 = vio_edie.yaml 카메라와 같은 투영
  ament_add_gtest(test_pinhole_calib_override test/test_pinhole_calib_override.cpp
    src/utility/pinhole_calib_override.cpp)
  target_compile_definitions(test_pinhole_calib_override PRIVATE
    VINS_SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_include_directories(test_pinhole_calib_override PRIVATE ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(test_pinhole_calib_override camera_model)
  target_link_libraries(test_pinhole_calib_override ${OpenCV_LIBS} ${VINS_YAML_CPP_TARGET})

  # [SW1-1948] 설정 파일을 못 열면 readParameters 가 즉시 예외 — 값 0 으로 계속 가다 cv2eigen abort 하던 결함 회귀.
  # rclcpp 노드를 만드는 첫 테스트라 격리 도메인(ros_isolated)에서 돌린다 — PC 셸이 로봇과 같은 ROS_DOMAIN_ID 를 쓰기 때문.
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(test_config_fail_fast test/test_config_fail_fast.cpp)
  target_link_libraries(test_config_fail_fast vins_lib)

  # [SW1-1948] 설치본(share)에 config 가 실제로 들어가고 launch 기본값이 그 파일을 가리키는지 — colcon test 는 install 뒤라 share 를 직접 본다
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_installed_config test/test_installed_config.py)
  ament_add_pytest_test(test_vision_calib_launch test/test_vision_calib_launch.py)
  ament_add_pytest_test(test_body_t_wheel_beta test/test_body_t_wheel_beta.py)

  # [SW1-1959] Sophus 판 고정(cmake/SophusPin.cmake) — 가짜 설치본·소스 트리로 configure 만 돌려
  # 판 검사·찾는 순서·교차 빌드 분기·안내 문구를 확인한다(컴파일러 불필요, 수 초).
  add_test(NAME test_sophus_pin
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sophus_pin.sh
            ${CMAKE_COMMAND} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/SophusPin.cmake)

  # [SW1-1959] 시스템 OpenCV 힌트(cmake/OpenCvSystemHint.cmake) — 네이티브/교차 빌드/외부 지정 동작 + 세 패키지 사본 동일성
  add_test(NAME test_opencv_hint
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_opencv_hint.sh
            ${CMAKE_COMMAND} ${CMAKE_CURRENT_SOURCE_DIR}/..)
endif()

ament_package()
