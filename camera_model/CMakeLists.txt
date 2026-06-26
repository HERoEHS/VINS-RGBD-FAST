cmake_minimum_required(VERSION 3.8)
project(camera_model)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

# === OpenCV ABI 고정 (SW1-1837) — ★반드시 모든 find_package 보다 먼저 ===
#   다른 find_package가 transitive 하게 OpenCV_DIR을 /usr/local 커스텀(4.14)으로 먼저 박으면
#   가드가 무력화 → cv_bridge(시스템 4.5d)와 2버전 로드 → ABI 충돌·크래시. 시스템 OpenCV 선점.
#   외부 -DOpenCV_DIR 지정 시 존중(아키텍처 무관 glob=이식성).
if(NOT DEFINED OpenCV_DIR)
  file(GLOB _sys_opencv_dirs "/usr/lib/*/cmake/opencv4")
  if(_sys_opencv_dirs)
    list(GET _sys_opencv_dirs 0 OpenCV_DIR)
    message(STATUS "OpenCV_DIR 힌트(시스템 apt): ${OpenCV_DIR}")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

# OpenCV_DIR은 위(모든 find_package 이전)에서 시스템 경로로 선점됨(SW1-1837). 여기선 그대로 사용.
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(CAMERA_MODEL_SRCS
  src/chessboard/Chessboard.cc
  src/calib/CameraCalibration.cc
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc
)

add_library(camera_model ${CAMERA_MODEL_SRCS})
target_link_libraries(camera_model
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
target_include_directories(camera_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(Calibration
  src/intrinsic_calib.cc
  ${CAMERA_MODEL_SRCS}
)
target_link_libraries(Calibration
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

install(TARGETS camera_model
  EXPORT camera_modelTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(EXPORT camera_modelTargets
  FILE camera_modelTargets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(camera_modelTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Boost OpenCV Ceres Eigen3)

ament_package()
