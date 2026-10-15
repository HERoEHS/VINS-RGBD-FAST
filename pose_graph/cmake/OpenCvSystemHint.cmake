# OpenCvSystemHint.cmake — 시스템(apt) OpenCV 를 먼저 고르는 힌트(SW1-1837), 교차 빌드에선 끈다(SW1-1959)
#
# 왜 있나: Ubuntu 개발 PC·로봇엔 /usr/local 에 직접 빌드한 OpenCV(4.14)가 시스템 OpenCV(4.5)와 같이 있다.
#   cv_bridge/camera_model 의 config 가 먼저 find_package(OpenCV) 를 부르면 OpenCV_DIR 이 /usr/local 로 박혀
#   cv_bridge(시스템 판)와 두 판이 한 프로세스에 올라가 cv::Mat ABI 충돌로 죽었다(SW1-1837). 그래서 어떤
#   find_package 보다 먼저 시스템 OpenCV 의 cmake 폴더를 OpenCV_DIR 로 선점한다.
#
# 교차 빌드(Yocto SDK colcon·bitbake)에선 이 힌트가 호스트 PC 의 /usr/lib(x86 OpenCV)를 집어 가 sysroot 의
#   OpenCV 를 가린다. CMAKE_CROSSCOMPILING 이면 아무것도 하지 않고 toolchain 의 sysroot 검색에 맡긴다.
#   Yocto 는 cv_bridge 와 같은 OpenCV 레시피 하나뿐이라 두 판 충돌 문제 자체가 없다.
#
# 규칙: 외부 -DOpenCV_DIR 지정은 항상 존중. 이 파일은 세 패키지(vins_estimator·camera_model·pose_graph)에
#   같은 내용으로 복사돼 있다 — vins_estimator/test/test_opencv_hint.sh 가 세 사본의 동일성과 동작을 검사한다.
# 시험용: VINS_OPENCV_SYS_LIB_ROOT(기본 /usr/lib)로 검색 뿌리를 바꿀 수 있다(<뿌리>/*/cmake/opencv4).

set(VINS_OPENCV_SYS_LIB_ROOT "/usr/lib" CACHE PATH
  "시스템 OpenCV cmake 폴더를 찾을 뿌리(<뿌리>/*/cmake/opencv4) — 교차 빌드가 아닐 때만 쓴다")

if(DEFINED OpenCV_DIR)
  # -DOpenCV_DIR 외부 지정이거나, 이전 configure 가 캐시에 남긴 값(재configure 땐 이 분기로 온다)
  message(STATUS "OpenCV_DIR 이미 정해짐(외부 지정 또는 이전 configure): ${OpenCV_DIR}")
elseif(CMAKE_CROSSCOMPILING)
  message(STATUS "교차 빌드 — 호스트 OpenCV 힌트 건너뜀, sysroot 의 OpenCV 를 쓴다")
else()
  file(GLOB _sys_opencv_dirs "${VINS_OPENCV_SYS_LIB_ROOT}/*/cmake/opencv4")
  if(_sys_opencv_dirs)
    list(GET _sys_opencv_dirs 0 OpenCV_DIR)
    message(STATUS "OpenCV_DIR 힌트(시스템 apt): ${OpenCV_DIR}")
  endif()
endif()
