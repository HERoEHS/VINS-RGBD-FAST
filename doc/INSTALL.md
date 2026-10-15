# Dynamic-VINS

## 1. Prerequisites

**ROS**
```
sudo apt-get install ros-melodic-cv-bridge ros-melodic-tf ros-melodic-message-filters ros-melodic-image-transport ros-melodic-nav-msgs ros-melodic-visualization-msgs
```

**Ceres-Solver**
```
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
sudo apt-get install libsuitesparse-dev
```
```
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout 2.0.0
mkdir ceres-bin
cd ceres-bin
cmake ..
make -j3
sudo make install
```

**Sophus** — 판 고정 1.24.6 (옛 a621ff 판·so2.cpp 손질은 더 이상 쓰지 않는다, [SW1-1959])
```
git clone -b 1.24.6 https://github.com/strasdat/Sophus.git ~/Sophus
```
헤더 전용이라 빌드가 필요 없다. 판 검사는 `vins_estimator/cmake/SophusPin.cmake` 가 한다(README 1.4 참고).
cmake ≥ 3.24 면 설치본(`cmake --install`)으로 두어도 되고, 그렇지 않으면(Ubuntu 22.04 로봇) 소스 트리 그대로 쓴다
(또는 `cmake_minimum_required` 를 3.22 로 한 줄 낮춰 설치본을 만든다). Eigen 3.4 외 다른 라이브러리는 필요 없다.



## 2. Prerequisites for object detection 

We offer two kinds of device for tests, please follow the instruction for your match device.

### 2.1. NVIDIA devices

Clone the repository and catkin_make:

```
cd {YOUR_WORKSPACE}/src
git clone https://github.com/jianhengLiu/Dynamic-VINS.git

# build
cd ..
catkin_make
```



### 2.2. HUAWEI Atlas200


**!!!Note:**
 It is recommended to use high write speeds's microSD card(TF card), and a low write speeds' microSD card may result in Dyanmic-VINS not perferming in real time.(at least 40Mbs)

0. prequisities

```
  sudo apt install ros-melodic-image-transport-plugins
```

1. Clone the repository:

```
  cd {YOUR_WORKSPACE}/src
  git clone https://github.com/jianhengLiu/Dynamic-VINS.git

  git clone https://github.com/jianhengLiu/compressedimg2img.git
```

1. allocate core 
```
  sudo npu-smi set -t aicpu-config -i 0 -c 0 -d 2
```

4.  build
```   
  cd ../..
  catkin_make
```