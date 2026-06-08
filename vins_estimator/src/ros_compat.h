#pragma once
// ROS1→ROS2 porting shim — 순수 알고리즘 파일용 로깅/어설션 호환 매크로

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>

// 로깅 매크로 (printf 기반)
#define ROS_INFO(fmt, ...)      printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define ROS_WARN(fmt, ...)      fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__)
#define ROS_ERROR(fmt, ...)     fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#define ROS_DEBUG(fmt, ...)     do {} while (0)
#define ROS_FATAL(fmt, ...)     fprintf(stderr, "[FATAL] " fmt "\n", ##__VA_ARGS__)

// once 변형
#define ROS_INFO_ONCE(fmt, ...)  do { static bool _once=false; if(!_once){ ROS_INFO(fmt,##__VA_ARGS__); _once=true; } } while(0)
#define ROS_WARN_ONCE(fmt, ...)  do { static bool _once=false; if(!_once){ ROS_WARN(fmt,##__VA_ARGS__); _once=true; } } while(0)
#define ROS_ERROR_ONCE(fmt, ...) do { static bool _once=false; if(!_once){ ROS_ERROR(fmt,##__VA_ARGS__); _once=true; } } while(0)

// 스트림 변형
#define ROS_INFO_STREAM(x)       (std::cout << "[INFO] " << x << std::endl)
#define ROS_WARN_STREAM(x)       (std::cerr << "[WARN] " << x << std::endl)
#define ROS_ERROR_STREAM(x)      (std::cerr << "[ERROR] " << x << std::endl)
#define ROS_DEBUG_STREAM(x)      do {} while (0)
#define ROS_FATAL_STREAM(x)      (std::cerr << "[FATAL] " << x << std::endl)

#define ROS_INFO_STREAM_ONCE(x)  do { static bool _once=false; if(!_once){ ROS_INFO_STREAM(x); _once=true; } } while(0)
#define ROS_WARN_STREAM_ONCE(x)  do { static bool _once=false; if(!_once){ ROS_WARN_STREAM(x); _once=true; } } while(0)

// 어설션 매크로
#define ROS_ASSERT(cond)             assert(cond)
#define ROS_ASSERT_MSG(cond, fmt, ...) \
    do { if (!(cond)) { fprintf(stderr, "[ASSERT] " fmt "\n", ##__VA_ARGS__); assert(cond); } } while(0)
#define ROS_BREAK() \
    do { fprintf(stderr, "[BREAK] %s:%d\n", __FILE__, __LINE__); std::abort(); } while(0)
