# ==============================================================================
# FAST-LIVO2 CMakeLists.txt
# 功能: 构建 FAST-LIVO2 激光-惯性-视觉融合 SLAM 系统
# ==============================================================================

# ==================== CMake 基础配置 ====================
# 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

# 定义项目名称
project(fast_livo)

# ==================== 编译类型配置 ====================
# 设置编译类型为 Release (优化性能)
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# ==================== C++ 标准配置 ====================
# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
# 强制要求使用指定的 C++ 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 禁用编译器特定的 C++ 扩展
set(CMAKE_CXX_EXTENSIONS OFF)

# ==================== ROS 版本兼容性检查 ====================
# 检查 ROS 版本是否低于 Iron, 如果是则添加预处理宏
IF("$ENV{ROS_DISTRO}" STRLESS "iron")
  add_definitions(-DPRE_ROS_IRON)
ENDIF()

# ==================== 通用编译选项 ====================
# 启用多线程支持和异常处理
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Debug 模式编译选项: 禁用优化, 启用调试信息
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# ==================== CPU 架构检测与优化 ====================
# 输出当前 CPU 架构信息
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# 根据 CPU 架构设置 Release 模式的优化选项
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  # ARM 架构优化
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64 位 ARM 优化 (适用于 RK3588, Jetson Orin NX 等)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32 位 ARM 优化 (启用 NEON 指令集)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  # 定义 ARM 架构宏
  add_definitions(-DARM_ARCH)
else()
  # x86-64 架构优化 (Intel/AMD)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops")
  # 定义 x86 架构宏
  add_definitions(-DX86_ARCH)
endif()

# ==================== 项目路径定义 ====================
# 定义项目根目录宏, 用于代码中访问资源文件
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# ==================== 多线程配置 ====================
# 检测 CPU 核心数
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# 根据核心数配置多线程参数
if(N GREATER 4)
  # 核心数大于 4 时, 使用 4 个线程
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  # 核心数在 2-4 之间, 使用全部核心
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  # 单核心, 禁用多线程
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# ==================== 可选依赖检测 ====================
# 检测 OpenMP 并行计算库
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# 检测 mimalloc 高性能内存分配器
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# ==================== ROS2 依赖查找 ====================
# ament 构建系统
find_package(ament_cmake REQUIRED)

# ROS2 核心库
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# ROS2 消息类型
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ROS2 功能包
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# vikit 视觉工具库
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)

# ==================== 第三方库依赖查找 ====================
# Eigen3 线性代数库
find_package(Eigen3 REQUIRED)

# PCL 点云处理库
find_package(PCL REQUIRED)

# OpenCV 计算机视觉库
find_package(OpenCV REQUIRED)

# Sophus 李群/李代数库
find_package(Sophus REQUIRED)

# Boost 多线程库
find_package(Boost REQUIRED COMPONENTS thread)

# ==================== 头文件目录配置 ====================
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${vikit_common_INCLUDE_DIRS}
  ${vikit_ros_INCLUDE_DIRS}
  include  # 项目头文件目录
)

# ==================== ROS2 依赖列表 ====================
# 定义 ament 依赖列表, 用于 ament_target_dependencies
set(dependencies
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  vikit_common
  vikit_ros
  image_transport
  pcl_ros
  pcl_conversions
  tf2_ros
)

# 通用依赖 (OpenMP)
set(COMMON_DEPENDENCIES OpenMP::OpenMP_CXX)

# ==================== 库文件构建 ====================
# VIO 视觉惯性里程计库
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)

# LIO 激光惯性里程计库 (体素地图)
add_library(lio src/voxel_map.cpp)

# 预处理库 (点云预处理)
add_library(pre src/preprocess.cpp)

# IMU 处理库
add_library(imu_proc src/IMU_Processing.cpp)

# 激光建图核心库
add_library(laser_mapping src/LIVMapper.cpp)

# 工具函数库
add_library(utils src/utils.cpp)

# ==================== 库文件 ROS2 依赖配置 ====================
ament_target_dependencies(vio ${dependencies})
ament_target_dependencies(lio ${dependencies})
ament_target_dependencies(pre ${dependencies})
ament_target_dependencies(imu_proc ${dependencies})
ament_target_dependencies(laser_mapping ${dependencies})

# ==================== 库文件链接配置 ====================
# laser_mapping 链接 vikit 库和 OpenMP
target_link_libraries(laser_mapping
  vikit_common::vikit_common
  vikit_ros::vikit_ros
  ${COMMON_DEPENDENCIES}
)

# 其他库链接 OpenMP
target_link_libraries(vio ${COMMON_DEPENDENCIES})
target_link_libraries(lio utils ${COMMON_DEPENDENCIES})
target_link_libraries(pre ${COMMON_DEPENDENCIES})
target_link_libraries(imu_proc ${COMMON_DEPENDENCIES})

# ==================== 可执行文件构建 ====================
# 主程序: FAST-LIVO2 建图节点
add_executable(fastlivo_mapping src/main.cpp)

# 配置 ROS2 依赖
ament_target_dependencies(fastlivo_mapping ${dependencies})

# 链接所有库文件
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 如果找到 mimalloc, 链接以提升内存分配性能
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# ==================== 安装配置 ====================
# 安装可执行文件到 lib 目录
install(TARGETS
  fastlivo_mapping
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件、启动文件和 RViz 配置到 share 目录
install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# ==================== 导出依赖 ====================
# 导出项目依赖, 供其他包使用
ament_export_dependencies(
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  Eigen3
  PCL
  OpenCV
  Sophus
)

# ==================== ament 包配置 ====================
ament_package()
