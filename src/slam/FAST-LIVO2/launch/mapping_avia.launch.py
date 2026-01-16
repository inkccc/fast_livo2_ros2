#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
FAST-LIVO2 SLAM 启动文件
功能: 启动 FAST-LIVO2 激光-惯性-视觉融合 SLAM 系统
包含: 图像重发布节点、SLAM 主节点、RViz2 可视化节点
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== 路径配置 ====================
    # 获取 fast_livo 功能包的共享目录路径
    pkg_path_fast_livo = get_package_share_directory("fast_livo")

    # 配置文件目录路径
    config_path_dir = os.path.join(pkg_path_fast_livo, "config")

    # RViz2 配置文件路径 - 用于 3D 可视化显示
    config_path_rviz = os.path.join(pkg_path_fast_livo, "rviz_cfg", "fast_livo2.rviz")

    # ==================== 参数文件配置 ====================
    # Avia 激光雷达配置文件路径 - 包含 LiDAR 和 IMU 参数
    config_path_avia = os.path.join(config_path_dir, "avia.yaml")

    # 相机配置文件路径 - 包含相机内参和畸变参数
    config_path_camera = os.path.join(config_path_dir, "camera_pinhole.yaml")

    # ==================== 启动参数声明 ====================
    # 声明启动参数: 是否启动 RViz2 可视化工具
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",  # 参数名称
        default_value="True",  # 默认值为 True（启动 RViz2）
        description="Whether to launch Rviz2"  # 参数描述
    )

    # 声明启动参数: Avia 激光雷达配置文件路径
    declare_avia_params = DeclareLaunchArgument(
        'avia_params_file',  # 参数名称
        default_value=config_path_avia,  # 默认使用预设的配置文件路径
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes'  # 参数描述
    )

    # 声明启动参数: 相机配置文件路径
    declare_camera_params = DeclareLaunchArgument(
        'camera_params_file',  # 参数名称
        default_value=config_path_camera,  # 默认使用预设的配置文件路径
        description='Full path to the ROS2 parameters file to use for vikit_ros nodes'  # 参数描述
    )

    # 声明启动参数: 节点崩溃后是否自动重启
    # 参考: https://github.com/ros-navigation/navigation2/blob/1c68c212db01f9f75fcb8263a0fbb5dfa711bdea/nav2_bringup/launch/navigation_launch.py#L40
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn',  # 参数名称
        default_value='True',  # 默认值为 True（启用自动重启）
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'  # 参数描述
    )

    # ==================== 获取启动参数配置值 ====================
    # 获取 Avia 配置文件路径参数值
    avia_params_file = LaunchConfiguration('avia_params_file')
    # 获取相机配置文件路径参数值
    camera_params_file = LaunchConfiguration('camera_params_file')
    # 获取是否自动重启参数值
    use_respawn = LaunchConfiguration('use_respawn')
    # 获取是否启动 RViz2 参数值
    use_rviz = LaunchConfiguration("use_rviz")

    # ==================== 节点定义 ====================
    # 图像重发布节点 - 将压缩图像转换为原始图像
    # 用于将 compressed 格式的图像话题转换为 raw 格式
    # 参考: https://robotics.stackexchange.com/questions/110939/how-do-i-remap-compressed-video-to-raw-video-in-ros2
    # 命令行等效: ros2 run image_transport republish compressed raw --ros-args --remap in:=/left_camera/image --remap out:=/left_camera/image
    action_republish = Node(
        package="image_transport",  # 包名
        executable="republish",  # 可执行文件名
        name="republish",  # 节点名称
        arguments=[  # 命令行参数: 输入格式和输出格式
            'compressed',  # 输入: 压缩图像
            'raw',  # 输出: 原始图像
        ],
        remappings=[  # 话题重映射
            ("in", "/left_camera/image"),  # 输入话题
            ("out", "/left_camera/image")  # 输出话题
        ],
        output="screen",  # 输出到屏幕
        respawn=use_respawn,  # 崩溃后是否自动重启
    )

    # FAST-LIVO2 SLAM 主节点 - 执行激光-惯性-视觉融合定位与建图
    action_fastlivo_mapping = Node(
        package="fast_livo",  # 包名
        executable="fastlivo_mapping",  # 可执行文件名
        name="laserMapping",  # 节点名称
        parameters=[  # 参数文件列表
            avia_params_file,  # Avia 激光雷达和 IMU 参数
            camera_params_file,  # 相机内参和畸变参数
        ],
        # 调试前缀配置 (当前已禁用)
        # 参考: https://docs.ros.org/en/humble/How-To-Guides/Getting-Backtraces-in-ROS-2.html
        prefix=[
            # GDB 调试器: ("gdb -ex run --args"),
            # Valgrind 内存检测: ("valgrind --log-file=./valgrind_report.log --tool=memcheck --leak-check=full --show-leak-kinds=all -s --track-origins=yes --show-reachable=yes --undef-value-errors=yes --track-fds=yes")
        ],
        output="screen"  # 输出到屏幕
    )

    # RViz2 可视化节点 - 3D 点云和轨迹可视化
    action_rviz2 = Node(
        condition=IfCondition(use_rviz),  # 条件: 仅在 use_rviz 为 True 时启动
        package="rviz2",  # 包名
        executable="rviz2",  # 可执行文件名
        name="rviz2",  # 节点名称
        arguments=["-d", config_path_rviz],  # 加载指定的 RViz 配置文件
        output="screen"  # 输出到屏幕
    )

    # ==================== ROS2 Bag 播放 (可选, 当前已禁用) ====================
    # 用于播放录制的数据集进行离线测试
    action_rosbag_play = ExecuteProcess(
        cmd=[['ros2 bag play ', '~/datasets/Retail_Street ', '--clock ', "-l"]],
        shell=True
    )

    # ==================== 返回启动描述 ====================
    # 包含所有要启动的参数声明和节点
    return LaunchDescription([
        # 启动参数声明
        declare_use_rviz,  # 声明 use_rviz 参数
        declare_avia_params,  # 声明 avia_params_file 参数
        declare_camera_params,  # 声明 camera_params_file 参数
        declare_use_respawn,  # 声明 use_respawn 参数

        # 启动节点
        # action_republish,  # 启动图像重发布节点
        # action_rosbag_play,  # 启动 ROS2 Bag 播放节点 (可选)
        action_fastlivo_mapping,  # 启动 FAST-LIVO2 SLAM 主节点
        action_rviz2,  # 启动 RViz2 可视化节点 (条件启动)
    ])
