# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch realsense2_camera node."""
import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
"""
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    rgb_camera.color_profile:=640,480,30 \
    rgb_camera.color_format:=RGB8 \
    enable_depth:=true \
    depth_module.depth_profile:=640,480,30 \
    depth_module.depth_format:=Z16 \
    enable_infra1:=true \
    depth_module.infra1_profile:=640,480,30 \
    depth_module.infra1_format:=Y8 \
    enable_infra2:=true \
    depth_module.infra2_profile:=640,480,30 \
    depth_module.infra2_format:=Y8 \
    enable_sync:=true \
    align_depth.enable:=true \
    pointcloud.enable:=true
"""
configurable_parameters = [
    # ==================== 设备识别与基础配置 ====================
    {'name': 'camera_name',                  'default': 'camera', 'description': '相机节点名称，用于ROS话题和服务的唯一标识'},
    {'name': 'camera_namespace',             'default': '', 'description': '相机命名空间，用于组织多个相机的ROS话题'},
    {'name': 'serial_no',                    'default': "''", 'description': '通过设备序列号选择特定相机，空值表示使用第一个可用设备'},
    {'name': 'usb_port_id',                  'default': "''", 'description': '通过USB端口号选择设备，如"2-1.4"'},
    {'name': 'device_type',                  'default': "D435i", 'description': '通过设备型号选择，如"D435", "D415", "L515"等'},
    
    # ==================== 配置文件与高级设置 ====================
    {'name': 'config_file',                  'default': "''", 'description': 'YAML配置文件路径，可从文件加载参数配置'},
    {'name': 'json_file_path',               'default': "''", 'description': 'RealSense高级配置JSON文件路径，用于设备高级设置'},
    {'name': 'initial_reset',                'default': 'false', 'description': '启动时是否执行硬件复位，解决设备连接问题'},
    {'name': 'accelerate_gpu_with_glsl',     'default': "false", 'description': '启用GLSL GPU加速，提升图像处理性能'},
    
    # ==================== 回放与仿真模式 ====================
    {'name': 'rosbag_filename',              'default': "''", 'description': 'rosbag文件路径，用于从录制的数据运行虚拟设备'},
    {'name': 'rosbag_loop',                  'default': 'false', 'description': '播放rosbag时是否循环播放，用于连续测试'},
    
    # ==================== 日志与输出配置 ====================
    {'name': 'log_level',                    'default': 'info', 'description': '日志输出级别：DEBUG|INFO|WARN|ERROR|FATAL'},
    {'name': 'output',                       'default': 'screen', 'description': '节点输出目标：screen(终端)或log(日志文件)'},
    
    # ==================== 彩色图像流配置 ====================
    {'name': 'enable_color',                 'default': 'true', 'description': '启用彩色图像流，获取RGB彩色图像'},
    {'name': 'rgb_camera.color_profile',     'default': '640,480,30', 'description': '彩色流配置格式：宽度,高度,帧率 (0,0,0为自动选择)'},
    {'name': 'rgb_camera.color_format',      'default': 'RGB8', 'description': '彩色图像格式：RGB8或BGR8等'},
    {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': '启用彩色图像自动曝光，适应不同光照条件'},
    
    # ==================== 深度与红外流配置 ====================
    {'name': 'enable_depth',                 'default': 'false', 'description': '启用深度图像流，获取距离信息（立方体检测必须启用）'},
    {'name': 'enable_infra',                 'default': 'false', 'description': '启用红外0图像流（部分设备使用）'},
    {'name': 'enable_infra1',                'default': 'false', 'description': '启用左红外图像流，用于深度计算（推荐启用）'},
    {'name': 'enable_infra2',                'default': 'false', 'description': '启用右红外图像流，用于深度计算（推荐启用）'},
    
    # ==================== 深度模块配置 ====================
    {'name': 'depth_module.depth_profile',   'default': '640,480,30', 'description': '深度流配置：宽度,高度,帧率'},
    {'name': 'depth_module.depth_format',    'default': 'Z16', 'description': '深度图像格式：Z16(16位深度)'},
    {'name': 'depth_module.infra_profile',   'default': '640,480,30', 'description': '红外流配置：宽度,高度,帧率'},
    {'name': 'depth_module.infra_format',    'default': 'RGB8', 'description': '红外0图像格式'},
    {'name': 'depth_module.infra1_format',   'default': 'Y8', 'description': '左红外图像格式：Y8(8位灰度)'},
    {'name': 'depth_module.infra2_format',   'default': 'Y8', 'description': '右红外图像格式：Y8(8位灰度)'},
    
    # ==================== D405专用彩色配置 ====================
    {'name': 'depth_module.color_profile',   'default': '640,480,15', 'description': 'D405深度模块的彩色流配置'},
    {'name': 'depth_module.color_format',    'default': 'RGB8', 'description': 'D405彩色图像格式'},
    
    # ==================== 深度传感器参数 ====================
    {'name': 'depth_module.exposure',        'default': '8500', 'description': '深度模块手动曝光值（微秒），影响图像亮度'},
    {'name': 'depth_module.gain',            'default': '16', 'description': '深度模块手动增益值，影响图像灵敏度'},
    {'name': 'depth_module.hdr_enabled',     'default': 'false', 'description': '启用HDR高动态范围模式，用于高对比度场景'},
    {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': '启用深度图像自动曝光'},
    
    # ==================== HDR模式参数 ====================
    {'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'HDR模式第一曝光值'},
    {'name': 'depth_module.gain.1',          'default': '16', 'description': 'HDR模式第一增益值'},
    {'name': 'depth_module.exposure.2',      'default': '1', 'description': 'HDR模式第二曝光值'},
    {'name': 'depth_module.gain.2',          'default': '16', 'description': 'HDR模式第二增益值'},
    
    # ==================== 同步与多相机配置 ====================
    {'name': 'enable_sync',                  'default': 'false', 'description': '启用多流同步模式，确保各数据流时间对齐（推荐启用）'},
    {'name': 'depth_module.inter_cam_sync_mode', 'default': "1", 'description': '多相机同步模式：0-默认 1-主机 2-从机'},
    {'name': 'enable_rgbd',                  'default': 'false', 'description': '启用RGBD话题，同时发布彩色和深度注册图像'},
    
    # ==================== IMU惯性测量单元配置 ====================
    {'name': 'enable_gyro',                  'default': 'false', 'description': '启用陀螺仪数据流，获取角速度信息'},
    {'name': 'enable_accel',                 'default': 'false', 'description': '启用加速度计数据流，获取线性加速度信息'},
    {'name': 'gyro_fps',                     'default': '0', 'description': '陀螺仪帧率，0为设备默认值'},
    {'name': 'enable_motion',                'default': 'false', 'description': '启用运动传感器数据流（DDS设备专用）'},
    {'name': 'accel_fps',                    'default': '0', 'description': '加速度计帧率，0为设备默认值'},
    {'name': 'unite_imu_method',             'default': "0", 'description': 'IMU数据整合方式：0-不整合 1-拷贝 2-线性插值'},
    
    # ==================== 深度处理与滤波参数 ====================
    {'name': 'clip_distance',                'default': '-2.', 'description': '深度裁剪距离（米），负值表示禁用，正值裁剪远处点'},
    {'name': 'angular_velocity_cov',         'default': '0.01', 'description': '角速度测量协方差，影响IMU数据精度'},
    {'name': 'linear_accel_cov',             'default': '0.01', 'description': '线性加速度测量协方差，影响IMU数据精度'},
    
    # ==================== 系统监控与诊断 ====================
    {'name': 'diagnostics_period',           'default': '0.0', 'description': '诊断信息发布周期（秒），0表示禁用诊断'},
    
    # ==================== 坐标变换(TF)配置 ====================
    {'name': 'publish_tf',                   'default': 'true', 'description': '发布坐标变换(TF)，用于相机坐标系间的转换'},
    {'name': 'tf_publish_rate',              'default': '0.0', 'description': '动态TF发布频率（Hz），0表示仅发布静态TF'},
    
    # ==================== 点云生成配置 ====================
    {'name': 'pointcloud.enable',            'default': 'false', 'description': '启用点云生成，将深度图转换为3D点云（立方体检测必须启用）'},
    {'name': 'pointcloud.stream_filter',     'default': '2', 'description': '点云纹理来源：0-彩色 1-红外0 2-红外1 3-红外2'},
    {'name': 'pointcloud.stream_index_filter','default': '0', 'description': '点云纹理流索引，用于选择特定纹理源'},
    {'name': 'pointcloud.ordered_pc',        'default': 'true', 'description': '生成有序点云，保持图像像素顺序（推荐启用）'},
    {'name': 'pointcloud.allow_no_texture_points', 'default': 'false', 'description': '允许无纹理的点云点，提高点云完整性'},
    
    # ==================== 图像处理滤波器配置 ====================
    {'name': 'align_depth.enable',           'default': 'false', 'description': '启用深度对齐滤波器，将深度图对齐到彩色图（推荐启用）'},
    {'name': 'colorizer.enable',             'default': 'false', 'description': '启用深度图上色滤波器，为深度图添加伪彩色'},
    
    # ==================== 深度图滤波算法 ====================
    {'name': 'decimation_filter.enable',     'default': 'false', 'description': '启用降采样滤波器，降低深度图分辨率'},
    {'name': 'rotation_filter.enable',       'default': 'false', 'description': '启用旋转滤波器，旋转深度图像'},
    {'name': 'rotation_filter.rotation',     'default': '0.0',   'description': '旋转角度：0.0, 90.0, -90.0, 180.0'},
    {'name': 'spatial_filter.enable',        'default': 'false', 'description': '启用空间滤波器，平滑深度图减少噪声（推荐启用）'},
    {'name': 'temporal_filter.enable',       'default': 'false', 'description': '启用时间滤波器，时域平滑减少抖动（推荐启用）'},
    {'name': 'disparity_filter.enable',      'default': 'false', 'description': '启用视差滤波器，转换深度到视差空间'},
    {'name': 'hole_filling_filter.enable',   'default': 'false', 'description': '启用空洞填充滤波器，填补深度图中的空洞（推荐启用）'},
    {'name': 'hdr_merge.enable',             'default': 'false', 'description': '启用HDR合并滤波器，合并多曝光深度图'},
    
    # ==================== 设备连接与重连设置 ====================
    {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': '等待设备连接超时时间（秒），-1表示无限等待'},
    {'name': 'reconnect_timeout',            'default': '6.', 'description': '设备断开后重连尝试间隔时间（秒）'},
    
    # ==================== 坐标系设置 ====================
    {'name': 'base_frame_id',                'default': 'link', 'description': '传感器TF树的根坐标系名称'},
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    # Load lifecycle nodes setting from YAML dynamically generated by CMAKE instead of environment variable
    lifecycle_param_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'global_settings.yaml'
    )
    lifecycle_params = yaml_to_dict(lifecycle_param_file)
    use_lifecycle_node = lifecycle_params.get("use_lifecycle_node", False)

    _output = LaunchConfiguration('output' + param_name_suffix)
    
    # Dynamically choose Node or LifecycleNode
    node_action = launch_ros.actions.LifecycleNode if use_lifecycle_node else launch_ros.actions.Node
    log_message = "Launching as LifecycleNode" if use_lifecycle_node else "Launching as Normal ROS Node"

    if(os.getenv('ROS_DISTRO') == 'foxy'):
        # Foxy doesn't support output as substitution object (LaunchConfiguration object)
        # but supports it as string, so we fetch the string from this substitution object
        # see related PR that was merged for humble, iron, rolling: https://github.com/ros2/launch/pull/577
        _output = context.perform_substitution(_output)

    return [
        LogInfo(msg=f"🚀 {log_message}"),
        node_action(
            package='realsense2_camera',
            namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
            name=LaunchConfiguration('camera_name' + param_name_suffix),
            executable='realsense2_camera_node',
            parameters=[params, params_from_file],
            output=_output,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
            )
    ]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup, kwargs = {'params' : set_configurable_parameters(configurable_parameters)})
    ])
