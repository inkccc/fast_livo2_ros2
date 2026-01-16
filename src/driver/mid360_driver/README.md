# Mid-360 驱动

这是一个 Mid-360 驱动的实现，旨在作为 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) 的替代方案。

它具有以下特性：

- 不依赖 Livox-SDK2，而是直接实现 UDP 通信，因此非常轻量。
- 支持自动获取雷达 IP，无需手动配置雷达 IP。
- 支持多雷达。
- 在未启用时间同步时，只会在收到第一帧数据时计算一次时间差并在后续使用该时间差，以保证基本的时间同步。

特别说明：

- 发布的点云格式为 PointCloud2，与 livox_ros_driver2 的点云格式不同，用户可能需要修改其他包中的代码。

<img src="./img/ACE.jpg" width="200px">

## 安装依赖

1. 请确保已经安装 ROS2。
2. 安装 Asio。如果你使用的是 Ubuntu，可以通过以下命令安装：  
   `sudo apt install libasio-dev`

## 参数说明

以下是在配置文件中可设置的参数：

```yaml
mid360_driver:
    ros__parameters:
        lidar_topic: /livox/lidar
        imu_topic: /livox/imu
        lidar_frame: livox_frame
        imu_frame: imu_frame
        lidar_publish_time_interval: 0.1 # 10hz
        host_ip: 192.168.32.81
        is_topic_name_with_lidar_ip: false # 是否在话题名后面加雷达ip，可以用于区分多个雷达
        
        publish_tf: true        # 是否发布TF变换
        transform_enable: true  # 是否启用坐标变换
        # 点云坐标变换参数
        rotation_roll: 0.0      # 绕X轴旋转角度（度）
        rotation_pitch: 0.0     # 绕Y轴旋转角度（度）  
        rotation_yaw: 0.0       # 绕Z轴旋转角度（度）
        translation_x: 0.0      # X轴平移（米）
        translation_y: 0.0      # Y轴平移（米）
        translation_z: 0.0      # Z轴平移（米）
        
        # 新增点云距离筛选参数
        min_point_distance: -1.0  # 保留>=该距离的点云（米），-1表示不启用
        max_point_distance: -1.0  # 保留<=该距离的点云（米），-1表示不启用
```

## Contact

QQ group: 1070252119

Email: 1709185482@qq.com

## License

Copyright (C) 2025 Yingjie Huang

Licensed under the MIT License. See License.txt in the project root for license information.
