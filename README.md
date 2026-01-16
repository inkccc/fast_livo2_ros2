# FAST-LIVO2 ROS2 工作空间

基于 FAST-LIVO2 的 ROS2 Humble 激光-惯性-视觉融合 SLAM 系统，集成相机内参标定、LiDAR-Camera 外参标定及传感器驱动。

> 本项目已移除对 `livox_ros_driver` 的依赖，使用轻量级 `mid360_driver` 替代。

## 功能包结构

```
src/
├── depd/                          # 依赖库
│   └── rpg_vikit/                 # 相机模型与数学工具库
├── driver/                        # 传感器驱动
│   ├── mid360_driver/             # Livox Mid-360 轻量级驱动
│   └── realsense-ros/             # Intel RealSense 相机驱动
├── exts/                          # 扩展工具
│   ├── calib/FAST-Calib-ROS2/     # LiDAR-Camera 外参标定
│   └── fastCalibParams.py         # 点云范围可视化调整工具
└── slam/
    └── FAST-LIVO2/                # FAST-LIVO2 SLAM 算法
```

## 依赖安装

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# 相机标定工具
sudo apt install ros-humble-camera-calibration

# PCL, Eigen, OpenCV
sudo apt install libpcl-dev libeigen3-dev libopencv-dev

# Sophus (非模板版本)
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout a621ff
mkdir build && cd build && cmake .. && make && sudo make install
```

## 编译

```bash
cd ~/fast_livo2_ros2
colcon build --symlink-install --continue-on-error
source install/setup.bash
```

---

## 1. 相机内参标定

使用 ROS2 官方标定工具进行相机内参标定。

### 1.1 启动 RealSense 相机

```bash
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
```

### 1.2 运行标定程序

```bash
# 安装标定工具
sudo apt install ros-humble-camera-calibration

# 运行标定 (根据棋盘格规格修改参数)
ros2 run camera_calibration cameracalibrator \
    --size 7x5 \
    --square 0.036 \
    --ros-args --remap image:=/camera/color/image_raw
```

参数说明：
- `--size`: 棋盘格内角点数 (列x行)
- `--square`: 单个格子边长 (米)

### 1.3 标定结果格式 (FAST-LIVO2)

```yaml
cam_model: Pinhole
cam_width: 640
cam_height: 480
scale: 1.0
cam_fx: 598.993
cam_fy: 599.107
cam_cx: 333.441
cam_cy: 244.164
cam_d0: 0.0486055
cam_d1: 0.412061
cam_d2: -0.00173143
cam_d3: 0.0004922
```

---

## 2. LiDAR-Camera 外参标定

使用 FAST-Calib-ROS2 进行激光雷达与相机的外参标定。

### 2.1 配置标定参数

编辑 `src/exts/calib/FAST-Calib-ROS2/config/qr_params.yaml`:

```yaml
fast_calib:
  ros__parameters:
    # 相机内参 (从内参标定获取)
    fx: 608.7890
    fy: 608.8247
    cx: 330.0415
    cy: 243.2684
    k1: 0.114226
    k2: -0.221661
    p1: -0.000682
    p2: 0.001757

    # 标定板参数
    marker_size: 0.20              # ArUco 标记尺寸 (米)
    delta_width_qr_center: 0.55    # 水平方向标记中心距离的一半
    delta_height_qr_center: 0.35   # 垂直方向标记中心距离的一半

    # 点云过滤范围
    x_min: 1.16
    x_max: 2.15
    y_min: -0.90
    y_max: 1.49
    z_min: 0.39
    z_max: 4.83

    # 输入
    lidar_topic: "/livox/lidar"
    bag_path: "/path/to/your/bag"
    image_path: "/path/to/your/image.png"

    # 输出
    output_path: "/path/to/output"
```

### 2.2 点云范围调整工具

使用可视化工具调整点云过滤范围：

```bash
python3 src/exts/fastCalibParams.py [config_path]
```

### 2.3 运行外参标定

```bash
ros2 launch fast_calib calib.launch.py
```

### 2.4 外参结果格式

```yaml
# 旋转矩阵 (Camera <- LiDAR)
Rcl: [  0.037680,  -0.999262,  -0.007445,
        0.076252,   0.010304,  -0.997035,
        0.996376,   0.037001,   0.076584]

# 平移向量 (Camera <- LiDAR)
Pcl: [ -0.004102,  -0.138772,  -0.030806]
```

---

## 3. 启动 FAST-LIVO2

### 3.1 启动传感器驱动

```bash
# 终端1: 启动 Mid-360 激光雷达
ros2 launch mid360_driver mid360_driver.launch.py

# 终端2: 启动 RealSense 相机
ros2 launch realsense2_camera rs_launch.py enable_color:=true rgb_camera.color_profile:=640,480,30
```

### 3.2 启动 SLAM

```bash
ros2 launch fast_livo mapping_avia.launch.py use_rviz:=True
```

### 3.3 播放数据集

```bash
# 转换 ROS1 bag 到 ROS2
pip install rosbags
rosbags-convert --src your_bag.bag --dst your_bag

# 播放
ros2 bag play your_bag
```

---

## 参考

- [FAST-LIVO2 Paper](https://arxiv.org/pdf/2408.14035)
- [FAST-LIVO2 Dataset](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA)
- [HKU MARS Lab](https://mars.hku.hk/)

## License

GPLv2
