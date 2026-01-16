#!/usr/bin/env python3
import os
import sys
import yaml
import tkinter as tk
from tkinter import ttk
import numpy as np
import open3d as o3d
import rclpy 
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import time

# ===== 静态配置参数 =====
DEFAULT_CONFIG_PATH = "/home/inkc/inkc/tws/fast_livo2_ros2/src/exts/calib/FAST-Calib-ROS2/config/qr_params.yaml"
MAX_POINTS = 100000              # 最大点云点数
VIS_WIDTH, VIS_HEIGHT = 800, 800  # 可视化窗口大小
VIEW_FRONT = [0, -1, 0.3]        # 默认视角 front
VIEW_UP = [0, 0, 1]              # 默认视角 up
VIEW_ZOOM = 0.1                  # 默认缩放

# 新增：滑动条配置参数
SLIDER_LENGTH = 500              # 滑动条长度（像素）
SLIDER_RESOLUTION = 1000         # 滑动条分辨率（步数）
PRECISION = 6                    # 数值显示精度

class PointCloudRangeAdjuster:
    def __init__(self, config_path=None):
        self.config_path = config_path if config_path else DEFAULT_CONFIG_PATH
        print(f"Using config file: {self.config_path}")
        
        self.load_config()
        self.original_cloud = self.load_pointcloud_from_bag()
        if len(self.original_cloud) == 0:
            print("ERROR: No point cloud data loaded. Check bag file and topic.")
            sys.exit(1)
            
        self.filtered_cloud = o3d.geometry.PointCloud()
        
        # 计算初始范围
        self.original_ranges = {
            'x': [np.min(self.original_cloud[:, 0]), np.max(self.original_cloud[:, 0])],
            'y': [np.min(self.original_cloud[:, 1]), np.max(self.original_cloud[:, 1])],
            'z': [np.min(self.original_cloud[:, 2]), np.max(self.original_cloud[:, 2])]
        }
        
        # 计算扩展范围（用于滑动条）
        self.slider_ranges = {}
        for axis in ['x', 'y', 'z']:
            min_val = self.original_ranges[axis][0]
            max_val = self.original_ranges[axis][1]
            range_size = max_val - min_val
            # 扩大范围以便调节
            expanded_min = min_val - range_size * 0.5
            expanded_max = max_val + range_size * 0.5
            self.slider_ranges[axis] = [expanded_min, expanded_max]
        
        # 初始化当前范围为原始范围
        self.ranges = {k: v.copy() for k, v in self.original_ranges.items()}
        
        print(f"Original point cloud range:")
        for axis in ['x', 'y', 'z']:
            print(f"  {axis.upper()}: {self.original_ranges[axis][0]:.2f} to {self.original_ranges[axis][1]:.2f}")
        print(f"Total points: {len(self.original_cloud)}")
        
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("Point Cloud Range Adjuster", width=VIS_WIDTH, height=VIS_HEIGHT)
        
        # 保存当前视角
        self.current_view = None
        
        self.root = tk.Tk()
        self.root.title("Point Cloud Range Control")
        self.root.geometry("900x600")  # 增加窗口宽度
        self.create_control_panel()
        
        self.update_pointcloud_display(init_view=True)
    
    def load_config(self):
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        if 'fast_calib' in config:
            self.bag_path = config['fast_calib']['ros__parameters']['bag_path']
            self.lidar_topic = config['fast_calib']['ros__parameters']['lidar_topic']
        else:
            self.bag_path = config['bag_path']
            self.lidar_topic = config['lidar_topic']
        
        if self.bag_path.startswith("~"):
            self.bag_path = os.path.expanduser(self.bag_path)
        
        print(f"Loaded config: bag_path={self.bag_path}, lidar_topic={self.lidar_topic}")
    
    def load_pointcloud_from_bag(self):
        print(f"Loading point cloud from: {self.bag_path}")
        start_time = time.time()
        
        storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr',
                                             output_serialization_format='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        cloud_points = []
        total_points = 0
        topics = set()
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            topics.add(topic)
            
            if topic == self.lidar_topic:
                try:
                    msg = deserialize_message(data, PointCloud2)
                    points = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
                    for p in points:
                        cloud_points.append([p[0], p[1], p[2]])
                        total_points += 1
                        if total_points >= MAX_POINTS:
                            print(f"Reached maximum points limit ({MAX_POINTS}). Stopping early.")
                            break
                    if total_points >= MAX_POINTS:
                        break
                except Exception as e:
                    print(f"Error parsing PointCloud2: {e}")
        
        print(f"Available topics in bag: {topics}")
        print(f"Loaded {len(cloud_points)} points in {time.time() - start_time:.2f} seconds")
        return np.array(cloud_points)
    
    def format_value(self, value):
        """格式化数值显示，根据大小自动调整精度"""
        if abs(value) < 0.01:
            return f"{value:.6f}"
        elif abs(value) < 1:
            return f"{value:.4f}"
        else:
            return f"{value:.3f}"
    
    def create_control_panel(self):
        # 配置主窗口
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        
        # 创建可滚动的画布
        canvas_frame = tk.Frame(self.root)
        canvas_frame.grid(row=0, column=0, sticky="nsew")
        
        canvas = tk.Canvas(canvas_frame)
        scrollbar = ttk.Scrollbar(canvas_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 控制面板
        control_frame = ttk.Frame(scrollable_frame, padding="15")
        control_frame.grid(row=0, column=0, sticky="nsew")
        control_frame.grid_columnconfigure(1, weight=1)
        
        # 标题
        title_label = ttk.Label(control_frame, text="Point Cloud Range Adjuster", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        row = 1
        for axis in ['x', 'y', 'z']:
            frame = ttk.LabelFrame(control_frame, text=f"{axis.upper()} Axis Range", padding="10")
            frame.grid(row=row, column=0, sticky="ew", padx=5, pady=10, columnspan=3)
            frame.grid_columnconfigure(1, weight=1)
            frame.grid_columnconfigure(3, weight=1)
            
            # 当前范围显示
            range_text = f"Current: [{self.format_value(self.ranges[axis][0])}, {self.format_value(self.ranges[axis][1])}]"
            range_label = ttk.Label(frame, text=range_text, font=("Arial", 10))
            range_label.grid(row=0, column=0, columnspan=4, padx=5, pady=5, sticky="w")
            setattr(self, f"{axis}_range_label", range_label)
            
            # 原始范围显示
            original_text = f"Original: [{self.format_value(self.original_ranges[axis][0])}, {self.format_value(self.original_ranges[axis][1])}]"
            original_label = ttk.Label(frame, text=original_text, font=("Arial", 9, "italic"), foreground="gray")
            original_label.grid(row=1, column=0, columnspan=4, padx=5, pady=(0, 10), sticky="w")
            
            # 最小值控制
            min_frame = ttk.Frame(frame)
            min_frame.grid(row=2, column=0, columnspan=4, sticky="ew", padx=5, pady=2)
            min_frame.grid_columnconfigure(1, weight=1)
            
            min_label = ttk.Label(min_frame, text="Min:", width=8)
            min_label.grid(row=0, column=0, padx=(0, 5))
            
            # 长滑动条
            min_slider = ttk.Scale(
                min_frame,
                from_=self.slider_ranges[axis][0],
                to=self.slider_ranges[axis][1],
                value=self.ranges[axis][0],
                length=SLIDER_LENGTH,
                command=lambda v, a=axis: self.update_range(a, 'min', float(v))
            )
            min_slider.grid(row=0, column=1, sticky="ew", padx=5)
            
            min_value = ttk.Entry(min_frame, width=12)
            min_value.insert(0, self.format_value(self.ranges[axis][0]))
            min_value.grid(row=0, column=2, padx=(5, 0))
            min_value.bind("<Return>", lambda e, a=axis: self.update_range_from_entry(a, 'min'))
            
            setattr(self, f"{axis}_min_slider", min_slider)
            setattr(self, f"{axis}_min_value", min_value)
            
            # 最大值控制
            max_frame = ttk.Frame(frame)
            max_frame.grid(row=3, column=0, columnspan=4, sticky="ew", padx=5, pady=2)
            max_frame.grid_columnconfigure(1, weight=1)
            
            max_label = ttk.Label(max_frame, text="Max:", width=8)
            max_label.grid(row=0, column=0, padx=(0, 5))
            
            # 长滑动条
            max_slider = ttk.Scale(
                max_frame,
                from_=self.slider_ranges[axis][0],
                to=self.slider_ranges[axis][1],
                value=self.ranges[axis][1],
                length=SLIDER_LENGTH,
                command=lambda v, a=axis: self.update_range(a, 'max', float(v))
            )
            max_slider.grid(row=0, column=1, sticky="ew", padx=5)
            
            max_value = ttk.Entry(max_frame, width=12)
            max_value.insert(0, self.format_value(self.ranges[axis][1]))
            max_value.grid(row=0, column=2, padx=(5, 0))
            max_value.bind("<Return>", lambda e, a=axis: self.update_range_from_entry(a, 'max'))
            
            setattr(self, f"{axis}_max_slider", max_slider)
            setattr(self, f"{axis}_max_value", max_value)
            
            row += 1
        
        # 按钮面板
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=row, column=0, sticky="ew", padx=5, pady=20, columnspan=3)
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(1, weight=1)
        button_frame.grid_columnconfigure(2, weight=1)
        button_frame.grid_columnconfigure(3, weight=1)
        
        save_button = ttk.Button(button_frame, text="Print Parameters", command=self.save_to_config)
        save_button.grid(row=0, column=0, padx=5, sticky="ew")
        
        reset_view_button = ttk.Button(button_frame, text="Reset View", command=self.reset_to_initial_view)
        reset_view_button.grid(row=0, column=1, padx=5, sticky="ew")
        
        reset_range_button = ttk.Button(button_frame, text="Reset Ranges", command=self.reset_ranges)
        reset_range_button.grid(row=0, column=2, padx=5, sticky="ew")
        
        export_button = ttk.Button(button_frame, text="Export Config", command=self.export_config)
        export_button.grid(row=0, column=3, padx=5, sticky="ew")
        
        row += 1
        
        # 参数预览
        preview_frame = ttk.LabelFrame(control_frame, text="Parameters Preview", padding="10")
        preview_frame.grid(row=row, column=0, sticky="ew", padx=5, pady=10, columnspan=3)
        preview_frame.grid_columnconfigure(0, weight=1)
        
        self.param_preview = tk.Text(preview_frame, height=8, width=60, font=("Courier", 10))
        self.param_preview.insert("1.0", self.get_param_string())
        self.param_preview.config(state="disabled")
        self.param_preview.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        row += 1
        
        # 状态信息
        status_frame = ttk.Frame(control_frame)
        status_frame.grid(row=row, column=0, sticky="ew", padx=5, pady=5, columnspan=3)
        
        self.status_label = ttk.Label(status_frame, text="Ready", foreground="green")
        self.status_label.pack(anchor="w")
        
        # 绑定鼠标滚轮事件用于滚动
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
    
    def update_range_from_entry(self, axis, range_type):
        try:
            entry = getattr(self, f"{axis}_{range_type}_value")
            value = float(entry.get())
            
            # 确保值在有效范围内
            min_range = self.slider_ranges[axis][0]
            max_range = self.slider_ranges[axis][1]
            
            if value < min_range:
                value = min_range
            elif value > max_range:
                value = max_range
            
            # 更新滑动条
            slider = getattr(self, f"{axis}_{range_type}_slider")
            slider.set(value)
            
            # 更新范围
            self.update_range(axis, range_type, value)
            
        except ValueError:
            # 输入无效，恢复原值
            current_val = self.ranges[axis][0] if range_type == 'min' else self.ranges[axis][1]
            entry.delete(0, tk.END)
            entry.insert(0, self.format_value(current_val))
    
    def update_range(self, axis, range_type, value):
        idx = 0 if range_type == 'min' else 1
        self.ranges[axis][idx] = value
        
        # 更新显示
        entry = getattr(self, f"{axis}_{range_type}_value")
        entry.delete(0, tk.END)
        entry.insert(0, self.format_value(value))
        
        # 更新范围标签
        range_text = f"Current: [{self.format_value(self.ranges[axis][0])}, {self.format_value(self.ranges[axis][1])}]"
        getattr(self, f"{axis}_range_label").config(text=range_text)
        
        # 更新参数预览
        self.param_preview.config(state="normal")
        self.param_preview.delete("1.0", tk.END)
        self.param_preview.insert("1.0", self.get_param_string())
        self.param_preview.config(state="disabled")
        
        # 更新点云显示
        self.update_pointcloud_display()
        
        # 更新状态
        self.status_label.config(text=f"Updated {axis.upper()}{range_type.title()}: {self.format_value(value)}")
    
    def get_param_string(self):
        return (
            f"x_min: {self.ranges['x'][0]:.8f}\n"
            f"x_max: {self.ranges['x'][1]:.8f}\n"
            f"y_min: {self.ranges['y'][0]:.8f}\n"
            f"y_max: {self.ranges['y'][1]:.8f}\n"
            f"z_min: {self.ranges['z'][0]:.8f}\n"
            f"z_max: {self.ranges['z'][1]:.8f}"
        )
    
    def save_current_view(self):
        """保存当前视角"""
        try:
            ctr = self.vis.get_view_control()
            self.current_view = ctr.convert_to_pinhole_camera_parameters()
        except Exception as e:
            print(f"Warning: Failed to save current view: {e}")
    
    def restore_current_view(self):
        """恢复保存的视角"""
        if self.current_view:
            try:
                ctr = self.vis.get_view_control()
                ctr.convert_from_pinhole_camera_parameters(self.current_view, allow_arbitrary=True)
            except Exception as e:
                print(f"Warning: Failed to restore view: {e}")
    
    def update_pointcloud_display(self, init_view=False):
        if len(self.original_cloud) == 0:
            return
        
        # 在更新前保存当前视角
        self.save_current_view()
        
        # 应用范围过滤
        x_mask = (self.original_cloud[:, 0] >= self.ranges['x'][0]) & \
                 (self.original_cloud[:, 0] <= self.ranges['x'][1])
        y_mask = (self.original_cloud[:, 1] >= self.ranges['y'][0]) & \
                 (self.original_cloud[:, 1] <= self.ranges['y'][1])
        z_mask = (self.original_cloud[:, 2] >= self.ranges['z'][0]) & \
                 (self.original_cloud[:, 2] <= self.ranges['z'][1])
        filtered_points = self.original_cloud[x_mask & y_mask & z_mask]
        
        if len(filtered_points) == 0:
            self.status_label.config(text="WARNING: No points in current range. Adjust filters.", foreground="orange")
            return
        
        self.status_label.config(text=f"Displaying {len(filtered_points)} points", foreground="green")
        
        # 更新点云
        self.filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)
        self.vis.clear_geometries()
        self.vis.add_geometry(self.filtered_cloud)
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.vis.add_geometry(coordinate_frame)
        
        # 设置视角
        ctr = self.vis.get_view_control()
        if init_view:
            # 如果是初始化，设置默认视角并保存
            ctr.set_front(VIEW_FRONT)
            ctr.set_up(VIEW_UP)
            ctr.set_zoom(VIEW_ZOOM)
            self.current_view = ctr.convert_to_pinhole_camera_parameters()
        else:
            # 恢复之前保存的视角
            self.restore_current_view()
        
        self.vis.poll_events()
        self.vis.update_renderer()
    
    def reset_to_initial_view(self):
        """重置到初始视角"""
        ctr = self.vis.get_view_control()
        ctr.set_front(VIEW_FRONT)
        ctr.set_up(VIEW_UP)
        ctr.set_zoom(VIEW_ZOOM)
        # 保存为新的当前视角
        self.current_view = ctr.convert_to_pinhole_camera_parameters()
        self.vis.update_renderer()
        self.status_label.config(text="View reset to initial position", foreground="blue")
    
    def reset_ranges(self):
        """重置所有范围到原始值"""
        for axis in ['x', 'y', 'z']:
            self.ranges[axis] = self.original_ranges[axis].copy()
            
            # 更新滑动条
            getattr(self, f"{axis}_min_slider").set(self.ranges[axis][0])
            getattr(self, f"{axis}_max_slider").set(self.ranges[axis][1])
            
            # 更新输入框
            getattr(self, f"{axis}_min_value").delete(0, tk.END)
            getattr(self, f"{axis}_min_value").insert(0, self.format_value(self.ranges[axis][0]))
            getattr(self, f"{axis}_max_value").delete(0, tk.END)
            getattr(self, f"{axis}_max_value").insert(0, self.format_value(self.ranges[axis][1]))
            
            # 更新范围标签
            range_text = f"Current: [{self.format_value(self.ranges[axis][0])}, {self.format_value(self.ranges[axis][1])}]"
            getattr(self, f"{axis}_range_label").config(text=range_text)
        
        # 更新预览
        self.param_preview.config(state="normal")
        self.param_preview.delete("1.0", tk.END)
        self.param_preview.insert("1.0", self.get_param_string())
        self.param_preview.config(state="disabled")
        
        # 更新点云显示
        self.update_pointcloud_display()
        self.status_label.config(text="All ranges reset to original values", foreground="blue")
    
    def export_config(self):
        """导出配置到文件"""
        try:
            export_path = os.path.join(os.path.dirname(self.config_path), "exported_ranges.yaml")
            config_data = {
                'x_min': float(self.ranges['x'][0]),
                'x_max': float(self.ranges['x'][1]),
                'y_min': float(self.ranges['y'][0]),
                'y_max': float(self.ranges['y'][1]),
                'z_min': float(self.ranges['z'][0]),
                'z_max': float(self.ranges['z'][1])
            }
            
            with open(export_path, 'w') as f:
                yaml.dump(config_data, f, default_flow_style=False)
            
            self.status_label.config(text=f"Config exported to {export_path}", foreground="green")
            print(f"Config exported to: {export_path}")
        except Exception as e:
            self.status_label.config(text=f"Export failed: {str(e)}", foreground="red")
            print(f"Export error: {e}")
    
    def save_to_config(self):
        print("=== Current Parameters ===")
        print(self.get_param_string())
        print("==========================")
        self.status_label.config(text="Parameters printed to console", foreground="blue")
    
    def run(self):
        self.root.after(100, self.check_vis_window)
        self.root.mainloop()
    
    def check_vis_window(self):
        try:
            if not self.vis.poll_events():
                self.root.destroy()
                return
        except:
            self.root.destroy()
            return
        self.root.after(100, self.check_vis_window)

if __name__ == "__main__":
    config_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONFIG_PATH
    rclpy.init()
    try:
        adjuster = PointCloudRangeAdjuster(config_path)
        adjuster.run()
    finally:
        rclpy.shutdown()