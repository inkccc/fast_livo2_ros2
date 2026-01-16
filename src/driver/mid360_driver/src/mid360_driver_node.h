/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "mid360_driver.h"
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mid360_driver {

    // 新增：简单一阶低通滤波器类（指数平滑）
    class LowPassFilter {
    private:
        bool initialized = false;
        float filtered_value = 0.0f;

    public:
        float update(float new_value, float alpha) {
            if (!initialized) {
                initialized = true;
                filtered_value = new_value;
            } else {
                filtered_value = alpha * new_value + (1.0f - alpha) * filtered_value;
            }
            return filtered_value;
        }

        void reset() {
            initialized = false;
        }
    };

    class LidarPublisher {
    private:
        std::vector<Point> points_wait_to_publish;
        std::vector<ImuMsg> imu_wait_to_publish;
        std::vector<Point> points_to_publish;
        std::vector<ImuMsg> imu_to_publish;
        bool is_init = false;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher;

    public:
        LidarPublisher() = default;

        void make_sure_init(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic);

        void make_sure_init(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic, const asio::ip::address &lidar_ip);

        void on_receive_pointcloud(const std::vector<Point> &points);

        void on_receive_imu(const ImuMsg &imu_msg);

        void prepare_pointcloud_to_publish();

        void prepare_imu_to_publish();

        void publish_pointcloud(const std::string &frame_id) const;

        void publish_imu(const std::string &frame_id) const;
    };

    class Mid360DriverNode : public rclcpp::Node {
    private:
        asio::io_context io_context;
        std::mutex mutex;
        std::thread io_thread;
        std::unique_ptr<mid360_driver::Mid360Driver> mid360_driver;
        LidarPublisher lidar_publisher;
        std::unordered_map<asio::ip::address, LidarPublisher, IpAddressHasher> muti_lidar_publisher;
        std::vector<std::pair<asio::ip::address, LidarPublisher *>> muti_lidar_publisher_temp;
        rclcpp::TimerBase::SharedPtr publish_pointcloud_timer;
        rclcpp::TimerBase::SharedPtr publish_imu_timer;
        
        // TF广播器
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr tf_timer_;

        // === 新增：IMU低通滤波相关 ===
        bool imu_filter_enable_;
        float imu_filter_alpha_;
        std::string imu_filtered_topic_;
        
        // 每个轴的独立滤波器（角速度 + 加速度 = 6个）
        LowPassFilter filter_ang_vel_x_;
        LowPassFilter filter_ang_vel_y_;
        LowPassFilter filter_ang_vel_z_;
        LowPassFilter filter_lin_acc_x_;
        LowPassFilter filter_lin_acc_y_;
        LowPassFilter filter_lin_acc_z_;

        // 新增：滤波后IMU发布器（单设备模式下）
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_filtered_publisher_;

        // 多设备模式下需要动态创建（暂不实现复杂逻辑，因为目前默认单设备）

        void publish_tf_transforms(const std::string &lidar_frame, 
                                  const std::string &imu_frame,
                                  const std::vector<double> &extrinsic_T,
                                  const std::vector<double> &extrinsic_R);

        // 新增：发布滤波后的IMU消息
        void publish_filtered_imu(const ImuMsg &raw_imu, const std::string &frame_id);

    public:
        explicit Mid360DriverNode(const rclcpp::NodeOptions &options);

        ~Mid360DriverNode();
    };

}// namespace mid360_driver