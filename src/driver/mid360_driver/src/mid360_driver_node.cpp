/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#define _USE_MATH_DEFINES
#include "mid360_driver_node.h"
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mid360_driver {

    void LidarPublisher::make_sure_init(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic) {
        if (!is_init) {
            pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, 1000);
            imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1000);
            is_init = true;
        }
    }

    void LidarPublisher::make_sure_init(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic, const asio::ip::address &lidar_ip) {
        if (!is_init) {
            auto lidar_ip_bytes = lidar_ip.to_v4().to_bytes();
            std::string lidar_ip_str;
            // lidar_ip_str.push_back('_');
            // lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[0])));
            // lidar_ip_str.push_back('_');
            // lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[1])));
            // lidar_ip_str.push_back('_');
            // lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[2])));
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[3])));
            pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic + lidar_ip_str, 1000);
            imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(imu_topic + lidar_ip_str, 1000);
            is_init = true;
        }
    }

    void LidarPublisher::on_receive_pointcloud(const std::vector<Point> &points) {
        points_wait_to_publish.insert(points_wait_to_publish.end(), points.begin(), points.end());
    }

    void LidarPublisher::on_receive_imu(const ImuMsg &imu_msg) {
        imu_wait_to_publish.push_back(imu_msg);
    }

    void LidarPublisher::prepare_pointcloud_to_publish() {
        std::swap(points_wait_to_publish, points_to_publish);
        points_wait_to_publish.clear();
    }

    void LidarPublisher::prepare_imu_to_publish() {
        std::swap(imu_wait_to_publish, imu_to_publish);
        imu_wait_to_publish.clear();
    }

    void LidarPublisher::publish_pointcloud(const std::string &frame_id) const {
        if (points_to_publish.empty()) {
            return;
        }
        double timestamp = std::numeric_limits<double>::max();
        for (const auto &point: points_to_publish) {
            if (point.timestamp < timestamp) {
                timestamp = point.timestamp;
            }
        }
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp.sec = std::floor(timestamp);
        msg.header.stamp.nanosec = static_cast<uint32_t>((timestamp - msg.header.stamp.sec) * 1e9);
        msg.header.frame_id = frame_id;
        msg.width = points_to_publish.size();
        msg.height = 1;
        msg.fields.reserve(4);
        sensor_msgs::msg::PointField field;
        field.name = "x";
        field.offset = 0;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "y";
        field.offset = 4;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "z";
        field.offset = 8;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "intensity";
        field.offset = 12;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "timestamp";
        field.offset = 16;
        field.datatype = sensor_msgs::msg::PointField::FLOAT64;
        field.count = 1;
        msg.fields.push_back(field);
        msg.is_bigendian = false;
        msg.point_step = 24;
        msg.row_step = msg.width * msg.point_step;
        msg.data.resize(msg.row_step * msg.height);
        auto pointer = reinterpret_cast<float *>(msg.data.data());
        for (const auto &point: points_to_publish) {
            *pointer = point.x;
            ++pointer;
            *pointer = point.y;
            ++pointer;
            *pointer = point.z;
            ++pointer;
            *pointer = point.intensity;
            ++pointer;
            *reinterpret_cast<double *>(pointer) = point.timestamp;
            pointer += 2;
        }
        msg.is_dense = true;
        pointcloud_publisher->publish(msg);
    }

    void LidarPublisher::publish_imu(const std::string &frame_id) const {
        for (const auto &imu: imu_to_publish) {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp.sec = std::floor(imu.timestamp);
            msg.header.stamp.nanosec = static_cast<uint32_t>((imu.timestamp - msg.header.stamp.sec) * 1e9);
            msg.header.frame_id = frame_id;
            msg.angular_velocity.x = imu.angular_velocity_x;
            msg.angular_velocity.y = imu.angular_velocity_y;
            msg.angular_velocity.z = imu.angular_velocity_z;
            msg.linear_acceleration.x = imu.linear_acceleration_x;
            msg.linear_acceleration.y = imu.linear_acceleration_y;
            msg.linear_acceleration.z = imu.linear_acceleration_z;
            imu_publisher->publish(msg);
        }
    }

    // 新增函数：发布滤波后的IMU（仅用于单设备模式）
    void Mid360DriverNode::publish_filtered_imu(const ImuMsg &raw_imu, const std::string &frame_id) {
        if (!imu_filter_enable_ || !imu_filtered_publisher_) {
            return;
        }

        sensor_msgs::msg::Imu msg;
        msg.header.stamp.sec = std::floor(raw_imu.timestamp);
        msg.header.stamp.nanosec = static_cast<uint32_t>((raw_imu.timestamp - msg.header.stamp.sec) * 1e9);
        msg.header.frame_id = frame_id;

        // 对每个轴进行独立一阶低通滤波
        msg.angular_velocity.x = filter_ang_vel_x_.update(raw_imu.angular_velocity_x, imu_filter_alpha_);
        msg.angular_velocity.y = filter_ang_vel_y_.update(raw_imu.angular_velocity_y, imu_filter_alpha_);
        msg.angular_velocity.z = filter_ang_vel_z_.update(raw_imu.angular_velocity_z, imu_filter_alpha_);

        msg.linear_acceleration.x = filter_lin_acc_x_.update(raw_imu.linear_acceleration_x, imu_filter_alpha_);
        msg.linear_acceleration.y = filter_lin_acc_y_.update(raw_imu.linear_acceleration_y, imu_filter_alpha_);
        msg.linear_acceleration.z = filter_lin_acc_z_.update(raw_imu.linear_acceleration_z, imu_filter_alpha_);

        // 协方差暂不处理（保持默认0）
        imu_filtered_publisher_->publish(msg);
    }

    Mid360DriverNode::Mid360DriverNode(const rclcpp::NodeOptions &options)
        : Node("mid360_driver_node", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame", "livox_frame");
        std::string imu_topic = declare_parameter<std::string>("imu_topic", "/livox/imu");
        imu_filtered_topic_ = declare_parameter<std::string>("imu_filtered_topic", "/livox/imu_filtered");
        std::string imu_frame = declare_parameter<std::string>("imu_frame", "imu_frame");
        std::string host_ip = declare_parameter<std::string>("host_ip");
        double lidar_publish_time_interval = declare_parameter<double>("lidar_publish_time_interval", 0.1);
        bool is_topic_name_with_lidar_ip = declare_parameter<bool>("is_topic_name_with_lidar_ip",false); //这个功能暂时关闭
        bool publish_tf = declare_parameter<bool>("publish_tf", false);
        
        // 读取坐标变换参数 默认不进行变换
        bool transform_enable = declare_parameter<bool>("transform_enable", false);
        float rotation_roll = declare_parameter<float>("rotation_roll", 0.0f);
        float rotation_pitch = declare_parameter<float>("rotation_pitch", 0.0f);
        float rotation_yaw = declare_parameter<float>("rotation_yaw", 0.0f);
        float translation_x = declare_parameter<float>("translation_x", 0.0f);
        float translation_y = declare_parameter<float>("translation_y", 0.0f);
        float translation_z = declare_parameter<float>("translation_z", 0.0f);
        
        // 读取点云距离筛选参数
        float min_point_distance = declare_parameter<float>("min_point_distance", -1.0f);
        float max_point_distance = declare_parameter<float>("max_point_distance", -1.0f);

        // === 新增：读取IMU滤波参数 ===
        imu_filter_enable_ = declare_parameter<bool>("imu_filter_enable", false);
        imu_filter_alpha_ = declare_parameter<float>("imu_filter_alpha", 0.1f);
        
        // 创建变换对象并设置参数
        Transform transform;
        transform.setTransform(rotation_roll, rotation_pitch, rotation_yaw, 
                              translation_x, translation_y, translation_z, 
                              transform_enable);
        
        // 设置距离筛选参数
        transform.setDistanceFilter(min_point_distance, max_point_distance);
        
        // 如果开启距离筛选，打印配置信息
        if (min_point_distance != -1.0f || max_point_distance != -1.0f) {
            RCLCPP_INFO(this->get_logger(), "Distance filter configuration:");
            RCLCPP_INFO(this->get_logger(), "Min distance: %.2f m", min_point_distance);
            RCLCPP_INFO(this->get_logger(), "Max distance: %.2f m", max_point_distance);
        }

        // 如果开启IMU滤波，打印配置信息
        if (imu_filter_enable_) {
            if (imu_filter_alpha_ <= 0.0f || imu_filter_alpha_ > 1.0f) {
                RCLCPP_WARN(this->get_logger(), "imu_filter_alpha must be in (0, 1], clamping to 0.1");
                imu_filter_alpha_ = 0.1f;
            }
            RCLCPP_INFO(this->get_logger(), "IMU low-pass filter ENABLED with alpha = %.3f", imu_filter_alpha_);
            RCLCPP_INFO(this->get_logger(), "Filtered IMU will be published on: %s", imu_filtered_topic_.c_str());
        }
        
        if (!is_topic_name_with_lidar_ip) {
            lidar_publisher.make_sure_init(*this, lidar_topic, imu_topic);
            // 新增：如果启用滤波，创建滤波后IMU发布器
            if (imu_filter_enable_) {
                imu_filtered_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_filtered_topic_, 1000);
            }
        }
        mid360_driver = std::make_unique<mid360_driver::Mid360Driver>(
            io_context,
            asio::ip::make_address(host_ip),
            [this, is_topic_name_with_lidar_ip](const asio::ip::address &lidar_ip, const std::vector<Point> &points) {
                mutex.lock();
                if (is_topic_name_with_lidar_ip) {
                    auto iter = muti_lidar_publisher.try_emplace(lidar_ip).first;
                    iter->second.on_receive_pointcloud(points);
                } else {
                    lidar_publisher.on_receive_pointcloud(points);
                }
                mutex.unlock();
            },
            // 修改IMU回调：在原始数据进入缓冲前，先进行滤波并发布
            [this, is_topic_name_with_lidar_ip, imu_frame](const asio::ip::address &lidar_ip, const ImuMsg &imu_msg) {
                mutex.lock();

                // 始终将原始IMU加入待发布队列（保持原有话题不变）
                if (is_topic_name_with_lidar_ip) {
                    auto iter = muti_lidar_publisher.try_emplace(lidar_ip).first;
                    iter->second.on_receive_imu(imu_msg);
                } else {
                    lidar_publisher.on_receive_imu(imu_msg);
                }

                // 新增：如果启用滤波，立即发布滤波后数据（不进缓冲队列，直接发）
                // 这样可以减少延迟，且避免与定时发布冲突
                if (!is_topic_name_with_lidar_ip && imu_filter_enable_) {
                    this->publish_filtered_imu(imu_msg, imu_frame);
                }

                mutex.unlock();
            },
            transform
        );
        
        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        
        // 雷达到IMU的外参变换（硬编码在代码中）
        // extrinsic_T: [ -0.011, -0.02329, 0.04412 ] # imu
        // extrinsic_R: [1.0, 0.0, 0.0,
        //              0.0, 1.0, 0.0,
        //              0.0, 0.0, 1.0]
        const std::vector<double> extrinsic_T = {0.011, 0.02329, -0.04412};
        const std::vector<double> extrinsic_R = {1.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0,
                                                0.0, 0.0, 1.0};
        
        if (publish_tf) {
            // 创建TF发布定时器
            tf_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),  // 10Hz
                [this, lidar_frame, imu_frame, extrinsic_T, extrinsic_R]() {
                    this->publish_tf_transforms(lidar_frame, imu_frame, extrinsic_T, extrinsic_R);
                });
        }
        
        if (is_topic_name_with_lidar_ip) {
            publish_pointcloud_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this, lidar_topic, imu_topic, lidar_frame]() {
                mutex.lock();
                for (auto &[lidar_ip, lidar_publisher]: muti_lidar_publisher) {
                    lidar_publisher.prepare_pointcloud_to_publish();
                }
                if (muti_lidar_publisher_temp.size() != muti_lidar_publisher.size()) {
                    muti_lidar_publisher_temp.clear();
                    for (auto &[lidar_ip, lidar_publisher]: muti_lidar_publisher) {
                        muti_lidar_publisher_temp.emplace_back(lidar_ip, &lidar_publisher);
                    }
                }
                mutex.unlock();
                for (auto &[lidar_ip, lidar_publisher]: muti_lidar_publisher_temp) {
                    lidar_publisher->make_sure_init(*this, lidar_topic, imu_topic, lidar_ip);
                    lidar_publisher->publish_pointcloud(lidar_frame);
                }
            });
            publish_imu_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1), [this, lidar_topic, imu_topic, imu_frame]() {
                muti_lidar_publisher_temp.clear();
                mutex.lock();
                for (auto &[lidar_ip, lidar_publisher]: muti_lidar_publisher) {
                    lidar_publisher.prepare_imu_to_publish();
                }
                if (muti_lidar_publisher_temp.size() != muti_lidar_publisher.size()) {
                    muti_lidar_publisher_temp.clear();
                    for (auto &[lidar_ip, lidar_publisher]: muti_lidar_publisher) {
                        muti_lidar_publisher_temp.emplace_back(lidar_ip, &lidar_publisher);
                    }
                }
                mutex.unlock();
                for (auto &[lidar_ip, lidar_publisher]: muti_lidar_publisher_temp) {
                    lidar_publisher->make_sure_init(*this, lidar_topic, imu_topic, lidar_ip);
                    lidar_publisher->publish_imu(imu_frame);
                }
            });
        } else {
            publish_pointcloud_timer = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double, std::ratio<1, 1>>(lidar_publish_time_interval), [this, lidar_frame]() {
                mutex.lock();
                lidar_publisher.prepare_pointcloud_to_publish();
                mutex.unlock();
                lidar_publisher.publish_pointcloud(lidar_frame);
            });
            publish_imu_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1), [this, imu_frame]() {
                mutex.lock();
                lidar_publisher.prepare_imu_to_publish();
                mutex.unlock();
                lidar_publisher.publish_imu(imu_frame);
            });
        }
        io_thread = std::thread([this]() {
            io_context.run();
        });
    }

    void Mid360DriverNode::publish_tf_transforms(const std::string &lidar_frame, 
                                               const std::string &imu_frame,
                                               const std::vector<double> &extrinsic_T,
                                               const std::vector<double> &extrinsic_R) {
        auto now = this->now();
        
        // 发布雷达到IMU的变换
        geometry_msgs::msg::TransformStamped lidar_to_imu_tf;
        lidar_to_imu_tf.header.stamp = now;
        lidar_to_imu_tf.header.frame_id = lidar_frame;
        lidar_to_imu_tf.child_frame_id = imu_frame;
        
        // 设置平移
        lidar_to_imu_tf.transform.translation.x = extrinsic_T[0];
        lidar_to_imu_tf.transform.translation.y = extrinsic_T[1];
        lidar_to_imu_tf.transform.translation.z = extrinsic_T[2];
        
        // 设置旋转（从旋转矩阵转换为四元数）
        // 旋转矩阵是单位矩阵，对应的四元数为 (0, 0, 0, 1)
        lidar_to_imu_tf.transform.rotation.x = 0.0;
        lidar_to_imu_tf.transform.rotation.y = 0.0;
        lidar_to_imu_tf.transform.rotation.z = 0.0;
        lidar_to_imu_tf.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(lidar_to_imu_tf);
    }

    Mid360DriverNode::~Mid360DriverNode() {
        if (mid360_driver) {
            mid360_driver->stop();
        }
        io_thread.join();
    }

}// namespace mid360_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mid360_driver::Mid360DriverNode)