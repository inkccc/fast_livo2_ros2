/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#define ASIO_NO_DEPRECATED
#include <asio.hpp>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <vector>
#include <array>
#include <cmath>

namespace mid360_driver {

    struct Point {
        double timestamp;
        float x, y, z;
        float intensity;
    };

    struct ImuMsg {
        double timestamp;
        float angular_velocity_x;
        float angular_velocity_y;
        float angular_velocity_z;
        float linear_acceleration_x;
        float linear_acceleration_y;
        float linear_acceleration_z;
    };

    struct IpAddressHasher {
        std::size_t operator()(const asio::ip::address &addr) const noexcept;
    };

    class Transform {
    private:
        std::array<std::array<float, 4>, 4> transform_matrix;
        bool enabled;
        float min_distance;
        float max_distance;
        bool distance_filter_enabled;

    public:
        Transform() : enabled(false), min_distance(-1.0f), max_distance(-1.0f), distance_filter_enabled(false) {
            // 初始化单位矩阵
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    transform_matrix[i][j] = (i == j) ? 1.0f : 0.0f;
                }
            }
        }

        void setTransform(float roll_deg, float pitch_deg, float yaw_deg, float tx, float ty, float tz, bool enable) {
            enabled = enable;
            if (!enabled) {
                // 重置为单位矩阵
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        transform_matrix[i][j] = (i == j) ? 1.0f : 0.0f;
                    }
                }
                return;
            }

            // 将角度从度转换为弧度
            float roll = roll_deg * M_PI / 180.0f;
            float pitch = pitch_deg * M_PI / 180.0f;
            float yaw = yaw_deg * M_PI / 180.0f;

            // 计算旋转矩阵（ZYX顺序：yaw -> pitch -> roll）
            float cr = cos(roll);
            float sr = sin(roll);
            float cp = cos(pitch);
            float sp = sin(pitch);
            float cy = cos(yaw);
            float sy = sin(yaw);

            // 旋转矩阵 R = Rz(yaw) * Ry(pitch) * Rx(roll)
            transform_matrix[0][0] = cy * cp;
            transform_matrix[0][1] = cy * sp * sr - sy * cr;
            transform_matrix[0][2] = cy * sp * cr + sy * sr;
            transform_matrix[0][3] = tx;

            transform_matrix[1][0] = sy * cp;
            transform_matrix[1][1] = sy * sp * sr + cy * cr;
            transform_matrix[1][2] = sy * sp * cr - cy * sr;
            transform_matrix[1][3] = ty;

            transform_matrix[2][0] = -sp;
            transform_matrix[2][1] = cp * sr;
            transform_matrix[2][2] = cp * cr;
            transform_matrix[2][3] = tz;

            transform_matrix[3][0] = 0.0f;
            transform_matrix[3][1] = 0.0f;
            transform_matrix[3][2] = 0.0f;
            transform_matrix[3][3] = 1.0f;
        }

        void setDistanceFilter(float min_dist, float max_dist) {
            min_distance = min_dist;
            max_distance = max_dist;
            distance_filter_enabled = (min_dist >= 0.0f) || (max_dist >= 0.0f);
        }

        bool filterPointByDistance(float x, float y, float z) const {
            if (!distance_filter_enabled) {
                return true; // 不过滤
            }
            
            float distance = std::sqrt(x*x + y*y + z*z);
            
            // 检查最小距离条件
            if (min_distance >= 0.0f && distance < min_distance) {
                return false; // 过滤掉
            }
            
            // 检查最大距离条件
            if (max_distance >= 0.0f && distance > max_distance) {
                return false; // 过滤掉
            }
            
            return true; // 保留
        }

        void transformPoint(float &x, float &y, float &z) const {
            if (!enabled) return;

            float x_new = transform_matrix[0][0] * x + transform_matrix[0][1] * y + transform_matrix[0][2] * z + transform_matrix[0][3];
            float y_new = transform_matrix[1][0] * x + transform_matrix[1][1] * y + transform_matrix[1][2] * z + transform_matrix[1][3];
            float z_new = transform_matrix[2][0] * x + transform_matrix[2][1] * y + transform_matrix[2][2] * z + transform_matrix[2][3];

            x = x_new;
            y = y_new;
            z = z_new;
        }

        void transformVector(float &vx, float &vy, float &vz) const {
            if (!enabled) return;

            // 只应用旋转，不应用平移
            float vx_new = transform_matrix[0][0] * vx + transform_matrix[0][1] * vy + transform_matrix[0][2] * vz;
            float vy_new = transform_matrix[1][0] * vx + transform_matrix[1][1] * vy + transform_matrix[1][2] * vz;
            float vz_new = transform_matrix[2][0] * vx + transform_matrix[2][1] * vy + transform_matrix[2][2] * vz;

            vx = vx_new;
            vy = vy_new;
            vz = vz_new;
        }

        bool isEnabled() const { return enabled; }
        bool isDistanceFilterEnabled() const { return distance_filter_enabled; }
        float getMinDistance() const { return min_distance; }
        float getMaxDistance() const { return max_distance; }
    };

    class Mid360Driver {
    private:
        std::atomic<bool> is_running = true;
        asio::ip::address host_ip;
        asio::ip::udp::socket receive_pointcloud_socket;
        asio::ip::udp::socket receive_imu_socket;
        std::vector<Point> points;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> delta_time_map;
        std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> on_receive_pointcloud;
        std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> on_receive_imu;
        Transform transform;

    public:
        Mid360Driver(asio::io_context &io_context,
                     const asio::ip::address &host_ip,
                     const std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> &on_receive_pointcloud,
                     const std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> &on_receive_imu,
                     const Transform &transform = Transform());

        ~Mid360Driver();

        void stop();

        void setTransform(const Transform &new_transform);

        asio::awaitable<void> receive_pointcloud();

        asio::awaitable<void> receive_imu();
    };

}// namespace mid360_driver