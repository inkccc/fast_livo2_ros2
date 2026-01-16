/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <iostream>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace cv;

class DataPreprocess
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_;

    cv::Mat img_input_;

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        string bag_path = params.bag_path;
        string image_path = params.image_path;
        string lidar_topic = params.lidar_topic;
        string output_path = params.output_path;

        // Load image
        img_input_ = cv::imread(params.image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            std::string msg = "Loading the image " + image_path + " failed";
            std::cerr << msg << std::endl;
            return;
        }
        std::cout << "Successfully loaded image: " << image_path << std::endl;

        // Try to load point cloud from ROS2 bag file
        std::cout << "Attempting to load point cloud from ROS2 bag: " << bag_path << std::endl;
        std::cout << "Looking for topic: " << lidar_topic << std::endl;

        loadPointCloudFromBag(bag_path, lidar_topic, cloud_input_);
        // pcl::io::savePLYFile(output_path + "/cloud_input.ply", *cloud_input_);
    }

private:
    bool loadPointCloudFromBag(
        const std::string &bag_path,
        const std::string &topic_name,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        try
        {
            // 创建bag读取器
            rosbag2_cpp::Reader reader;
            reader.open(bag_path);

            // 设置序列化格式
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

            while (reader.has_next())
            {
                auto bag_message = reader.read_next();

                // 检查是否是目标topic
                if (bag_message->topic_name != topic_name)
                {
                    continue;
                }

                // 反序列化消息
                auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&extracted_serialized_msg, ros_msg.get());

                // 转换为PCL点云
                pcl::PointCloud<pcl::PointXYZ>::Ptr frame(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*ros_msg, *frame);
                *cloud += *frame;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading bag file: " << e.what() << std::endl;
            return false;
        }

        return true;
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif