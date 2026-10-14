/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */



#pragma once
#include "ly_lio/ly_lio.h"
#include "lidar_adapter/base_lidar.h"
#include "util/pointcloud_mapping.h"
#include "terminal_dashboard.h"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <mutex>

namespace ly_lio {
// 默认互斥回调组串行更新算法；文件服务单独分组，快照加锁后释放再进行磁盘写入。
class LyLioNode : public rclcpp::Node {
    std::unique_ptr<LyLio> lio_;
    std::unique_ptr<LidarAdapterBase> lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> sensor_tf_broadcaster_;
    rclcpp::CallbackGroup::SharedPtr save_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    std::unique_ptr<TerminalDashboard> terminal_;
    rclcpp::CallbackGroup::SharedPtr terminal_group_;
    rclcpp::TimerBase::SharedPtr terminal_timer_;
    std::unique_ptr<util::PointcloudMapping> saved_map_;
    std::mutex map_mutex_;
    nav_msgs::msg::Path path_;
    std::string odom_frame_, body_frame_, imu_frame_, lidar_frame_, map_path_;
    Eigen::Matrix3d R_BI_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_BI_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_BL_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_BL_ = Eigen::Vector3d::Zero();
    bool body_from_lidar_mount_ = false;
    bool publish_dynamic_sensor_tf_ = false;
    double last_tf_time_ = -1;
    double last_path_time_ = -1;
    double path_publish_interval_ = 0.1;
    double save_resolution_ = 0.02;
    bool publish_tf_, publish_path_;
    int path_max_poses_;
    void publish_odometry(const common::Odometry &odometry);
    void publish_cloud(double timestamp, const std::vector<Eigen::Vector3f> &points);
    void clear_output_history();
    void process_pending();
public:
    explicit LyLioNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
};
}  // namespace ly_lio
