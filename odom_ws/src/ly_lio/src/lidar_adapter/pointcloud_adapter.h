/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */


#pragma once
#include "base_lidar.h"
#include "pointcloud_decoder.h"

namespace ly_lio {
class PointCloudAdapter : public LidarAdapterBase {
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
    PointCloudTime time_;
public:
    explicit PointCloudAdapter(PointCloudTime time) : time_(time) {}
    void setup_subscription(rclcpp::Node *node, const std::string &topic,
        std::function<void(const std::vector<common::Point> &)> callback) override {
        // 节点声明参数，这里只读取，避免多个适配器重复声明相同参数。
        time_.relative = node->get_parameter("timestamp_mode").as_string() == "relative";
        time_.scale = node->get_parameter("timestamp_scale").as_double();
        subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic,
            rclcpp::SensorDataQoS(), [this, node, callback](const sensor_msgs::msg::PointCloud2 &msg) {
                if (!msg.header.frame_id.empty() && msg.header.frame_id != node->get_parameter("lidar_frame").as_string()) {
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                        "点云坐标系 %s 与 lidar_frame 不一致，拒绝应用错误外参", msg.header.frame_id.c_str());
                    return;
                }
                try { callback(PointCloudDecoder::decode(msg, time_)); }
                catch (const std::exception &error) {
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                        "丢弃不合法的点云：%s", error.what());
                }
            });
    }
};
}  // namespace ly_lio
