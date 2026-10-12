/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#pragma once

#ifdef HAVE_LIVOX_DRIVER

#include "base_lidar.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace ly_lio {

    class LivoxCustomMsgAdapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription;

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            subscription = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [node, callback](const livox_ros_driver2::msg::CustomMsg &msg) {
                        if (!msg.header.frame_id.empty() && msg.header.frame_id != node->get_parameter("lidar_frame").as_string()) {
                            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "自定义 Livox 点云坐标系与配置不一致");
                            return;
                        }
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(msg.points.size());
                        // 自定义消息的 timebase 已经是绝对纳秒时间。
                        for (const auto &point: msg.points) {
                            if ((point.tag & 0x30) <= 0x10 && (point.tag & 0x0f) == 0) {
                                common::Point new_point;
                                new_point.position << point.x, point.y, point.z;
                                new_point.timestamp = static_cast<double>(msg.timebase) * 1e-9 + point.offset_time * 1e-9;
                                if (new_point.position.allFinite()) { pointcloud.push_back(new_point); }
                            }
                        }
                        callback(pointcloud);
                    });
        }
    };

}// namespace ly_lio

#endif
