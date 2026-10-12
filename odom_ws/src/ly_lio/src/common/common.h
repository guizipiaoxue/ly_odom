/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace ly_lio::common {

    struct Odometry {
        Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> twist_covariance = Eigen::Matrix<double, 6, 6>::Zero();
        double timestamp;                // 时间，单位秒
        Eigen::Vector3d position;        // 位置，单位米
        Eigen::Vector3d velocity;        // 世界坐标系速度，单位 m/s
        Eigen::Quaterniond orientation;  // 参考系到世界；算法回调为 IMU，ROS 输出另转换到 baselink。
        Eigen::Vector3d angular_velocity;// 参考系中角速度，单位 rad/s。
    };

    struct ImuMsg {
        double timestamp;                   // 时间，单位秒
        Eigen::Vector3d linear_acceleration;// 原始加速度单位，由 acc_norm 指定
        Eigen::Vector3d angular_velocity;   // 角速度，单位 rad/s
    };

    struct Point {
        double timestamp;        // 时间，单位秒
        Eigen::Vector3f position;// 位置，单位米
    };

}// namespace ly_lio::common
