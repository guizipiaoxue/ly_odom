/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace ly_lio {

    class Parameters {
    public:
        // 空间过滤、初始化及队列上限；达到上限时丢弃最旧消息，防止断流后无限增长。
        int init_imu_samples = 200;
        // 静止初始化门限均使用 SI 单位，与输入是 g 还是 m/s² 无关。
        double init_max_gyro_mean = 0.05;
        double init_max_gyro_std = 0.02;
        double init_max_accel_std = 0.30;
        double init_max_gravity_error = 0.75;
        int max_buffer_points = 2000000;
        int max_buffer_imu = 20000;
        bool imu_en = true;
        float min_distance_squared;
        float max_distance_squared;

        Eigen::Vector3d gravity;
        bool check_satu;
        bool fix_gravity_direction;
        double satu_acc;
        double satu_gyro;
        double acc_norm;


        bool extrinsic_est_en;
        Eigen::Vector3d extrinsic_T;
        Eigen::Matrix3d extrinsic_R;

        double imu_meas_acc_cov;
        double imu_meas_omg_cov;
        double velocity_cov;
        double omg_cov;
        double acceleration_cov;
        double bg_cov;
        double ba_cov;
        double process_noise_reference_dt = 0.005;
        double max_imu_gap = 0.1;  // s，超限/队列截断后重新初始化，禁止跨缺段积分。

        // 单雷达Point-LIO：保留真实点与真实时间，标量点到历史地图平面更新。
        int point_filter_num = 1;
        bool space_downsample = true;
        double space_downsample_leaf_size = 0.2;
        double map_resolution = 0.2;
        size_t map_capacity = 1000000;
        size_t map_max_points_per_voxel = 20;
        double map_min_point_distance = 0.05;
        size_t init_map_size = 100;
        size_t init_map_scans = 5;
        double laser_point_cov = 0.01;
        double plane_threshold = 0.1;
        double match_sqaured = 81.0;
        bool publish_odometry_without_downsample = false;

        void read_parameters(rclcpp::Node &node);
    };

}// namespace ly_lio
