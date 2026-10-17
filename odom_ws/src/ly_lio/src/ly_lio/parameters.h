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


        // 固定lidar→IMU标定；读取时将YAML中IMU在激光系的位置换算为t_IL。
        // 内部始终满足p_I=extrinsic_R*p_L+extrinsic_T，不进入24维状态/P。
        Eigen::Vector3d extrinsic_T;
        Eigen::Matrix3d extrinsic_R;

        double imu_meas_acc_cov;
        double imu_meas_omg_cov;
        double omg_cov;
        double acceleration_cov;
        double bg_cov;
        double ba_cov;
        double process_noise_reference_dt = 0.005;
        // 直接速度协方差增长率，单位m²/s³；0保持旧配置，无额外点数上限。
        double velocity_process_noise_rate = 0.0;
        double max_imu_gap = 0.1;  // s，超限/队列截断后重新初始化，禁止跨缺段积分。

        // 文档Batch模型：保留真实点时间，补偿到批末后联合匹配历史地图。
        double lidar_batch_duration = 0.001;  // s = 1ms；严格<10ms，遇IMU采样提前结束。
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
        bool publish_odometry_per_batch = false;

        void read_parameters(rclcpp::Node &node);
    };

}// namespace ly_lio
