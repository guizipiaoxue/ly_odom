/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#pragma once

#include "eskf.h"
#include "parameters.h"
#include "voxel_map.h"
#include <pch.h>

namespace ly_lio {

    class Estimator {
    public:
        // 滤波状态及共享参数
        Parameters *parameters = nullptr;
        eskf kf;
        // 激光观测只查询前面完整扫描构成的历史地图，当前扫描后验点在scan_end入图。
        std::shared_ptr<VoxelMap> ivox;
        Eigen::Vector3f point_lidar_frame = Eigen::Vector3f::Zero();
        std::vector<Eigen::Vector3f> nearest_points;
        // lidar→IMU外参：固定模式从参数取值，在线模式使用30维状态中的外参。
        Eigen::Vector3d Lidar_T_wrt_IMU = Eigen::Vector3d::Zero();
        Eigen::Matrix3d Lidar_R_wrt_IMU = Eigen::Matrix3d::Identity();
        // IMU 观测：原始测量与加速度单位缩放
        Eigen::Matrix<state::value_type, 3, 1> angular_velocity;
        Eigen::Matrix<state::value_type, 3, 1> linear_acceleration;
        double imu_acceleration_scale;

        Estimator();

        void reset();

        [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov() const;

        void h_point(const state &s, point_measurement_result &measurement_result);

        void h_imu(const state &s, imu_measurement_result &measurement_result);
    };

}// namespace ly_lio
