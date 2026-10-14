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

    // 运动补偿冻结在同一批末先验；保留原LiDAR点及旋转，计算外参的真实导数。
    struct DeskewedPoint {
        Eigen::Vector3d point_imu = Eigen::Vector3d::Zero();
        Eigen::Vector3d point_lidar = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rotation_to_end = Eigen::Matrix3d::Identity();
    };

    class Estimator {
    public:
        // 滤波状态及共享参数
        Parameters *parameters = nullptr;
        eskf kf;
        // 激光观测只查询前面完整扫描构成的历史地图，当前扫描后验点在scan_end入图。
        std::shared_ptr<VoxelMap> ivox;
        std::vector<Eigen::Vector3f> nearest_points;
        // 启动/重建使用的lidar→IMU外参；当前外参同时保存在30维滤波状态中。
        // extrinsic_est_en=false时对应状态与协方差冻结，true时由激光联合更新。
        Eigen::Vector3d Lidar_T_wrt_IMU = Eigen::Vector3d::Zero();
        Eigen::Matrix3d Lidar_R_wrt_IMU = Eigen::Matrix3d::Identity();
        // IMU 观测：原始测量与加速度单位缩放
        Eigen::Matrix<state::value_type, 3, 1> angular_velocity;
        Eigen::Matrix<state::value_type, 3, 1> linear_acceleration;
        double imu_acceleration_scale;

        Estimator();

        void reset();

        [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov() const;

        // 点必须用同一先验补偿到batch末端IMU系；返回成功联合更新的匹配点数。
        // 在线外参雅可比保留原点与补偿旋转，不能把补偿后的点视为原LiDAR点。
        size_t update_lidar_batch(const std::vector<DeskewedPoint> &points);

        void h_imu(const state &s, imu_measurement_result &measurement_result);

    private:
        void h_point(const state &s, const DeskewedPoint &point,
                     point_measurement_result &measurement_result);
    };

}// namespace ly_lio
