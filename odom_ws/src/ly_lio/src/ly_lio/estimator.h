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

    // 保留原始测量和相对批末时间；补偿是批末状态的函数，不再视作已知常量。
    struct DeskewedPoint {
        Eigen::Vector3d point_lidar = Eigen::Vector3d::Zero();
        double time_offset = 0.0;  // t_point-t_end，批内非正。

        [[nodiscard]] Eigen::Vector3d point_at_end(const state &s) const {
            return exp<double>(s.omg * time_offset) * (s.offset_R_L_I * point_lidar + s.offset_T_L_I) +
                s.rotation.transpose() * s.velocity * time_offset;
        }

        [[nodiscard]] Eigen::Vector3d point_world(const state &s) const {
            // 与R*point_at_end+p等价；直接表达使姿态对平移补偿的抵消显式可见。
            return s.position + s.rotation * (exp<double>(s.omg * time_offset) *
                (s.offset_R_L_I * point_lidar + s.offset_T_L_I)) + s.velocity * time_offset;
        }

        [[nodiscard]] Eigen::Matrix<double, 1, 18> jacobian(
            const state &s, const Eigen::Vector3d &normal, bool estimate_extrinsics) const {
            const Eigen::Vector3d phi = s.omg * time_offset;
            const Eigen::Matrix3d D = exp<double>(phi);
            const Eigen::Vector3d u = s.offset_R_L_I * point_lidar + s.offset_T_L_I;
            const Eigen::Vector3d C = s.rotation.transpose() * normal;
            const Eigen::Vector3d normal_at_sample = D.transpose() * C;
            Eigen::Matrix<double, 1, 18> H = Eigen::Matrix<double, 1, 18>::Zero();
            H.segment<3>(state::position_index) = normal.transpose();
            // 不能用包含R^T*v*dt的完整补偿点叉乘C，否则多出虚假的姿态项。
            H.segment<3>(state::rotation_index) = (D * u).cross(C).transpose();
            H.segment<3>(state::velocity_index) = time_offset * normal.transpose();
            // Exp(phi+dphi)=Exp(phi)*Exp(J_r(phi)*dphi)，J_r(phi)=A_matrix(-phi)。
            H.segment<3>(state::omg_index) =
                time_offset * u.cross(normal_at_sample).transpose() * A_matrix<double>(-phi);
            if (estimate_extrinsics) {
                H.segment<3>(state::offset_R_L_I_index) =
                    point_lidar.cross(s.offset_R_L_I.transpose() * normal_at_sample).transpose();
                H.segment<3>(state::offset_T_L_I_index) = normal_at_sample.transpose();
            }
            return H;
        }
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

        // 点的相对时间以共同批末为零；一次先验线性化包含补偿对速度/角速度的导数。
        size_t update_lidar_batch(const std::vector<DeskewedPoint> &points);

        void h_imu(const state &s, imu_measurement_result &measurement_result);

    private:
        void h_point(const state &s, const DeskewedPoint &point,
                     point_measurement_result &measurement_result);
    };

}// namespace ly_lio
