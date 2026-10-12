/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#include "estimator.h"

namespace ly_lio {

    Estimator::Estimator() {// NOLINT(cppcoreguidelines-pro-type-member-init)
        kf.init(
            [this](const state &s, point_measurement_result &result) { h_point(s, result); },
            [this](const state &s, imu_measurement_result &result) { h_imu(s, result); });
    }

    void Estimator::reset() {
        kf.x = state();  // 同时重置状态和协方差，保留外参由 LyLio::reset 恢复。
        ivox = std::make_shared<VoxelMap>(parameters->map_resolution, parameters->map_capacity,
            parameters->map_max_points_per_voxel, parameters->map_min_point_distance);
        nearest_points.clear();
        nearest_points.reserve(5);
        kf.P = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity() * 0.01;
        kf.P.block<3, 3>(state::gravity_index, state::gravity_index).diagonal().fill(0.0001);
        kf.P.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(0.001);
        kf.P.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(0.001);
    }

    void Estimator::h_point(const state &s, point_measurement_result &result) {
        result.valid = false;
        if (!ivox || !point_lidar_frame.allFinite() || !s.rotation.allFinite() || !s.position.allFinite()) { return; }
        const Eigen::Matrix3d &E = parameters->extrinsic_est_en ? s.offset_R_L_I : Lidar_R_wrt_IMU;
        const Eigen::Vector3d &t = parameters->extrinsic_est_en ? s.offset_T_L_I : Lidar_T_wrt_IMU;
        const Eigen::Vector3d point_imu = E * point_lidar_frame.cast<double>() + t;
        const Eigen::Vector3d point_world = s.rotation * point_imu + s.position;
        ivox->get_closest_point(point_world.cast<float>(), nearest_points, 5);
        if (nearest_points.size() != 5) { return; }

        // PCA平面允许通过坐标原点；先拒绝重合/共线近邻，不能把任意特征向量当法向。
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto &point : nearest_points) { centroid += point.cast<double>(); }
        centroid /= static_cast<double>(nearest_points.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto &point : nearest_points) {
            const Eigen::Vector3d centered = point.cast<double>() - centroid;
            covariance.noalias() += centered * centered.transpose();
        }
        covariance /= static_cast<double>(nearest_points.size());
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) { return; }
        const auto eigenvalues = solver.eigenvalues();
        if (eigenvalues[1] < 1e-6 || eigenvalues[1] < 1e-4 * eigenvalues[2] ||
            eigenvalues[0] > 0.1 * eigenvalues[1]) { return; }
        const Eigen::Vector3d normal = solver.eigenvectors().col(0);
        const double offset = -normal.dot(centroid);
        for (const auto &point : nearest_points) {
            if (std::abs(normal.dot(point.cast<double>()) + offset) > parameters->plane_threshold) { return; }
        }
        const double distance = normal.dot(point_world) + offset;
        // 沿用Point-LIO的距离相关门控：远处点允许稍大残差，拒绝明显错误关联。
        if (!std::isfinite(distance) || point_lidar_frame.norm() <= parameters->match_sqaured * distance * distance) { return; }
        const Eigen::Vector3d normal_imu = s.rotation.transpose() * normal;
        result.H.setZero();
        result.H.template segment<3>(state::position_index) = normal.transpose();
        result.H.template segment<3>(state::rotation_index) = point_imu.cross(normal_imu).transpose();
        if (parameters->extrinsic_est_en) {
            result.H.template segment<3>(state::offset_R_L_I_index) =
                point_lidar_frame.cast<double>().cross(E.transpose() * normal_imu).transpose();
            result.H.template segment<3>(state::offset_T_L_I_index) = normal_imu.transpose();
        }
        result.z = -distance;
        result.laser_point_cov = parameters->laser_point_cov;
        result.valid = true;
    }

    [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> Estimator::process_noise_cov() const {
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> cov = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Zero();
        cov.block<3, 3>(state::velocity_index, state::velocity_index).diagonal().fill(static_cast<state::value_type>(parameters->velocity_cov));
        cov.block<3, 3>(state::omg_index, state::omg_index).diagonal().fill(static_cast<state::value_type>(parameters->omg_cov));
        cov.block<3, 3>(state::acceleration_index, state::acceleration_index).diagonal().fill(static_cast<state::value_type>(parameters->acceleration_cov));
        cov.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(static_cast<state::value_type>(parameters->bg_cov));
        cov.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(static_cast<state::value_type>(parameters->ba_cov));
        return cov;
    }

    void Estimator::h_imu(const state &s, imu_measurement_result &measurement_result) {
        // 饱和轴的 H 和残差同时屏蔽，其他轴仍参与观测，而非丢弃整条 IMU。
        std::fill_n(measurement_result.satu_check, 6, false);
        measurement_result.z.segment<3>(0) = angular_velocity - s.omg - s.bg;
        measurement_result.z.segment<3>(3) = linear_acceleration * imu_acceleration_scale - s.acceleration - s.ba;
        measurement_result.imu_meas_omg_cov = static_cast<state::value_type>(parameters->imu_meas_omg_cov);
        measurement_result.imu_meas_acc_cov = static_cast<state::value_type>(parameters->imu_meas_acc_cov);
        if (parameters->check_satu) {
            for (int i = 0; i < 3; i++) {
                if (std::abs(angular_velocity(i)) >= parameters->satu_gyro) {
                    measurement_result.satu_check[i] = true;
                    measurement_result.z(i) = 0.0;
                }
                if (std::abs(linear_acceleration(i)) >= parameters->satu_acc) {
                    measurement_result.satu_check[i + 3] = true;
                    measurement_result.z(i + 3) = 0.0;
                }
            }
        }
    }

}// namespace ly_lio
