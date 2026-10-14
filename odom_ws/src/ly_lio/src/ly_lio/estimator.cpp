/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#include "estimator.h"

namespace ly_lio {

    Estimator::Estimator() {// NOLINT(cppcoreguidelines-pro-type-member-init)
        kf.init([this](const state &s, imu_measurement_result &result) { h_imu(s, result); });
    }

    void Estimator::reset() {
        kf.x = state();
        // reset须恢复配置外参，不能把上一段在线标定结果当新odom的隐式先验。
        kf.x.offset_R_L_I = Lidar_R_wrt_IMU;
        kf.x.offset_T_L_I = Lidar_T_wrt_IMU;
        ivox = std::make_shared<VoxelMap>(parameters->map_resolution, parameters->map_capacity,
            parameters->map_max_points_per_voxel, parameters->map_min_point_distance);
        nearest_points.clear();
        nearest_points.reserve(5);
        kf.P = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity() * 0.01;
        kf.P.block<3, 3>(state::gravity_index, state::gravity_index).diagonal().fill(0.0001);
        kf.P.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(0.001);
        kf.P.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(0.001);
        if (parameters->extrinsic_est_en) {
            kf.P.block<3, 3>(state::offset_R_L_I_index, state::offset_R_L_I_index)
                .diagonal().fill(parameters->extrinsic_rotation_cov);
            kf.P.block<3, 3>(state::offset_T_L_I_index, state::offset_T_L_I_index)
                .diagonal().fill(parameters->extrinsic_translation_cov);
        } else {
            // 固定外参仍占30维原状态的位置，但无不确定度/交叉项，不会被观测注入。
            kf.P.middleRows<6>(state::offset_R_L_I_index).setZero();
            kf.P.middleCols<6>(state::offset_R_L_I_index).setZero();
        }
    }

    void Estimator::h_point(const state &s, const DeskewedPoint &point, point_measurement_result &result) {
        result.valid = false;
        const auto &point_imu = point.point_imu;
        if (!ivox || !point_imu.allFinite() || !s.rotation.allFinite() || !s.position.allFinite()) { return; }
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
        if (!std::isfinite(distance) || point_imu.norm() <= parameters->match_sqaured * distance * distance) { return; }
        const Eigen::Vector3d normal_imu = s.rotation.transpose() * normal;
        result.H.setZero();
        result.H.template segment<3>(state::position_index) = normal.transpose();
        result.H.template segment<3>(state::rotation_index) = point_imu.cross(normal_imu).transpose();
        if (parameters->extrinsic_est_en) {
            if (!s.offset_R_L_I.allFinite() || !s.offset_T_L_I.allFinite() ||
                !point.point_lidar.allFinite() || !point.rotation_to_end.allFinite()) { return; }
            // q_I_end=D(R_IL p_L+t_IL)+b；运动补偿D/b冻结在本批先验。
            // R_IL采用右扰动：H_RIL=p_L×(R_IL^T D^T R_WI^T n)，H_tIL=D^T R_WI^T n。
            // D=I时退化为原Point-LIO外参观测；非零采样时间差不能遗漏D的作用。
            const Eigen::Vector3d normal_at_sample = point.rotation_to_end.transpose() * normal_imu;
            result.H.template segment<3>(state::offset_R_L_I_index) =
                point.point_lidar.cross(s.offset_R_L_I.transpose() * normal_at_sample).transpose();
            result.H.template segment<3>(state::offset_T_L_I_index) = normal_at_sample.transpose();
        }
        result.z = -distance;
        // 点在IMU系的3D各向同性协方差Σ=laser_point_cov*I。投影到标量点到面残差：
        // D_noise=-nᵀR，R_eff=D_noiseΣD_noiseᵀ；刚性去畸变旋转不改变各向同性点噪声。
        const Eigen::RowVector3d noise_projection = -normal.transpose() * s.rotation;
        result.variance = parameters->laser_point_cov * noise_projection.squaredNorm();
        result.valid = std::isfinite(result.variance) && result.variance > 0;
    }

    size_t Estimator::update_lidar_batch(const std::vector<DeskewedPoint> &points) {
        batch_measurement_result batch;
        batch.estimate_extrinsics = parameters->extrinsic_est_en;
        // 冻结一次先验。地图在扫描结束才写入，batch内部没有逐点状态注入或自匹配。
        const state prior = kf.x;
        for (const auto &point : points) {
            point_measurement_result measurement;
            h_point(prior, point, measurement);
            if (!measurement.valid || !measurement.H.allFinite() || !std::isfinite(measurement.z)) { continue; }
            const double weight = 1.0 / measurement.variance;
            batch.information.noalias() += weight * measurement.H.transpose() * measurement.H;
            batch.information_vector.noalias() += weight * measurement.H.transpose() * measurement.z;
            ++batch.matched_points;
        }
        return kf.update_lidar_batch(batch) ? batch.matched_points : 0;
    }

    [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> Estimator::process_noise_cov() const {
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> cov = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Zero();
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
