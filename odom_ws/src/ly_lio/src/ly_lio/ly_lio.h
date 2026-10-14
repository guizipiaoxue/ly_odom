/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025 Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt.
 */
#pragma once
#include "common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"

namespace ly_lio {
// 只读运行快照：计数来自核心实际处理，不能据此推断DDS网络丢包或定位精度。
struct LioDiagnostics {
    std::uint64_t scans_received = 0, scans_accepted = 0, scans_processed = 0;
    std::uint64_t scans_rejected = 0, scans_dropped = 0;
    size_t pending_scans = 0, pending_imus = 0;
    size_t init_imu_collected = 0, init_imu_required = 0;
    size_t init_map_scans_collected = 0, init_map_scans_required = 0;
    size_t map_voxels = 0;
    size_t last_input_points = 0, last_tracking_points = 0, last_matched_points = 0;
    double last_scan_start = -1.0, last_scan_end = -1.0;
    double latest_lidar_time = -1.0, latest_imu_time = -1.0;
    double last_processing_ms = 0.0, mean_processing_ms = 0.0, max_processing_ms = 0.0;
    bool imu_enabled = false, imu_calibrated = false, initialized = false;
    std::uint64_t resets = 0;
};

// 算法只接收带真实采样时间的原始激光点/IMU；ROS、TF和文件保存留在节点边界。
class LyLio {
    Parameters parameters;
    Preprocess preprocess;
    Estimator estimator;
    double time_current = 0.0;
    double calibration_time = -1.0;
    double last_odometry_time = -1.0;
    bool imu_calibrated = false;
    bool is_init = false;
    size_t initial_map_scans = 0;
    size_t initial_map_points = 0;
    LioDiagnostics diagnostic_state;
    std::uint64_t processing_epoch_scans = 0;
    double processing_epoch_total_ms = 0.0;
    std::function<void(double, const std::vector<Eigen::Vector3f> &)> pointcloud_callback;
    std::function<void(const common::Odometry &)> odometry_callback;

    Eigen::Vector3f point_to_world(const Eigen::Vector3f &point) const;
    bool stationary_initial_state(Eigen::Vector3d &mean_acc, Eigen::Vector3d &mean_gyro) const;
    bool initialize();
    void clear_tracking_diagnostics();
    void process_scan(const ScanFrame &scan);
    void publish_odometry(double timestamp);
public:
    Eigen::Matrix<double, state::DIM, state::DIM> Q;
    explicit LyLio(rclcpp::Node &node);
    void reset();
    void on_point_cloud_callback(const std::vector<common::Point> &points);
    void on_imu_callback(const common::ImuMsg &imu);
    void handle_once();
    void set_pointcloud_callback(std::function<void(double, const std::vector<Eigen::Vector3f> &)> callback);
    void set_odometry_callback(std::function<void(const common::Odometry &)> callback);
    bool initialized() const { return is_init; }
    LioDiagnostics diagnostics() const;
    const state &current_state() const { return estimator.kf.x; }
    const Eigen::Matrix3d &lidar_to_imu_rotation() const { return estimator.kf.x.offset_R_L_I; }
    const Eigen::Vector3d &lidar_to_imu_translation() const { return estimator.kf.x.offset_T_L_I; }
    bool estimates_extrinsics() const { return parameters.extrinsic_est_en; }
    const auto &covariance() const { return estimator.kf.P; }
};
}  // namespace ly_lio
