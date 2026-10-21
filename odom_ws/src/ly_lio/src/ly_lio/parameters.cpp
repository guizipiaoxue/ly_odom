/**
 * 衍生自 Small Point-LIO；文档Batch激光与IMU输出观测模型的参数。
 * Copyright (C) 2025 Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt.
 */

#include "parameters.h"

namespace ly_lio {
void Parameters::read_parameters(rclcpp::Node &node) {
    // 每项参数均有默认值；检查数组长度后才访问，避免配置错误造成越界。
    const double min_distance = node.declare_parameter<double>("min_distance", 0.5);
    const double max_distance = node.declare_parameter<double>("max_distance", 100.0);
    min_distance_squared = min_distance * min_distance;
    max_distance_squared = max_distance * max_distance;
    max_buffer_points = node.declare_parameter<int>("max_buffer_points", 2000000);
    max_buffer_imu = node.declare_parameter<int>("max_buffer_imu", 20000);

    const auto gravity_values = node.declare_parameter<std::vector<double>>("gravity", {0.0, 0.0, -9.81});
    const auto translation = node.declare_parameter<std::vector<double>>("extrinsic_T", {0.0, 0.0, 0.0});
    const auto rotation = node.declare_parameter<std::vector<double>>("extrinsic_R", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    if (gravity_values.size() != 3 || translation.size() != 3 || rotation.size() != 9) {
        throw std::invalid_argument("gravity/extrinsic_T/extrinsic_R 的长度必须分别为 3/3/9");
    }
    gravity = Eigen::Map<const Eigen::Vector3d>(gravity_values.data());
    extrinsic_T = Eigen::Map<const Eigen::Vector3d>(translation.data());
    extrinsic_R = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
    // 内部始终使用Point-LIO的p_I=R_IL*p_L+t_IL；位置向量在读取边界只换算一次。
    const auto convention = node.declare_parameter<std::string>("extrinsic_translation_convention", "imu_in_lidar");
    if (convention == "imu_in_lidar") {
        extrinsic_T = (-extrinsic_R * extrinsic_T).eval();
    } else if (convention != "lidar_in_imu") {
        throw std::invalid_argument("extrinsic_translation_convention必须为imu_in_lidar或lidar_in_imu");
    }
    // 24维物理状态不包含外参。旧false仅供配置迁移，不能静默接受在线标定要求。
    const auto &overrides = node.get_node_parameters_interface()->get_parameter_overrides();
    if (node.declare_parameter<bool>("extrinsic_est_en", false)) {
        throw std::invalid_argument("当前SO(3)×R^21为24维固定外参模型，不支持extrinsic_est_en=true");
    }
    if (overrides.count("extrinsic_est_en")) {
        RCLCPP_WARN(node.get_logger(), "extrinsic_est_en已停用；当前24维模型始终使用固定激光-IMU标定，请删除旧false配置");
    }
    if (overrides.count("extrinsic_rotation_cov") || overrides.count("extrinsic_translation_cov")) {
        throw std::invalid_argument("24维模型已移除外参先验，请删除extrinsic_rotation_cov/extrinsic_translation_cov");
    }
    imu_en = node.declare_parameter<bool>("imu_en", true);
    check_satu = node.declare_parameter<bool>("check_satu", true);
    fix_gravity_direction = node.declare_parameter<bool>("fix_gravity_direction", true);
    init_imu_samples = node.declare_parameter<int>("init_imu_samples", 200);
    init_max_gyro_mean = node.declare_parameter<double>("init_max_gyro_mean", 0.05);
    init_max_gyro_std = node.declare_parameter<double>("init_max_gyro_std", 0.02);
    init_max_accel_std = node.declare_parameter<double>("init_max_accel_std", 0.30);
    init_max_gravity_error = node.declare_parameter<double>("init_max_gravity_error", 0.75);
    // 与《建模.md》一致，达到配置量程的95%即屏蔽该轴；单位沿用输入。
    satu_acc = node.declare_parameter<double>("satu_acc", 3.0) * 0.95;
    satu_gyro = node.declare_parameter<double>("satu_gyro", 35.0) * 0.95;
    acc_norm = node.declare_parameter<double>("acc_norm", 1.0);

    imu_meas_acc_cov = node.declare_parameter<double>("imu_meas_acc_cov", 0.01);
    imu_meas_omg_cov = node.declare_parameter<double>("imu_meas_omg_cov", 0.01);
    // 旧参数的数值/单位不作隐式迁移，新增速度方差率须显式配置。
    const double legacy_velocity_cov = node.declare_parameter<double>("velocity_cov", 0.0);
    if (legacy_velocity_cov != 0.0) {
        RCLCPP_WARN(node.get_logger(), "velocity_cov已停用且不生效；直接速度过程噪声请显式设置velocity_process_noise_rate（m²/s³）");
    }
    acceleration_cov = node.declare_parameter<double>("acceleration_cov", 500.0);
    omg_cov = node.declare_parameter<double>("omg_cov", 1000.0);
    ba_cov = node.declare_parameter<double>("ba_cov", 0.0001);
    process_noise_reference_dt = node.declare_parameter<double>("process_noise_reference_dt", 0.005);
    velocity_process_noise_rate = node.declare_parameter<double>("velocity_process_noise_rate", 0.0);
    max_imu_gap = node.declare_parameter<double>("max_imu_gap", 0.1);
    bg_cov = node.declare_parameter<double>("bg_cov", 0.0001);
    lidar_batch_duration = node.declare_parameter<double>("lidar_batch_duration", 0.001);
    point_filter_num = node.declare_parameter<int>("point_filter_num", 1);
    space_downsample = node.declare_parameter<bool>("space_downsample", true);
    space_downsample_leaf_size = node.declare_parameter<double>("space_downsample_leaf_size", 0.2);
    map_resolution = node.declare_parameter<double>("map_resolution", 0.2);
    const int capacity = node.declare_parameter<int>("map_capacity", 1000000);
    const int per_voxel = node.declare_parameter<int>("map_max_points_per_voxel", 20);
    map_min_point_distance = node.declare_parameter<double>("map_min_point_distance", 0.05);
    const int initial_points = node.declare_parameter<int>("init_map_size", 100);
    const int initial_scans = node.declare_parameter<int>("init_map_scans", 5);
    laser_point_cov = node.declare_parameter<double>("laser_point_cov", 0.01);
    plane_threshold = node.declare_parameter<double>("plane_threshold", 0.1);
    match_sqaured = node.declare_parameter<double>("match_sqaured", 81.0);
    publish_odometry_per_batch = node.declare_parameter<bool>("publish_odometry_per_batch", false);
    odometry_publish_rate = node.declare_parameter<double>("odometry_publish_rate", 100.0);
    if (!std::isfinite(odometry_publish_rate) || odometry_publish_rate <= 0 ||
        !std::isfinite(1.0 / odometry_publish_rate)) {
        throw std::invalid_argument("odometry_publish_rate必须为有限正频率（Hz）");
    }
    if (node.declare_parameter<bool>("publish_odometry_without_downsample", false)) {
        RCLCPP_WARN(node.get_logger(), "publish_odometry_without_downsample已弃用；按publish_odometry_per_batch=true输出真实批末位姿");
        publish_odometry_per_batch = true;
    }
    if (capacity < 1 || per_voxel < 1 || initial_points < 5 || initial_scans < 1) {
        throw std::invalid_argument("地图容量、每格点数和初始地图帧数必须为正，初始有效点数至少5");
    }
    map_capacity = static_cast<size_t>(capacity);
    map_max_points_per_voxel = static_cast<size_t>(per_voxel);
    init_map_size = static_cast<size_t>(initial_points);
    init_map_scans = static_cast<size_t>(initial_scans);

    const auto positive = [](double value) { return std::isfinite(value) && value > 0; };
    const auto nonnegative = [](double value) { return std::isfinite(value) && value >= 0; };
    if (!nonnegative(velocity_process_noise_rate)) {
        throw std::invalid_argument("velocity_process_noise_rate必须为有限且非负的速度协方差增长率（m²/s³）");
    }
    if (positive(process_noise_reference_dt) &&
        !std::isfinite(velocity_process_noise_rate / process_noise_reference_dt)) {
        throw std::invalid_argument("velocity_process_noise_rate/process_noise_reference_dt必须有限");
    }
    // 配置上限严格小于10ms；实际批长还会被下一IMU采样和扫描末端提前截断。
    if (!positive(lidar_batch_duration) || lidar_batch_duration >= 0.010) {
        throw std::invalid_argument("lidar_batch_duration必须为有限秒数且满足0<duration<0.010（严格小于10ms）");
    }
    if (init_imu_samples < 1 || max_buffer_points < 1 ||
        max_buffer_imu < init_imu_samples || !nonnegative(min_distance) ||
        !positive(max_distance) || min_distance >= max_distance ||
        !positive(acc_norm) || !positive(satu_acc) || !positive(satu_gyro) ||
        !positive(init_max_gyro_mean) || !positive(init_max_gyro_std) ||
        !positive(init_max_accel_std) || !positive(init_max_gravity_error) ||
        !positive(imu_meas_acc_cov) || !positive(imu_meas_omg_cov) ||
        !nonnegative(acceleration_cov) || !nonnegative(omg_cov) ||
        !nonnegative(ba_cov) || !nonnegative(bg_cov) || !positive(process_noise_reference_dt) || !positive(max_imu_gap) ||
        point_filter_num < 1 || !positive(space_downsample_leaf_size) || !positive(map_resolution) ||
        !nonnegative(map_min_point_distance) || !positive(laser_point_cov) ||
        !positive(plane_threshold) || !positive(match_sqaured)) {
        throw std::invalid_argument("滤波、地图、缓冲区和噪声参数超出合法范围");
    }
    if (!gravity.allFinite() || !positive(gravity.norm()) || !extrinsic_T.allFinite() ||
        !extrinsic_R.allFinite() ||
        !(extrinsic_R * extrinsic_R.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-5) ||
        std::abs(extrinsic_R.determinant() - 1.0) > 1e-5) {
        throw std::invalid_argument("重力和外参必须有限，extrinsic_R 必须为合法旋转矩阵");
    }
}
}  // namespace ly_lio
