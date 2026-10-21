/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025 Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt.
 */
#include "ly_lio.h"
#include <chrono>

namespace ly_lio {
LyLio::LyLio(rclcpp::Node &node) {
    parameters.read_parameters(node);
    preprocess.parameters = &parameters;
    estimator.parameters = &parameters;
    estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T;
    estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R;
    estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;
    Q = estimator.process_noise_cov();
    RCLCPP_INFO(node.get_logger(),
        "直接速度过程噪声率 velocity_process_noise_rate=%.9g m²/s³（速度方差增量=rate*dt，参考周期=%.9g s）",
        parameters.velocity_process_noise_rate, parameters.process_noise_reference_dt);
    reset();
}
void LyLio::clear_tracking_diagnostics() {
    // 自动重建保留累计处理/拒绝/重建数，但不把上一轨迹的末帧和耗时显示成当前状态。
    diagnostic_state.last_input_points = diagnostic_state.last_tracking_points = diagnostic_state.last_matched_points = 0;
    diagnostic_state.last_scan_start = diagnostic_state.last_scan_end = -1.0;
    diagnostic_state.last_processing_ms = diagnostic_state.mean_processing_ms = diagnostic_state.max_processing_ms = 0.0;
    processing_epoch_scans = 0;
    processing_epoch_total_ms = 0.0;
}
void LyLio::reset() {
    preprocess.reset();
    estimator.reset();
    time_current = 0.0;
    calibration_time = last_odometry_time = -1.0;
    odometry_scan_end = next_odometry_time = -1.0;
    odometry_slots_to_end = 0.0;
    imu_calibrated = is_init = false;
    initial_map_scans = initial_map_points = 0;
    // 用户显式reset建立新的统计周期；构造时也走这一分支。
    diagnostic_state = LioDiagnostics{};
    clear_tracking_diagnostics();
}
LioDiagnostics LyLio::diagnostics() const {
    LioDiagnostics result = diagnostic_state;
    result.scans_received = preprocess.scans_received();
    result.scans_accepted = preprocess.scans_accepted();
    result.scans_rejected += preprocess.scans_rejected();
    result.scans_dropped = preprocess.scans_dropped();
    result.pending_scans = preprocess.scan_deque.size();
    result.pending_imus = preprocess.imu_deque.size();
    result.latest_lidar_time = preprocess.lidar_watermark();
    result.latest_imu_time = preprocess.imu_watermark();
    result.imu_enabled = parameters.imu_en;
    result.imu_calibrated = imu_calibrated;
    result.initialized = is_init;
    result.init_imu_required = parameters.imu_en && parameters.fix_gravity_direction ?
        static_cast<size_t>(parameters.init_imu_samples) : 0;
    // 标定后IMU队列继续被消费，进度保持完成，而不是显示当前队列长度倒退。
    result.init_imu_collected = imu_calibrated ? result.init_imu_required :
        std::min(preprocess.imu_deque.size(), result.init_imu_required);
    result.init_map_scans_collected = initial_map_scans;
    result.init_map_scans_required = parameters.init_map_scans;
    result.map_voxels = estimator.ivox->size();
    return result;
}
void LyLio::on_point_cloud_callback(const std::vector<common::Point> &points) { preprocess.on_point_cloud_callback(points); }
void LyLio::on_imu_callback(const common::ImuMsg &imu) {
    if (parameters.imu_en) { preprocess.on_imu_callback(imu); }
}
Eigen::Vector3f LyLio::point_to_world(const Eigen::Vector3f &point) const {
    const auto &x = estimator.kf.x;
    const Eigen::Matrix3d &E = estimator.Lidar_R_wrt_IMU;
    const Eigen::Vector3d &t = estimator.Lidar_T_wrt_IMU;
    return (x.rotation * (E * point.cast<double>() + t) + x.position).cast<float>();
}

bool LyLio::stationary_initial_state(Eigen::Vector3d &mean_acc, Eigen::Vector3d &mean_gyro) const {
    mean_acc = -parameters.gravity;
    mean_gyro.setZero();
    if (!parameters.imu_en || !parameters.fix_gravity_direction) { return true; }
    if (preprocess.imu_deque.size() < static_cast<size_t>(parameters.init_imu_samples)) { return false; }
    const auto first = preprocess.imu_deque.end() - parameters.init_imu_samples;
    mean_acc.setZero();
    for (auto it = first; it != preprocess.imu_deque.end(); ++it) {
        mean_acc += it->linear_acceleration * estimator.imu_acceleration_scale;
        mean_gyro += it->angular_velocity;
    }
    mean_acc /= parameters.init_imu_samples;
    mean_gyro /= parameters.init_imu_samples;
    Eigen::Vector3d variance_acc = Eigen::Vector3d::Zero(), variance_gyro = Eigen::Vector3d::Zero();
    for (auto it = first; it != preprocess.imu_deque.end(); ++it) {
        variance_acc += (it->linear_acceleration * estimator.imu_acceleration_scale - mean_acc).array().square().matrix();
        variance_gyro += (it->angular_velocity - mean_gyro).array().square().matrix();
    }
    const double std_acc = (variance_acc / parameters.init_imu_samples).cwiseSqrt().maxCoeff();
    const double std_gyro = (variance_gyro / parameters.init_imu_samples).cwiseSqrt().maxCoeff();
    // 门限在SI单位上判断；单姿态无法区分横向加速度零偏和倾角，不作持续零速锁定。
    return mean_acc.norm() >= 1e-6 && mean_gyro.norm() <= parameters.init_max_gyro_mean &&
        std_gyro <= parameters.init_max_gyro_std && std_acc <= parameters.init_max_accel_std &&
        std::abs(mean_acc.norm() - parameters.gravity.norm()) <= parameters.init_max_gravity_error;
}

bool LyLio::initialize() {
    if (preprocess.lidar_watermark() < 0 || (parameters.imu_en && preprocess.imu_deque.empty())) { return false; }
    Eigen::Vector3d mean_acc, mean_gyro;
    if (!stationary_initial_state(mean_acc, mean_gyro)) {
        if (imu_calibrated) {
            // 建初图期间发生运动：固定原点假设已失效，整批清空，重新等静止标定。
            estimator.reset();
            preprocess.discard_scans();
            initial_map_scans = initial_map_points = 0;
            imu_calibrated = false;
            ++diagnostic_state.resets;
            clear_tracking_diagnostics();
            RCLCPP_WARN(rclcpp::get_logger("ly_lio"), "初始地图采集期间检测到运动，清空初图并重新等待静止");
        }
        return false;
    }
    auto &x = estimator.kf.x;
    x.gravity = parameters.gravity;
    x.omg.setZero();
    if (parameters.imu_en && parameters.fix_gravity_direction) {
        x.gravity = -mean_acc.normalized() * parameters.gravity.norm();
        x.bg = mean_gyro;
        x.ba = mean_acc + x.gravity;
    }
    x.acceleration = -x.gravity;
    if (!imu_calibrated) {
        calibration_time = preprocess.lidar_watermark();
        if (parameters.imu_en) { calibration_time = std::max(calibration_time, preprocess.imu_watermark()); }
        // 标定前数据无对应运动轨迹，不能拿来建立零姿态地图。
        preprocess.discard_scans();
        imu_calibrated = true;
        RCLCPP_INFO(rclcpp::get_logger("ly_lio"), "静止IMU标定完成；丢弃旧点，继续静止收集%zu帧初始地图", parameters.init_map_scans);
        return false;
    }
    const double covered_time = parameters.imu_en ? preprocess.imu_watermark() : preprocess.lidar_watermark();
    while (!preprocess.scan_deque.empty() && preprocess.scan_deque.front().timestamp_end <= covered_time) {
        auto scan = std::move(preprocess.scan_deque.front());
        preprocess.scan_deque.pop_front();
        size_t valid = 0;
        for (const auto &point : scan.points) {
            if (point.timestamp < calibration_time) { continue; }
            ++valid;
            if (estimator.ivox->add_point(point_to_world(point.position))) { ++initial_map_points; }
        }
        if (valid > 0) { ++initial_map_scans; }
        time_current = scan.timestamp_end;
        if (initial_map_scans >= parameters.init_map_scans && initial_map_points >= parameters.init_map_size) {
            // 初图阶段只在持续静止假设下收点，不长时间纯惯性积分；追踪从末扫描开始。
            estimator.kf.init_timestamp(time_current);
            while (!preprocess.imu_deque.empty() && preprocess.imu_deque.front().timestamp <= time_current) {
                preprocess.imu_deque.pop_front();
            }
            is_init = true;
            RCLCPP_INFO(rclcpp::get_logger("ly_lio"), "初始地图完成：%zu帧、%zu个有间距真实点，开始24维Batch激光/IMU联合估计", initial_map_scans, initial_map_points);
            return true;
        }
    }
    return false;
}

void LyLio::process_scan(const ScanFrame &scan) {
    const auto processing_start = std::chrono::steady_clock::now();
    size_t tracking_points = 0, matched_points = 0;
    std::vector<Eigen::Vector3f> map_increment;
    std::vector<Eigen::Vector3f> registered;
    map_increment.reserve(scan.points.size());
    registered.reserve(scan.dense_points.size());
    size_t laser_index = 0, dense_index = 0;
    std::vector<DeskewedPoint> batch_points;
    std::vector<DeskewedPoint> batch_dense;
    // 一批内统一线性化。IMU事件之前先结束旧批，同stamp先IMU后激光。
    // 每个批末取实际点时间，不用人为网格给旧状态盖时间戳；地图整扫描冻结。
    // F在区间起点状态上线性化，先传播P再传播x，二者到达同一观测时刻。
    for (;;) {
        while (laser_index < scan.points.size() && scan.points[laser_index].timestamp < time_current) { ++laser_index; }
        while (dense_index < scan.dense_points.size() && scan.dense_points[dense_index].timestamp < time_current) { ++dense_index; }
        const double infinity = std::numeric_limits<double>::infinity();
        const double imu_time = parameters.imu_en && !preprocess.imu_deque.empty() ? preprocess.imu_deque.front().timestamp : infinity;
        const double laser_time = laser_index < scan.points.size() ? scan.points[laser_index].timestamp : infinity;
        const double dense_time = dense_index < scan.dense_points.size() ? scan.dense_points[dense_index].timestamp : infinity;
        const double first_point_time = std::min(laser_time, dense_time);
        if (imu_time <= first_point_time && imu_time <= scan.timestamp_end) {
            const auto imu = preprocess.imu_deque.front();
            preprocess.imu_deque.pop_front();
            if (imu.timestamp < time_current) { continue; }
            estimator.kf.predict_cov(imu.timestamp, Q, parameters.process_noise_reference_dt);
            estimator.kf.predict_state(imu.timestamp);
            estimator.angular_velocity = imu.angular_velocity;
            estimator.linear_acceleration = imu.linear_acceleration;
            estimator.kf.update_imu();
            time_current = imu.timestamp;
            continue;
        }
        if (first_point_time <= scan.timestamp_end) {
            const double limit = std::min(first_point_time + parameters.lidar_batch_duration, scan.timestamp_end);
            size_t laser_end = laser_index, dense_end = dense_index;
            double batch_end = first_point_time;
            const auto in_batch = [&](const common::Point &point) {
                return point.timestamp <= limit && point.timestamp < imu_time;
            };
            while (laser_end < scan.points.size() && in_batch(scan.points[laser_end])) {
                batch_end = std::max(batch_end, scan.points[laser_end++].timestamp);
            }
            while (dense_end < scan.dense_points.size() && in_batch(scan.dense_points[dense_end])) {
                batch_end = std::max(batch_end, scan.dense_points[dense_end++].timestamp);
            }
            estimator.kf.predict_cov(batch_end, Q, parameters.process_noise_reference_dt);
            estimator.kf.predict_state(batch_end);
            time_current = batch_end;
            // 式3.44–3.47：所有点先变至IMU原点，再补偿到批末IMU系。
            // v存于世界系，必须先用R_end^T转轴；exp(0)也合法，无需omega归一化。
            // 保留原点/时间，补偿及其H统一在批末先验线性化，后验按同一模型重算。
            const auto deskew = [&](const common::Point &point) -> DeskewedPoint {
                DeskewedPoint result;
                result.point_lidar = point.position.cast<double>();
                result.time_offset = point.timestamp - batch_end;
                return result;
            };
            batch_points.clear();
            batch_dense.clear();
            for (size_t i = laser_index; i < laser_end; ++i) { batch_points.push_back(deskew(scan.points[i])); }
            for (size_t i = dense_index; i < dense_end; ++i) { batch_dense.push_back(deskew(scan.dense_points[i])); }
            tracking_points += batch_points.size();
            matched_points += estimator.update_lidar_batch(batch_points);
            const auto &posterior = estimator.kf.x;
            for (const auto &point : batch_points) {
                map_increment.push_back(point.point_world(posterior, estimator.Lidar_R_wrt_IMU, estimator.Lidar_T_wrt_IMU).cast<float>());
            }
            for (const auto &point : batch_dense) {
                registered.push_back(point.point_world(posterior, estimator.Lidar_R_wrt_IMU, estimator.Lidar_T_wrt_IMU).cast<float>());
            }
            laser_index = laser_end;
            dense_index = dense_end;
            publish_odometry(batch_end, scan.timestamp_end);
            continue;
        }
        // 距离过滤后即便末段没有点，仍推进到原始有效末点，保证输出时间定义不变。
        estimator.kf.predict_cov(scan.timestamp_end, Q, parameters.process_noise_reference_dt);
        estimator.kf.predict_state(scan.timestamp_end);
        time_current = scan.timestamp_end;
        for (const auto &point : map_increment) { estimator.ivox->add_point(point); }
        publish_odometry(scan.timestamp_end, scan.timestamp_end);
        if (pointcloud_callback && !registered.empty()) { pointcloud_callback(scan.timestamp_end, registered); }
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - processing_start).count();
        ++diagnostic_state.scans_processed;
        ++processing_epoch_scans;
        diagnostic_state.last_input_points = scan.input_points;
        diagnostic_state.last_tracking_points = tracking_points;
        diagnostic_state.last_matched_points = matched_points;
        diagnostic_state.last_processing_ms = elapsed_ms;
        processing_epoch_total_ms += elapsed_ms;
        diagnostic_state.mean_processing_ms = processing_epoch_total_ms / static_cast<double>(processing_epoch_scans);
        diagnostic_state.max_processing_ms = std::max(diagnostic_state.max_processing_ms, elapsed_ms);
        diagnostic_state.last_scan_start = scan.timestamp_start;
        diagnostic_state.last_scan_end = scan.timestamp_end;
        return;
    }
}

void LyLio::handle_once() {
    if (parameters.imu_en && preprocess.consume_imu_discontinuity()) {
        // 输入缺段或未处理IMU被迫裁剪后，当前轨迹没有连续惯性覆盖；明确重启
        // 标定/初图，不用旧omega/a跨大间隔积分，更不伪造中间100Hz输出。
        estimator.reset();
        preprocess.discard_scans();
        time_current = 0.0;
        calibration_time = last_odometry_time = -1.0;
        odometry_scan_end = next_odometry_time = -1.0;
        odometry_slots_to_end = 0.0;
        imu_calibrated = is_init = false;
        initial_map_scans = initial_map_points = 0;
        ++diagnostic_state.resets;
        clear_tracking_diagnostics();
        RCLCPP_WARN(rclcpp::get_logger("ly_lio"), "输入IMU时序不连续，滤波器与匹配地图已重置");
    }
    if (!is_init) { initialize(); return; }
    // 等IMU覆盖当前扫描末点再处理完整扫描；绝不借用下一时刻IMU更新更早的点。
    const double watermark = parameters.imu_en ? std::min(preprocess.lidar_watermark(), preprocess.imu_watermark()) : preprocess.lidar_watermark();
    while (!preprocess.scan_deque.empty() && preprocess.scan_deque.front().timestamp_end <= watermark) {
        auto scan = std::move(preprocess.scan_deque.front());
        preprocess.scan_deque.pop_front();
        if (scan.timestamp_end < time_current) {
            ++diagnostic_state.scans_rejected;
            continue;
        }
        process_scan(scan);
    }
}
void LyLio::set_pointcloud_callback(std::function<void(double, const std::vector<Eigen::Vector3f> &)> callback) { pointcloud_callback = std::move(callback); }
void LyLio::set_odometry_callback(std::function<void(const common::Odometry &)> callback) { odometry_callback = std::move(callback); }
void LyLio::publish_odometry(double timestamp, double scan_end) {
    if (!odometry_callback || !std::isfinite(timestamp) || !std::isfinite(scan_end) ||
        timestamp > scan_end || timestamp <= last_odometry_time) { return; }
    if (!parameters.publish_odometry_per_batch) {
        const double interval = 1.0 / parameters.odometry_publish_rate;
        if (scan_end != odometry_scan_end) {
            odometry_scan_end = scan_end;
            // 扫描末占一个槽；向前划槽，取槽后的第一个真实批末，不插值/外推。
            odometry_slots_to_end = std::floor((scan_end - timestamp) / interval);
            next_odometry_time = scan_end - odometry_slots_to_end * interval;
        }
        if (timestamp != scan_end &&
            (timestamp < next_odometry_time || next_odometry_time >= scan_end)) { return; }
        odometry_slots_to_end = std::max(0.0, odometry_slots_to_end -
            (std::floor((timestamp - next_odometry_time) / interval) + 1.0));
        next_odometry_time = scan_end - odometry_slots_to_end * interval;
    }
    last_odometry_time = timestamp;
    common::Odometry odometry;
    const auto &x = estimator.kf.x;
    odometry.timestamp = timestamp;
    odometry.position = x.position;
    odometry.velocity = x.velocity;
    odometry.orientation = Eigen::Quaterniond(x.rotation).normalized();
    odometry.angular_velocity = x.omg;
    // 算法回调描述 IMU 原点：姿态误差为 IMU 右扰动，pose 旋转协方差用世界系表达。
    Eigen::Matrix<double, 6, 6> frame_change = Eigen::Matrix<double, 6, 6>::Identity();
    frame_change.block<3, 3>(3, 3) = x.rotation;
    odometry.pose_covariance = frame_change * estimator.kf.P.block<6, 6>(0, 0) * frame_change.transpose();
    // 此处为 IMU 原点的 twist 协方差；ROS 节点根据完整 P 重新计算 baselink 输出。
    Eigen::Matrix<double, 6, 6> twist_change = Eigen::Matrix<double, 6, 6>::Identity();
    twist_change.block<3, 3>(0, 0) = x.rotation.transpose();
    odometry.twist_covariance = twist_change * estimator.kf.P.block<6, 6>(state::velocity_index, state::velocity_index) * twist_change.transpose();
    odometry_callback(odometry);
}
}  // namespace ly_lio
