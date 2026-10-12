/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025 Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt.
 */
#include "preprocess.h"

namespace ly_lio {
void Preprocess::reset() {
    scan_deque.clear();
    imu_deque.clear();
    last_timestamp_lidar = last_timestamp_imu = -1.0;
    imu_discontinuity = false;
    received_scans = accepted_scans = rejected_scans = dropped_scans = 0;
}
void Preprocess::discard_scans() { scan_deque.clear(); }

void Preprocess::enforce_scan_limit() {
    size_t buffered = 0;
    for (const auto &scan : scan_deque) {
        buffered += std::max<size_t>(1, scan.points.size() + scan.dense_points.size());
    }
    bool dropped = false;
    while (buffered > static_cast<size_t>(parameters->max_buffer_points) && !scan_deque.empty()) {
        const auto &oldest = scan_deque.front();
        buffered -= std::max<size_t>(1, oldest.points.size() + oldest.dense_points.size());
        scan_deque.pop_front();
        ++dropped_scans;
        dropped = true;
    }
    if (dropped) {
        RCLCPP_WARN(rclcpp::get_logger("ly_lio"), "扫描缓冲超限，丢弃完整旧扫描；检查IMU断流和处理速度");
    }
}

void Preprocess::on_point_cloud_callback(const std::vector<common::Point> &input) {
    ++received_scans;
    std::vector<common::Point> ordered;
    ordered.reserve(input.size());
    for (const auto &point : input) {
        if (point.position.allFinite() && std::isfinite(point.timestamp) && point.timestamp >= last_timestamp_lidar) {
            ordered.push_back(point);
        }
    }
    if (ordered.empty()) {
        ++rejected_scans;
        if (!input.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("ly_lio"), "丢弃非有限或时间完全回退的点云；循环回放需reset");
        }
        return;
    }
    std::stable_sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; });
    ScanFrame scan;
    scan.input_points = input.size();
    scan.timestamp_start = ordered.front().timestamp;
    // 结束时间来自最新有效输入点，距离过滤不能把这一扫描的时间水位提前。
    scan.timestamp_end = ordered.back().timestamp;
    last_timestamp_lidar = scan.timestamp_end;
    std::vector<common::Point> filtered;
    filtered.reserve(ordered.size());
    scan.dense_points.reserve(ordered.size());
    for (size_t i = 0; i < ordered.size(); ++i) {
        const auto &point = ordered[i];
        const double range_squared = point.position.cast<double>().squaredNorm();
        if (range_squared < parameters->min_distance_squared || range_squared > parameters->max_distance_squared) { continue; }
        scan.dense_points.push_back(point);
        if (i % static_cast<size_t>(parameters->point_filter_num) == 0) { filtered.push_back(point); }
    }
    if (parameters->space_downsample) {
        // util按体素保留距中心最近的原始坐标/时间对，不能对时变激光点做均值。
        downsampler.voxelgrid_sampling_omp(filtered, scan.points, parameters->space_downsample_leaf_size);
    } else {
        scan.points = std::move(filtered);
    }
    std::stable_sort(scan.points.begin(), scan.points.end(), [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; });
    scan_deque.push_back(std::move(scan));
    // 接受数指成功入队；之后的队列溢出或过期丢弃不撤销这次接受记录。
    ++accepted_scans;
    enforce_scan_limit();
}

void Preprocess::on_imu_callback(const common::ImuMsg &imu) {
    if (!std::isfinite(imu.timestamp) || !imu.angular_velocity.allFinite() || !imu.linear_acceleration.allFinite()) { return; }
    if (imu.timestamp <= last_timestamp_imu) {
        RCLCPP_WARN(rclcpp::get_logger("ly_lio"), "丢弃重复或时间回退的IMU；循环回放需reset");
        return;
    }
    if (last_timestamp_imu >= 0 && imu.timestamp - last_timestamp_imu > parameters->max_imu_gap) {
        // max时间水位不代表中间有测量覆盖；大缺口不能沿用旧角速度/比力跨越。
        imu_deque.clear();
        imu_discontinuity = true;
        RCLCPP_WARN(rclcpp::get_logger("ly_lio"), "IMU缺段 %.3fs 超过 %.3fs，要求重新静止初始化",
            imu.timestamp - last_timestamp_imu, parameters->max_imu_gap);
    }
    if (imu_deque.size() >= static_cast<size_t>(parameters->max_buffer_imu)) {
        // 不静默裁掉未处理历史。保留新样本作为重新标定起点，并使算法清理旧扫描。
        imu_deque.clear();
        imu_discontinuity = true;
        RCLCPP_WARN(rclcpp::get_logger("ly_lio"), "IMU缓冲超限，放弃旧时序并要求重新静止初始化");
    }
    imu_deque.push_back(imu);
    last_timestamp_imu = imu.timestamp;
}
}  // namespace ly_lio
