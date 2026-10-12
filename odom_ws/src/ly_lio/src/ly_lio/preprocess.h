/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025 Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt.
 */
#pragma once
#include <cstdint>
#include "common/common.h"
#include "parameters.h"
#include "util/voxelgrid_sampling.h"

namespace ly_lio {
// 完整ROS扫描保留自己的结束事件；积压时逐扫描发布/入图，不能合成一帧。
struct ScanFrame {
    double timestamp_start = -1.0;
    double timestamp_end = -1.0;
    size_t input_points = 0;                // 适配器交给核心的点数，未做本层过滤/降采样。
    std::vector<common::Point> points;        // 真实代表点：用于激光观测与地图增量。
    std::vector<common::Point> dense_points;  // 距离内真实点：只预测去畸变并输出。
};

class Preprocess {
    double last_timestamp_lidar = -1.0;
    double last_timestamp_imu = -1.0;
    bool imu_discontinuity = false;
    std::uint64_t received_scans = 0;
    std::uint64_t accepted_scans = 0;
    std::uint64_t rejected_scans = 0;
    std::uint64_t dropped_scans = 0;
    util::VoxelgridSampling downsampler;
    void enforce_scan_limit();
public:
    Parameters *parameters = nullptr;
    std::deque<ScanFrame> scan_deque;
    std::deque<common::ImuMsg> imu_deque;
    double lidar_watermark() const { return last_timestamp_lidar; }
    double imu_watermark() const { return last_timestamp_imu; }
    std::uint64_t scans_received() const { return received_scans; }
    std::uint64_t scans_accepted() const { return accepted_scans; }
    std::uint64_t scans_rejected() const { return rejected_scans; }
    std::uint64_t scans_dropped() const { return dropped_scans; }
    void reset();
    void discard_scans();
    bool consume_imu_discontinuity() {
        const bool discontinuity = imu_discontinuity;
        imu_discontinuity = false;
        return discontinuity;
    }
    void on_point_cloud_callback(const std::vector<common::Point> &points);
    void on_imu_callback(const common::ImuMsg &imu);
};
}  // namespace ly_lio
