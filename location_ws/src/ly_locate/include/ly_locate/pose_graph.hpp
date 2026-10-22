// Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
#pragma once
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

namespace ly_locate {
using Pose = Eigen::Isometry3d;

struct GraphSettings {
    double odom_rotation_sigma = 0.02;
    double odom_translation_sigma = 0.10;
    double loop_rotation_sigma = 0.03;
    double loop_translation_sigma = 0.08;
    double max_odom_error_translation = 0.5;
    double max_odom_error_rotation = 0.15;
    double max_loop_error_translation = 0.15;
    double max_loop_error_rotation = 0.08;
};

// 隔离GTSAM和PCL模板，避免同一翻译单元引入第三方内部Eigen头。
class PoseGraph {
public:
    // 固定T_MO仅改变全局坐标；原始T_OB和相邻/回环相对因子保持原定义。
    PoseGraph(GraphSettings settings, Pose odom_to_map);
    ~PoseGraph();
    void append(const Pose& raw);
    // from_T_to 把 to 帧的局部点变换到 from 帧；拒绝时保留原图。
    bool try_loop(size_t from, size_t to, const Pose& from_T_to, std::string& reason);
    std::vector<Pose> poses() const;
    void save(const std::string& path) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace ly_locate
