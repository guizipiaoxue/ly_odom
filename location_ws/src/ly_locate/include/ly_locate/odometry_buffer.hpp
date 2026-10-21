// Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
#pragma once
#include <map>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <set>
#include "ly_locate/common.hpp"

namespace ly_locate {
class OdometryUnavailable : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// 仅缓存轻量T_OB，不订阅TF或整帧点云；按纳秒精确关联，不插值/外推。
// 重放复用stamp且位姿冲突时拒绝关联，不能猜测旧odom原点下的位姿。
class OdometryBuffer {
public:
    void insert(const nav_msgs::msg::Odometry& m, const std::string& odom,
                const std::string& body) {
        if (m.header.frame_id != odom || m.child_frame_id != body) {
            throw std::invalid_argument("Odometry父/子坐标系不匹配");
        }
        const auto stamp = ns(m.header.stamp);
        const Pose value = pose(m.pose.pose);
        std::lock_guard lock(mutex_);
        const auto it = poses_.find(stamp);
        if (it != poses_.end() && !it->second.matrix().isApprox(value.matrix(), 1e-12)) {
            ambiguous_.insert(stamp);
        } else {
            poses_.emplace(stamp, value);
        }
        while (poses_.size() > 10000) {
            ambiguous_.erase(poses_.begin()->first);
            poses_.erase(poses_.begin());
        }
    }
    Pose lookup(int64_t stamp) const {
        std::lock_guard lock(mutex_);
        if (ambiguous_.count(stamp)) {
            throw OdometryUnavailable("同stamp位姿冲突，拒绝跨会话关联");
        }
        const auto it = poses_.find(stamp);
        if (it == poses_.end()) {
            throw OdometryUnavailable("尚无同stamp Odometry");
        }
        return it->second;
    }
private:
    mutable std::mutex mutex_;
    std::map<int64_t, Pose> poses_;
    std::set<int64_t> ambiguous_;
};
}  // namespace ly_locate
