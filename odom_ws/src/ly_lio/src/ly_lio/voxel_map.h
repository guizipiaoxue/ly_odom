/**
 * 衍生自 Small Point-LIO；保留其固定维度滤波和高效地图实现。
 * Copyright (C) 2025 Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt.
 */

#pragma once
#include <pch.h>

namespace ly_lio {
// 有符号坐标保留完整空间范围，修复原 uint16_t 键的负坐标转换及远处体素混叠。
struct VoxelKey {
    std::array<int32_t, 3> xyz{};
    bool operator==(const VoxelKey &) const = default;
};
struct VoxelKeyHash {
    uint64_t operator()(const VoxelKey &key) const {
        uint64_t hash = 0x9e3779b97f4a7c15ULL;
        for (const auto value : key.xyz) {
            uint64_t part = static_cast<uint32_t>(value);
            part = (part ^ (part >> 16)) * 0xbf58476d1ce4e5b9ULL;
            hash ^= part + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};

class VoxelMap {
    struct Entry { VoxelKey key; std::vector<Eigen::Vector3f> points; size_t next = 0; };
    std::list<Entry> cache_;
    ankerl::unordered_dense::map<VoxelKey, std::list<Entry>::iterator, VoxelKeyHash> cells_;
    double inv_resolution_;
    size_t capacity_;
    size_t max_points_;
    float min_distance_squared_;
    std::vector<std::pair<float, Eigen::Vector3f>> candidates_;

public:
    explicit VoxelMap(double resolution, size_t capacity, size_t max_points = 20, double min_distance = 0.05)
        : inv_resolution_(1.0 / resolution), capacity_(capacity), max_points_(max_points),
          min_distance_squared_(min_distance * min_distance) {
        if (!std::isfinite(resolution) || resolution <= 0 || capacity == 0 || max_points == 0 ||
            !std::isfinite(min_distance) || min_distance < 0) {
            throw std::invalid_argument("体素分辨率、容量和点数必须为正，最小点距必须非负");
        }
        candidates_.reserve(7 * max_points);
    }
    bool get_position_index(const Eigen::Vector3f &point, VoxelKey &key) const {
        if (!point.allFinite()) { return false; }
        for (int i = 0; i < 3; ++i) {
            const double coordinate = std::floor(static_cast<double>(point[i]) * inv_resolution_);
            // 留出一格余量，保证查询相邻体素时不发生整数溢出。
            if (coordinate <= std::numeric_limits<int32_t>::min() ||
                coordinate >= std::numeric_limits<int32_t>::max()) { return false; }
            key.xyz[i] = static_cast<int32_t>(coordinate);
        }
        return true;
    }
    size_t size() const { return cells_.size(); }

    bool add_point(const Eigen::Vector3f &point) {
        VoxelKey key;
        if (!get_position_index(point, key)) { return false; }
        auto iter = cells_.find(key);
        if (iter != cells_.end()) {
            // 同格保留多个真实样本，使斜墙不必依赖不存在的面邻格才能拟合平面。
            // 去除过近/重复点，避免五个相同回波造成退化；满格循环替换，内存有上限。
            cache_.splice(cache_.begin(), cache_, iter->second);
            auto &cell = *iter->second;
            for (const auto &previous : cell.points) {
                if ((previous - point).squaredNorm() <= min_distance_squared_) { return false; }
            }
            if (cell.points.size() < max_points_) { cell.points.push_back(point); }
            else {
                cell.points[cell.next] = point;
                cell.next = (cell.next + 1) % max_points_;
            }
            return true;
        }
        cache_.push_front({key, {point}, 0});
        cells_.emplace(key, cache_.begin());
        if (cells_.size() > capacity_) {
            cells_.erase(cache_.back().key);
            cache_.pop_back();
        }
        return true;
    }

    void get_closest_point(const Eigen::Vector3f &point, std::vector<Eigen::Vector3f> &nearest, size_t count = 5) {
        nearest.clear();
        VoxelKey center;
        if (count == 0 || !get_position_index(point, center)) { return; }
        candidates_.clear();
        // Small Point-LIO 的 7 邻域：当前格及六个面邻居；避免遍历 26 邻居。
        constexpr std::array<std::array<int, 3>, 7> offsets = {{
            {0,0,0}, {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}}};
        for (const auto &offset : offsets) {
            auto key = center;
            for (int axis = 0; axis < 3; ++axis) { key.xyz[axis] += offset[axis]; }
            const auto iter = cells_.find(key);
            if (iter != cells_.end()) {
                for (const auto &candidate : iter->second->points) {
                    candidates_.emplace_back((candidate - point).squaredNorm(), candidate);
                }
            }
        }
        const auto less = [](const auto &left, const auto &right) { return left.first < right.first; };
        if (candidates_.size() > count) {
            std::nth_element(candidates_.begin(), candidates_.begin() + count, candidates_.end(), less);
            candidates_.resize(count);
        }
        for (const auto &candidate : candidates_) { nearest.push_back(candidate.second); }
    }
};
}  // namespace ly_lio
