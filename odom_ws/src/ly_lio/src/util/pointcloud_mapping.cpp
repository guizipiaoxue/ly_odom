/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#include "pointcloud_mapping.h"

namespace ly_lio::util {

    void PointcloudMappingGrid::add_point(const Eigen::Vector3f &point) {
        ++points_size;
        points_sum += point;
    }

    [[nodiscard]] Eigen::Vector3f PointcloudMappingGrid::get_point() const {
        return points_sum / points_size;
    }

    uint64_t PointcloudMapping::GridKeyTypeHasher::operator()(const GridKeyType &v) const {
        // 无符号混合，支持负体素坐标，也不把哈希值强制压缩至千万个桶。
        return (uint64_t(static_cast<uint32_t>(v[0])) * 73856093ULL) ^
               (uint64_t(static_cast<uint32_t>(v[1])) * 471943ULL) ^
               (uint64_t(static_cast<uint32_t>(v[2])) * 83492791ULL);
    }

    PointcloudMapping::PointcloudMapping(float resolution)
        : inv_resolution(1 / resolution) {
        if (!std::isfinite(resolution) || resolution <= 0) { throw std::invalid_argument("保存地图分辨率必须为正"); }
    }

    [[nodiscard]] PointcloudMapping::GridKeyType PointcloudMapping::get_position_index(const Eigen::Vector3f &point) const {
        return (point * inv_resolution).array().floor().cast<int>();
    }

    void PointcloudMapping::add_point(const Eigen::Vector3f &point) {
        if (!point.allFinite()) { return; }
        grids_map.try_emplace(get_position_index(point)).first->second.add_point(point);
    }

    void PointcloudMapping::add_pointcloud(const std::vector<Eigen::Vector3f> &pointcloud) {
        for (const auto &point: pointcloud) {
            add_point(point);
        }
    }

    [[nodiscard]] std::vector<Eigen::Vector3f> PointcloudMapping::get_points() const {
        std::vector<Eigen::Vector3f> result;
        result.reserve(grids_map.size());
        for (const auto &item: grids_map) {
            result.push_back(item.second.get_point());
        }
        return result;
    }

}// namespace ly_lio::util
