// Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
// 独立实现Scan Context的极坐标最大高度描述、ring key与循环列余弦距离。
// 方法来源：G. Kim and A. Kim, Scan Context, IROS 2018；未复制第三方源码。
#pragma once
#include "ly_locate/common.hpp"

namespace ly_locate {
struct ScanContext {
    static constexpr int rings = 20;
    static constexpr int sectors = 60;
    Eigen::Matrix<double, rings, sectors> data = Eigen::Matrix<double, rings, sectors>::Zero();
    Eigen::Matrix<double, rings, 1> key = Eigen::Matrix<double, rings, 1>::Zero();
    int occupied = 0;
    int occupied_sectors = 0;

    // 每个极坐标格只保留最大高度；空格保持零，避免空白区域制造虚假相似度。
    explicit ScanContext(const Cloud& cloud, double radius) {
        for (const auto& point : cloud) {
            const double range = std::hypot(point.x, point.y);
            if (range < 1.0 || range >= radius) {
                continue;
            }
            const int row = std::min(rings - 1, int(range / radius * rings));
            const double azimuth = std::atan2(point.y, point.x) + M_PI;
            const int column = std::min(sectors - 1, int(azimuth / (2 * M_PI) * sectors));
            // 固定正高度偏置；空bin仍为0。单帧高度不足时由信息量门限拒绝。
            data(row, column) = std::max(data(row, column), std::max(0.01, double(point.z) + 5.0));
        }
        // ring key 只用于召回候选，不能直接作为回环判据。
        key = data.rowwise().mean();
        occupied = (data.array() > 0).count();
        for (int j = 0; j < sectors; ++j) {
            if (data.col(j).squaredNorm() > 0) {
                ++occupied_sectors;
            }
        }
    }

    struct Match {
        double distance = 1.0;
        double yaw = 0.0;
        int shift = 0;
    };

    // 循环移动历史描述子的列，得到 current 到历史帧的候选 yaw。
    Match compare(const ScanContext& current) const {
        Match best;
        std::array<double, sectors> distances;
        distances.fill(1.0);
        for (int shift = 0; shift < sectors; ++shift) {
            double similarity_sum = 0;
            int count = 0;
            for (int j = 0; j < sectors; ++j) {
                const auto history_column = data.col((j + shift) % sectors);
                const auto current_column = current.data.col(j);
                const double norm_product = history_column.norm() * current_column.norm();
                if (norm_product > 1e-9) {
                    similarity_sum += history_column.dot(current_column) / norm_product;
                    ++count;
                }
            }
            if (count < sectors / 3) {
                continue;
            }
            const double distance = std::clamp(1.0 - similarity_sum / count, 0.0, 1.0);
            distances[shift] = distance;
            if (distance < best.distance) {
                best = {distance, shift * (2 * M_PI / sectors), shift};
            }
        }
        // 另一个相差至少30度的yaw同样可信则拒绝：轴对称场景不能靠ICP局部收敛消除歧义。
        for (int shift = 0; shift < sectors; ++shift) {
            const int delta = std::min((shift - best.shift + sectors) % sectors,
                                       (best.shift - shift + sectors) % sectors);
            if (delta >= 5 && distances[shift] - best.distance < 0.015) {
                best.distance = 1.0;
                break;
            }
        }
        return best;
    }
};
}  // namespace ly_locate
