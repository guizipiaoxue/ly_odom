/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#include "voxelgrid_sampling.h"

/**
 * copy from https://github.com/koide3/small_gicp, modified to fit our need.
 * small_gicp is open source under the MIT license: https://github.com/koide3/small_gicp/blob/master/LICENSE
 */
namespace ly_lio::util {

    /// @brief Fast floor (https://stackoverflow.com/questions/824118/why-is-floor-so-slow).
    /// @param pt  Float vector
    /// @return    Floored int vector
    inline Eigen::Array3i fast_floor(const Eigen::Array3f &pt) {
        const Eigen::Array3i ncoord = pt.cast<int>();
        return ncoord - (pt < ncoord.cast<float>()).cast<int>();
    };

    /// @brief Implementation of quick sort with OpenMP parallelism. Do not call this directly. Use quick_sort_omp instead.
    /// @param first  First iterator
    /// @param last   Last iterator
    /// @param comp   Comparison function
    template<typename RandomAccessIterator, typename Compare>
    void quick_sort_omp_impl(RandomAccessIterator first, RandomAccessIterator last, const Compare &comp) {
        const std::ptrdiff_t n = std::distance(first, last);
        if (n < 1024) {
            std::sort(first, last, comp);
            return;
        }

        const auto median3 = [&](const auto &a, const auto &b, const auto &c, const Compare &comp) {
            return comp(a, b) ? (comp(b, c) ? b : (comp(a, c) ? c : a)) : (comp(a, c) ? a : (comp(b, c) ? c : b));
        };

        const int offset = n / 8;
        const auto m1 = median3(*first, *(first + offset), *(first + offset * 2), comp);
        const auto m2 = median3(*(first + offset * 3), *(first + offset * 4), *(first + offset * 5), comp);
        const auto m3 = median3(*(first + offset * 6), *(first + offset * 7), *(last - 1), comp);

        auto pivot = median3(m1, m2, m3, comp);
        auto middle1 = std::partition(first, last, [&](const auto &val) { return comp(val, pivot); });
        auto middle2 = std::partition(middle1, last, [&](const auto &val) { return !comp(pivot, val); });

#pragma omp task
        quick_sort_omp_impl(first, middle1, comp);

#pragma omp task
        quick_sort_omp_impl(middle2, last, comp);
    }

    /// @brief Quick sort with OpenMP parallelism.
    /// @param first        First iterator
    /// @param last         Last iterator
    /// @param comp         Comparison function
    /// @param num_threads  Number of threads
    template<typename RandomAccessIterator, typename Compare>
    void quick_sort_omp(RandomAccessIterator first, RandomAccessIterator last, const Compare &comp, int num_threads) {
#ifndef _MSC_VER
#pragma omp parallel num_threads(num_threads)
        {
#pragma omp single nowait
            { quick_sort_omp_impl(first, last, comp); }
        }
#else
        std::sort(first, last, comp);
#endif
    }


    // 21 位/轴的紧凑体素键：先检查范围再转换，不能让 NaN/超范围浮点转为整数。
    static uint64_t voxel_key(const Eigen::Vector3f &point, double inverse_leaf) {
        if (!point.allFinite()) { return std::numeric_limits<uint64_t>::max(); }
        constexpr int64_t offset = 1 << 20;
        uint64_t key = 0;
        for (int axis = 0; axis < 3; ++axis) {
            const double value = std::floor(static_cast<double>(point[axis]) * inverse_leaf);
            if (value < -offset || value >= offset) { return std::numeric_limits<uint64_t>::max(); }
            key |= static_cast<uint64_t>(static_cast<int64_t>(value) + offset) << (axis * 21);
        }
        return key;
    }

    void VoxelgridSampling::voxelgrid_sampling_omp(const std::vector<common::Point> &points,
        std::vector<common::Point> &downsampled, double leaf_size, int num_threads) {
        downsampled.clear();
        if (!std::isfinite(leaf_size) || leaf_size <= 0 || num_threads < 1) {
            throw std::invalid_argument("降采样体素大小和线程数必须为正");
        }
        if (points.empty()) { return; }
        coord_pt.resize(points.size());
        const double inverse_leaf = 1.0 / leaf_size;
        const bool parallel = points.size() >= 1024 && num_threads > 1;
#pragma omp parallel for if(parallel) num_threads(num_threads) schedule(static)
        for (size_t i = 0; i < points.size(); ++i) {
            const auto key = std::isfinite(points[i].timestamp) ?
                voxel_key(points[i].position, inverse_leaf) : std::numeric_limits<uint64_t>::max();
            coord_pt[i] = {key, i};
        }
        // 相同键按原序号排序，保证不同线程数产生相同结果。
        const auto compare = [](const auto &a, const auto &b) { return a < b; };
        if (parallel) {
            quick_sort_omp(coord_pt.begin(), coord_pt.end(), compare, num_threads);
        } else {
            std::sort(coord_pt.begin(), coord_pt.end(), compare);
        }
        for (size_t begin = 0; begin < coord_pt.size();) {
            if (coord_pt[begin].first == std::numeric_limits<uint64_t>::max()) { break; }
            size_t end = begin + 1;
            while (end < coord_pt.size() && coord_pt[end].first == coord_pt[begin].first) { ++end; }
            // 时变点云必须保留真实测量的 XYZ 与采样时间对。
            // 体素内不同时间的坐标平均后并不对应平均时间的激光测量，转动时会生成假点。
            const auto &first_point = points[coord_pt[begin].second];
            const Eigen::Vector3d center =
                ((first_point.position.cast<double>() * inverse_leaf).array().floor() + 0.5).matrix() * leaf_size;
            size_t representative = coord_pt[begin].second;
            double best_distance = std::numeric_limits<double>::infinity();
            for (size_t index = begin; index < end; ++index) {
                const auto &point = points[coord_pt[index].second];
                const double distance = (point.position.cast<double>() - center).squaredNorm();
                if (distance < best_distance) {
                    best_distance = distance;
                    representative = coord_pt[index].second;
                }
            }
            downsampled.push_back(points[representative]);
            begin = end;
        }
    }

    void VoxelgridSampling::voxelgrid_sampling(const std::vector<common::Point> &points,
        std::vector<common::Point> &output, double leaf) {
        voxelgrid_sampling_omp(points, output, leaf, 1);
    }
    void VoxelgridSampling::voxelgrid_sampling_omp(const std::vector<Eigen::Vector3f> &points,
        std::vector<Eigen::Vector3f> &output, double leaf, int threads) {
        std::vector<common::Point> timed;
        timed.reserve(points.size());
        for (const auto &point : points) { timed.push_back({0.0, point}); }
        std::vector<common::Point> reduced;
        voxelgrid_sampling_omp(timed, reduced, leaf, threads);
        output.clear();
        output.reserve(reduced.size());
        for (const auto &point : reduced) { output.push_back(point.position); }
    }
    void VoxelgridSampling::voxelgrid_sampling(const std::vector<Eigen::Vector3f> &points,
        std::vector<Eigen::Vector3f> &output, double leaf) {
        voxelgrid_sampling_omp(points, output, leaf, 1);
    }
}  // namespace ly_lio::util
