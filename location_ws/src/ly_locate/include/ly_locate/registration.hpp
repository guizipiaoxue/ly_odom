// Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
#pragma once
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <Eigen/Eigenvalues>

#include "ly_locate/common.hpp"

namespace ly_locate {
struct RegistrationSettings {
    double correspondence = 0.5;
    double inlier_distance = 0.20;
    double max_rmse = 0.10;
    double min_overlap = 0.70;
    double cycle_translation = 0.08;
    double cycle_rotation = 0.035;
    double max_translation_correction = 1.5;
    double max_rotation_correction = 0.35;
    double min_information_ratio = 0.002;
    int min_inliers = 150;
    int iterations = 60;
};

struct RegistrationResult {
    Pose transform = Pose::Identity();
    double rmse = 0;
    double overlap = 0;
};

// source 经 initial 变换到 target；输出始终是 target_T_source。
inline bool icp(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target, const Pose& initial,
                const RegistrationSettings& settings, Pose& result) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> solver;
    solver.setInputSource(source);
    solver.setInputTarget(target);
    solver.setMaxCorrespondenceDistance(settings.correspondence);
    solver.setMaximumIterations(settings.iterations);
    solver.setTransformationEpsilon(1e-8);
    solver.setEuclideanFitnessEpsilon(1e-8);
    Cloud aligned;
    solver.align(aligned, initial.matrix().cast<float>());
    if (!solver.hasConverged() || !solver.getFinalTransformation().allFinite()) {
        return false;
    }
    // 数值配准结果投影回 SO(3)，防止累计浮点误差产生非正交旋转。
    const auto t = solver.getFinalTransformation().cast<double>().eval();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(t.topLeftCorner<3, 3>(),
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d rotation_correction = Eigen::Matrix3d::Identity();
    rotation_correction(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    result = Pose::Identity();
    result.linear() = svd.matrixU() * rotation_correction * svd.matrixV().transpose();
    result.translation() = t.topRightCorner<3, 1>();
    return true;
}

struct Fit {
    double overlap = 0;
    double rmse = 1e9;
    size_t count = 0;
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
};

// 用实际最近邻重新统计内点与误差，不把 ICP 的收敛标志当作验收结果。
inline Fit evaluate(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target, const Pose& t,
                    double gate, bool information) {
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);
    Fit fit;
    double squared_error_sum = 0;
    std::vector<int> neighbor_indices(information ? 10 : 1);
    std::vector<float> distances(neighbor_indices.size());
    for (const auto& point : *source) {
        const Eigen::Vector3d transformed_point = t * point.getVector3fMap().cast<double>();
        pcl::PointXYZ query_point(transformed_point.x(), transformed_point.y(),
                                  transformed_point.z());
        const int neighbor_count =
            tree.nearestKSearch(query_point, neighbor_indices.size(), neighbor_indices, distances);
        if (neighbor_count == 0 || distances[0] > gate * gate) {
            continue;
        }
        ++fit.count;
        squared_error_sum += distances[0];
        if (information && neighbor_count == 10 && distances[9] < 1.0) {
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (int i : neighbor_indices) {
                mean += (*target)[i].getVector3fMap().cast<double>();
            }
            mean /= 10;
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (int i : neighbor_indices) {
                Eigen::Vector3d centered_point =
                    (*target)[i].getVector3fMap().cast<double>() - mean;
                covariance += centered_point * centered_point.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
            if (eigen_solver.info() != Eigen::Success || eigen_solver.eigenvalues()[1] < 1e-5 ||
                eigen_solver.eigenvalues()[0] / eigen_solver.eigenvalues()[1] > 0.2) {
                continue;
            }
            const Eigen::Vector3d normal = eigen_solver.eigenvectors().col(0);
            // 以场景长度归一化旋转列，检查几何秩；单墙/纯地面不能提供可靠6DoF回环。
            Eigen::Matrix<double, 6, 1> jacobian;
            jacobian.head<3>() = transformed_point.cross(normal) / 10.0;
            jacobian.tail<3>() = normal;
            fit.information += jacobian * jacobian.transpose();
        }
    }
    fit.overlap = source->empty() ? 0 : double(fit.count) / source->size();
    if (fit.count) {
        fit.rmse = std::sqrt(squared_error_sum / fit.count);
    }
    return fit;
}

inline bool verify_registration(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                                const Pose& initial, const RegistrationSettings& settings,
                                RegistrationResult& result, std::string& why) {
    // 双向一致性、捕获范围、重叠误差、几何可观测性必须全部通过。
    Pose forward;
    Pose reverse;
    if (!icp(source, target, initial, settings, forward) ||
        !icp(target, source, initial.inverse(), settings, reverse)) {
        why = "ICP未收敛";
        return false;
    }
    const Pose cycle = forward * reverse;
    if (cycle.translation().norm() > settings.cycle_translation ||
        angle(cycle) > settings.cycle_rotation) {
        why = "正反配准不一致";
        return false;
    }
    const Pose delta = initial.inverse() * forward;
    if (delta.translation().norm() > settings.max_translation_correction ||
        angle(delta) > settings.max_rotation_correction) {
        why = "超出保守捕获范围";
        return false;
    }
    const Fit forward_fit = evaluate(source, target, forward, settings.inlier_distance, true);
    const Fit reverse_fit = evaluate(target, source, reverse, settings.inlier_distance, false);
    if (std::min(forward_fit.count, reverse_fit.count) < size_t(settings.min_inliers) ||
        std::min(forward_fit.overlap, reverse_fit.overlap) < settings.min_overlap ||
        std::max(forward_fit.rmse, reverse_fit.rmse) > settings.max_rmse) {
        why = "双向重叠/内点/RMSE门限";
        return false;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(
        forward_fit.information);
    if (eigen_solver.info() != Eigen::Success || eigen_solver.eigenvalues()[0] < 1e-6 ||
        eigen_solver.eigenvalues()[0] / eigen_solver.eigenvalues()[5] <
            settings.min_information_ratio) {
        why = "场景6DoF几何退化";
        return false;
    }
    result = {forward, std::max(forward_fit.rmse, reverse_fit.rmse),
              std::min(forward_fit.overlap, reverse_fit.overlap)};
    return true;
}
}  // namespace ly_locate
