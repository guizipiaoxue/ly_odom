// Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
#include "ly_locate/pose_graph.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iomanip>

namespace ly_locate {
namespace {
gtsam::Pose3 to_gtsam_pose(const Pose& p) {
    return gtsam::Pose3(p.matrix());
}

Pose from_gtsam_pose(const gtsam::Pose3& p) {
    Pose t = Pose::Identity();
    t.matrix() = p.matrix();
    return t;
}

gtsam::SharedNoiseModel diagonal_noise(double r, double t) {
    // GTSAM Pose3 噪声顺序为旋转三维在前、平移三维在后。
    gtsam::Vector6 s;
    s << r, r, r, t, t, t;
    return gtsam::noiseModel::Diagonal::Sigmas(s);
}
}  // namespace

struct PoseGraph::Impl {
    GraphSettings settings;
    Pose odom_to_map;
    gtsam::ISAM2 isam;
    std::vector<Pose> raw;

    struct Edge {
        size_t a;
        size_t b;
        Pose z;
        bool loop;
    };

    std::vector<Edge> edges;

    Impl(GraphSettings s, Pose transform)
        : settings(s), odom_to_map(std::move(transform)), isam([] {
              gtsam::ISAM2Params p;
              p.relinearizeThreshold = 0.01;
              p.relinearizeSkip = 1;
              return p;
          }()) {
    }
};

PoseGraph::PoseGraph(GraphSettings s, Pose odom_to_map)
    : impl_(std::make_unique<Impl>(s, std::move(odom_to_map))) {
}

PoseGraph::~PoseGraph() = default;

void PoseGraph::append(const Pose& raw) {
    auto& d = *impl_;
    const size_t i = d.raw.size();
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    Pose initial = d.odom_to_map * raw;
    if (i == 0) {
        factors.add(
            gtsam::PriorFactor<gtsam::Pose3>(0, to_gtsam_pose(initial), diagonal_noise(1e-4, 1e-4)));
    } else {
        const Pose z = d.raw.back().inverse() * raw;
        factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
            i - 1, i, to_gtsam_pose(z),
            diagonal_noise(d.settings.odom_rotation_sigma, d.settings.odom_translation_sigma)));
        initial = from_gtsam_pose(d.isam.calculateEstimate<gtsam::Pose3>(i - 1)) * z;
    }
    values.insert(i, to_gtsam_pose(initial));
    // 在副本更新，异常不污染已提交图。
    gtsam::ISAM2 trial = d.isam;
    trial.update(factors, values);
    trial.update();
    if (!trial.calculateEstimate<gtsam::Pose3>(i).matrix().allFinite()) {
        throw std::runtime_error("GTSAM非有限里程计更新");
    }
    if (i) {
        d.edges.push_back({i - 1, i, d.raw.back().inverse() * raw, false});
    }
    d.raw.push_back(raw);
    d.isam = std::move(trial);
}

bool PoseGraph::try_loop(size_t from, size_t to, const Pose& z, std::string& reason) {
    auto& d = *impl_;
    if (from >= to || to >= d.raw.size()) {
        reason = "非法回环索引";
        return false;
    }
    gtsam::NonlinearFactorGraph factors;
    const auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
        diagonal_noise(d.settings.loop_rotation_sigma, d.settings.loop_translation_sigma));
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(from, to, to_gtsam_pose(z), robust));
    try {
        // 优化与残差检查都在副本上进行，鲁棒核不能替代显式拒绝门控。
        gtsam::ISAM2 trial = d.isam;
        trial.update(factors, gtsam::Values());
        for (int k = 0; k < 5; ++k) {
            trial.update();
        }
        const auto estimate = trial.calculateEstimate();
        for (size_t i = 0; i < d.raw.size(); ++i) {
            if (!estimate.at<gtsam::Pose3>(i).matrix().allFinite()) {
                reason = "非有限图解";
                return false;
            }
        }
        auto valid = [&](size_t a, size_t b, const Pose& measurement, bool loop) {
            const Pose error = measurement.inverse() *
                               from_gtsam_pose(estimate.at<gtsam::Pose3>(a)).inverse() *
                               from_gtsam_pose(estimate.at<gtsam::Pose3>(b));
            return error.translation().norm() <= (loop ? d.settings.max_loop_error_translation
                                                       : d.settings.max_odom_error_translation) &&
                   Eigen::AngleAxisd(error.rotation()).angle() <=
                       (loop ? d.settings.max_loop_error_rotation
                             : d.settings.max_odom_error_rotation);
        };
        if (!valid(from, to, z, true)) {
            reason = "新回环残差过大";
            return false;
        }
        for (const auto& e : d.edges) {
            if (!valid(e.a, e.b, e.z, e.loop)) {
                reason = "历史因子残差过大";
                return false;
            }
        }
        // 新增约束和全部历史约束仍一致，才提交候选图。
        d.isam = std::move(trial);
        d.edges.push_back({from, to, z, true});
        return true;
    } catch (const std::exception& e) {
        reason = e.what();
        return false;
    }
}

std::vector<Pose> PoseGraph::poses() const {
    std::vector<Pose> result;
    const auto values = impl_->isam.calculateEstimate();
    for (size_t i = 0; i < impl_->raw.size(); ++i) {
        result.push_back(from_gtsam_pose(values.at<gtsam::Pose3>(i)));
    }
    return result;
}

void PoseGraph::save(const std::string& path) const {
    std::ofstream f(path);
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f << std::setprecision(17);
    const auto p = poses();
    auto write = [&](const Pose& t) {
        Eigen::Quaterniond q(t.rotation());
        f << t.translation().transpose() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
          << q.w();
    };
    for (size_t i = 0; i < p.size(); ++i) {
        f << "VERTEX_SE3:QUAT " << i << ' ';
        write(p[i]);
        f << '\n';
    }
    for (const auto& e : impl_->edges) {
        f << "EDGE_SE3:QUAT " << e.a << ' ' << e.b << ' ';
        write(e.z);
        // g2o为translation/rotation顺序，GTSAM为rotation/translation。
        const auto& s = impl_->settings;
        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                double sigma = i < 3
                                   ? (e.loop ? s.loop_translation_sigma : s.odom_translation_sigma)
                                   : (e.loop ? s.loop_rotation_sigma : s.odom_rotation_sigma);
                f << ' ' << (i == j ? 1.0 / (sigma * sigma) : 0.0);
            }
        }
        f << '\n';
    }
    f.flush();
}
}  // namespace ly_locate
