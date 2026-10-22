// Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
#include <pcl/io/pcd_io.h>

#include <condition_variable>
#include <deque>
#include <ly_lio/msg/scan_session.hpp>
#include <ly_locate/msg/keyframe.hpp>
#include <ly_locate/msg/loop_update.hpp>
#include <ly_locate/srv/get_keyframe.hpp>
#include <map>
#include <set>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "ly_locate/common.hpp"
#include "ly_locate/odometry_buffer.hpp"
#include "ly_locate/registration.hpp"
#include "ly_locate/scan_context.hpp"

namespace ly_locate {
// ROS 回调只接收入队；工作线程依次处理扫描、持久化关键帧并更新位姿图。
class LoopNode final : public rclcpp::Node {
    struct Origin {
        int64_t stamp;
        Pose body_in_odom;
    };

    struct Frame {
        int64_t stamp;
        Pose raw;
        Cloud::Ptr points;
        ScanContext descriptor;
        double travel;

        Frame(int64_t s, Pose p, Cloud::Ptr c, double radius, double d)
            : stamp(s), raw(p), points(std::move(c)), descriptor(*points, radius), travel(d) {
        }
    };

    struct Input {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
        std::chrono::steady_clock::time_point arrival;
    };

public:
    LoopNode() : Node("location_loop") {
        // 关键帧、描述子、几何验证、位姿图分别配置，默认门限保持保守。
        Config c(*this, "location_loop");
        cloud_topic_ = c.get<std::string>("cloud_topic", "/ly_lio/cloud_registered");
        const auto session_topic = c.get<std::string>("session_topic", "/ly_lio/scan_session");
        const auto odometry_topic = c.get<std::string>("odometry_topic", "/ly_lio/odometry");
        odom_ = c.get<std::string>("odom_frame", "odom");
        body_ = c.get<std::string>("body_frame", "baselink");
        map_ = c.get<std::string>("map_frame", "map");
        archive_ = c.get<std::string>("archive_directory", "maps/location");
        distance_ = c.get("keyframe_distance", 0.75);
        rotation_ = c.get("keyframe_angle", 0.20);
        interval_ = c.get("min_keyframe_interval", 0.3);
        leaf_ = c.get("keyframe_leaf", 0.2);
        min_points_ = c.get("min_cloud_points", 300);
        max_frames_ = c.get("max_keyframes", 20000);
        queue_limit_ = c.get("input_queue_size", 100);
        odometry_wait_ = c.get("odometry_wait_seconds", 2.0);
        radius_ = c.get("scan_context_radius", 60.0);
        sc_threshold_ = c.get("scan_context_threshold", 0.1);
        sc_margin_ = c.get("scan_context_margin", 0.03);
        sc_ratio_ = c.get("scan_context_ratio", 0.75);
        min_bins_ = c.get("min_descriptor_bins", 80);
        min_sectors_ = c.get("min_descriptor_sectors", 30);
        candidates_ = c.get("candidate_count", 20);
        exclude_ = c.get("exclude_recent_keyframes", 30);
        min_time_ = c.get("min_loop_time", 30.0);
        min_travel_ = c.get("min_loop_travel", 15.0);
        candidate_radius_ = c.get("candidate_radius", 8.0);
        distinct_ = c.get("distinct_candidate_distance", 5.0);
        confirm_ = c.get("confirm_frames", 3);
        confirm_translation_ = c.get("confirm_translation", 0.25);
        confirm_rotation_ = c.get("confirm_rotation", 0.08);
        confirm_motion_ = c.get("confirm_min_motion", 1.0);
        cooldown_ = c.get("loop_cooldown_frames", 10);
        reg_.correspondence = c.get("icp_correspondence", 0.5);
        reg_.inlier_distance = c.get("icp_inlier_distance", 0.2);
        reg_.max_rmse = c.get("icp_max_rmse", 0.1);
        reg_.min_overlap = c.get("icp_min_overlap", 0.7);
        reg_.min_inliers = c.get("icp_min_inliers", 150);
        reg_.iterations = c.get("icp_iterations", 60);
        reg_.cycle_translation = c.get("icp_cycle_translation", 0.08);
        reg_.cycle_rotation = c.get("icp_cycle_rotation", 0.035);
        reg_.max_translation_correction = c.get("max_translation_correction", 1.5);
        reg_.max_rotation_correction = c.get("max_rotation_correction", 0.35);
        reg_.min_information_ratio = c.get("min_information_ratio", 0.002);
        graph_settings_.odom_rotation_sigma = c.get("odom_rotation_sigma", 0.02);
        graph_settings_.odom_translation_sigma = c.get("odom_translation_sigma", 0.1);
        graph_settings_.loop_rotation_sigma = c.get("loop_rotation_sigma", 0.03);
        graph_settings_.loop_translation_sigma = c.get("loop_translation_sigma", 0.08);
        graph_settings_.max_odom_error_translation = c.get("graph_max_odom_translation", 0.5);
        graph_settings_.max_odom_error_rotation = c.get("graph_max_odom_rotation", 0.15);
        graph_settings_.max_loop_error_translation = c.get("graph_max_loop_translation", 0.15);
        graph_settings_.max_loop_error_rotation = c.get("graph_max_loop_rotation", 0.08);
        for (double v : {distance_,
                         rotation_,
                         interval_,
                         leaf_,
                         odometry_wait_,
                         radius_,
                         sc_threshold_,
                         sc_margin_,
                         sc_ratio_,
                         min_time_,
                         min_travel_,
                         candidate_radius_,
                         distinct_,
                         confirm_translation_,
                         confirm_rotation_,
                         confirm_motion_,
                         reg_.correspondence,
                         reg_.inlier_distance,
                         reg_.max_rmse,
                         reg_.min_overlap,
                         reg_.cycle_translation,
                         reg_.cycle_rotation,
                         reg_.max_translation_correction,
                         reg_.max_rotation_correction,
                         reg_.min_information_ratio,
                         graph_settings_.odom_rotation_sigma,
                         graph_settings_.odom_translation_sigma,
                         graph_settings_.loop_rotation_sigma,
                         graph_settings_.loop_translation_sigma,
                         graph_settings_.max_odom_error_translation,
                         graph_settings_.max_odom_error_rotation,
                         graph_settings_.max_loop_error_translation,
                         graph_settings_.max_loop_error_rotation}) {
            positive(v, "回环参数");
        }
        if (odom_.empty() || body_.empty() || map_.empty() || odom_ == body_ || map_ == odom_ ||
            map_ == body_ || archive_.empty() || min_points_ < 30 || max_frames_ < 100 ||
            queue_limit_ < 1 || candidates_ < 2 || exclude_ < 1 || confirm_ < 3 || cooldown_ < 1 ||
            min_bins_ < 1 || min_sectors_ < 20 || min_sectors_ > 60 || sc_threshold_ >= 1 ||
            sc_ratio_ >= 1 || reg_.min_overlap > 1 || reg_.min_inliers < 30 ||
            reg_.iterations < 1 || reg_.min_information_ratio >= 1) {
            throw std::invalid_argument("回环参数范围非法");
        }
        update_pub_ = create_publisher<msg::LoopUpdate>(
            "/location/loop", rclcpp::QoS(1).reliable().transient_local());
        keyframe_pub_ =
            create_publisher<msg::Keyframe>("/location/keyframe", rclcpp::QoS(10).reliable());
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, rclcpp::QoS(1000).reliable(),
            [this](const nav_msgs::msg::Odometry& m) {
                try {
                    odometry_.insert(m, odom_, body_);
                    cv_.notify_one();
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                         "拒绝Odometry: %s", e.what());
                }
            });
        session_sub_ = create_subscription<ly_lio::msg::ScanSession>(
            session_topic, rclcpp::QoS(1000).reliable().transient_local(),
            [this](const ly_lio::msg::ScanSession& m) {
                try {
                    if (m.header.frame_id != odom_ || m.body_frame != body_ ||
                        m.session_id.empty()) {
                        return;
                    }
                    const auto stamp = ns(m.header.stamp);
                    const Origin origin{ns(m.origin_stamp), pose(m.origin_pose)};
                    if (origin.stamp > stamp) {
                        throw std::invalid_argument("会话原点晚于当前Odometry");
                    }
                    std::lock_guard lock(input_mutex_);
                    const auto previous = origins_.find(m.session_id);
                    if (previous != origins_.end() &&
                        (previous->second.stamp != origin.stamp ||
                         !previous->second.body_in_odom.matrix().isApprox(
                             origin.body_in_odom.matrix(), 1e-12))) {
                        invalid_origins_.insert(m.session_id);
                        throw std::invalid_argument("同会话首帧机体原点冲突，拒绝继续建图");
                    }
                    origins_.emplace(m.session_id, origin);
                    auto it = sessions_.find(stamp);
                    // 循环回放重用stamp且跨话题乱序时宁可丢点，不猜测其所属会话。
                    if (it != sessions_.end() && it->second != m.session_id) {
                        ambiguous_.insert(stamp);
                    }
                    sessions_[stamp] = m.session_id;
                    while (sessions_.size() > 10000) {
                        sessions_.erase(sessions_.begin());
                    }
                    cv_.notify_one();
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "会话标签拒绝: %s", e.what());
                }
            });
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::QoS(10).reliable(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
                std::lock_guard lock(input_mutex_);
                if (queue_.size() >= size_t(queue_limit_)) {
                    queue_.pop_front();
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                                          "回环输入队列溢出，丢弃未处理扫描；不是无损建图");
                }
                queue_.push_back({std::move(m), std::chrono::steady_clock::now()});
                cv_.notify_one();
            });
        fetch_ = create_service<srv::GetKeyframe>(
            "/location/get_keyframe",
            [this](const std::shared_ptr<srv::GetKeyframe::Request> request,
                   std::shared_ptr<srv::GetKeyframe::Response> response) {
                try {
                    std::lock_guard lock(state_mutex_);
                    if (request->session_id != session_ || request->id >= frames_.size()) {
                        response->message = "会话或关键帧不存在";
                        return;
                    }
                    response->keyframe = keyframe(request->id);
                    response->success = true;
                } catch (const std::exception& e) {
                    response->message = e.what();
                }
            });
        worker_ = std::thread([this] {
            work();
        });
        RCLCPP_INFO(
            get_logger(),
            "在线回环启用：先收历史关键帧，无预建图；/location/loop发布带revision的完整图快照");
    }

    ~LoopNode() override {
        {
            std::lock_guard lock(input_mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    // input_mutex_ 保护输入队列和标签；state_mutex_ 保护关键帧服务读取的图状态。
    void work() {
        while (rclcpp::ok()) {
            Input input;
            std::string source_session;
            Origin origin;
            {
                std::unique_lock lock(input_mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(20), [this] {
                    return stop_ || !queue_.empty();
                });
                if (stop_) {
                    return;
                }
                if (queue_.empty()) {
                    continue;
                }
                input = queue_.front();
                int64_t stamp;
                try {
                    stamp = ns(input.cloud->header.stamp);
                } catch (...) {
                    queue_.pop_front();
                    continue;
                }
                auto tag = sessions_.find(stamp);
                if (ambiguous_.count(stamp)) {
                    queue_.pop_front();
                    continue;
                }
                if (tag != sessions_.end()) {
                    source_session = tag->second;
                    if (invalid_origins_.count(source_session)) {
                        queue_.pop_front();
                        continue;
                    }
                    origin = origins_.at(source_session);
                }
            }
            const auto age =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - input.arrival)
                    .count();
            if (source_session.empty()) {
                if (age < odometry_wait_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                pop(input);
                RCLCPP_WARN(get_logger(), "扫描缺少精确stamp会话标签，拒绝跨原点建图");
                continue;
            }
            try {
                if (retired_.count(source_session)) {
                    pop(input);
                    continue;
                }
                if (source_session != source_session_) {
                    std::lock_guard lock(state_mutex_);
                    if (!source_session_.empty()) {
                        retired_.insert(source_session_);
                    }
                    source_session_ = source_session;
                    session_ = source_session + "_" + unique_id();
                    frames_.clear();
                    constraints_.clear();
                    pending_targets_.clear();
                    origin_ = origin;
                    // map就是首个标定后baselink：T_MO = inverse(T_OB_first)。
                    graph_ = std::make_unique<PoseGraph>(
                        graph_settings_, origin_.body_in_odom.inverse());
                    revision_ = 0;
                    pending_ = 0;
                    last_loop_ = 0;
                    faulted_ = false;
                    directory_ = std::filesystem::path(archive_) / session_;
                    std::filesystem::create_directories(directory_ / "keyframes");
                    publish(false, 0, 0, 0, 0, 0);
                    RCLCPP_WARN(get_logger(), "建立新图会话 %s；旧图已隔离", session_.c_str());
                }
                const Pose raw = odometry_.lookup(ns(input.cloud->header.stamp));
                pop(input);
                if (input.cloud->header.frame_id != body_) {
                    RCLCPP_ERROR(get_logger(), "只接受ly_lio扫描末机体系注册云，拒绝重复坐标变换");
                    continue;
                }
                std::lock_guard lock(state_mutex_);
                if (faulted_) {
                    continue;
                }
                process(*input.cloud, raw);
            } catch (const OdometryUnavailable& e) {
                if (age >= odometry_wait_) {
                    pop(input);
                    RCLCPP_WARN(get_logger(), "同stamp Odometry超时，丢扫描: %s", e.what());
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            } catch (const std::exception& e) {
                pop(input);
                std::lock_guard lock(state_mutex_);
                faulted_ = true;
                RCLCPP_ERROR(get_logger(), "本会话建图停止，保留已提交图，需排查/重启: %s",
                             e.what());
            }
        }
    }

    void pop(const Input& input) {
        std::lock_guard lock(input_mutex_);
        if (!queue_.empty() && queue_.front().cloud == input.cloud) {
            queue_.pop_front();
        }
    }

    msg::Keyframe keyframe(size_t id) const {
        const auto& f = frames_.at(id);
        msg::Keyframe k;
        k.header.stamp = rclcpp::Time(f.stamp);
        k.header.frame_id = body_;
        k.id = id;
        k.session_id = session_;
        k.odom_pose = message(f.raw);
        k.cloud = message(*f.points, k.header);
        return k;
    }

    void process(const sensor_msgs::msg::PointCloud2& m, const Pose& raw) {
        const int64_t stamp = ns(m.header.stamp);
        if (!frames_.empty()) {
            if (stamp <= frames_.back().stamp) {
                RCLCPP_WARN(get_logger(), "拒绝倒序/重复关键帧");
                return;
            }
            const Pose delta = frames_.back().raw.inverse() * raw;
            if ((stamp - frames_.back().stamp) * 1e-9 < interval_ ||
                (delta.translation().norm() < distance_ && angle(delta) < rotation_)) {
                return;
            }
        }
        if (frames_.size() >= size_t(max_frames_)) {
            throw std::runtime_error("关键帧容量达到上限，不静默丢历史");
        }
        // 注册云已经在扫描末 body 系，直接保存；不重复施加位姿逆或雷达安装外参。
        auto local = downsample(cloud(m), leaf_);
        if (local->size() < size_t(min_points_)) {
            return;
        }
        const size_t id = frames_.size();
        const double travel = id ? frames_.back().travel +
                                       (raw.translation() - frames_.back().raw.translation()).norm()
                                 : 0;
        // 持久局部点云先于图提交；任何磁盘错误停止会话，不发布不可恢复的关键帧。
        const auto path = directory_ / "keyframes" / (std::to_string(id) + ".pcd");
        const auto tmp = path.string() + ".tmp";
        if (pcl::io::savePCDFileBinaryCompressed(tmp, *local) != 0) {
            throw std::runtime_error("关键帧PCD写入失败");
        }
        std::filesystem::rename(tmp, path);
        graph_->append(raw);
        frames_.emplace_back(stamp, raw, local, radius_, travel);
        keyframe_pub_->publish(keyframe(id));
        bool closed = false;
        size_t target = 0;
        double sc = 0;
        double rmse = 0;
        double overlap = 0;
        if (id >= last_loop_ + size_t(cooldown_) &&
            frames_.back().descriptor.occupied >= min_bins_ &&
            frames_.back().descriptor.occupied_sectors >= min_sectors_) {
            closed = detect(id, target, sc, rmse, overlap);
        } else {
            pending_ = 0;
        }
        publish(closed, target, id, sc, rmse, overlap);
    }

    bool detect(size_t current, size_t& target, double& sc, double& rmse, double& overlap) {
        // 描述子只产生候选，独立几何检查与连续运动确认通过后才允许加因子。
        const auto optimized = graph_->poses();
        const auto& now = frames_[current];
        std::vector<std::pair<double, size_t>> shortlist;
        for (size_t i = 0; i + size_t(exclude_) < current; ++i) {
            const auto& old = frames_[i];
            if ((now.stamp - old.stamp) * 1e-9 < min_time_ ||
                now.travel - old.travel < min_travel_ || old.descriptor.occupied < min_bins_ ||
                old.descriptor.occupied_sectors < min_sectors_ ||
                (optimized[current].translation() - optimized[i].translation()).norm() >
                    candidate_radius_) {
                continue;
            }
            shortlist.emplace_back((old.descriptor.key - now.descriptor.key).squaredNorm(), i);
        }
        std::sort(shortlist.begin(), shortlist.end());
        if (shortlist.size() > size_t(candidates_)) {
            shortlist.resize(candidates_);
        }

        struct Candidate {
            size_t id;
            ScanContext::Match match;
        };

        std::vector<Candidate> ranked;
        for (const auto& [key, i] : shortlist) {
            (void)key;
            ranked.push_back({i, frames_[i].descriptor.compare(now.descriptor)});
        }
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
            return a.match.distance < b.match.distance;
        });
        auto reject = [this] {
            pending_ = 0;
            return false;
        };
        if (ranked.empty() || ranked.front().match.distance > sc_threshold_) {
            return reject();
        }
        const auto best = ranked.front();
        target = best.id;
        sc = best.match.distance;
        // 同一地点邻近关键帧可相似；另一个空间不同地点也相似则整次拒绝。
        for (size_t k = 1; k < ranked.size(); ++k) {
            if ((frames_[ranked[k].id].raw.translation() - frames_[target].raw.translation())
                    .norm() >= distinct_) {
                if (ranked[k].match.distance - sc < sc_margin_ ||
                    sc > sc_ratio_ * ranked[k].match.distance) {
                    return reject();
                }
                break;
            }
        }
        const Pose predicted = optimized[target].inverse() * optimized[current];
        Pose initial = predicted;
        const double initial_yaw = std::atan2(initial(1, 0), initial(0, 0));
        const double yaw_delta = std::remainder(best.match.yaw - initial_yaw, 2 * M_PI);
        if (std::abs(yaw_delta) > reg_.max_rotation_correction) {
            return reject();
        }
        initial.linear() =
            Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            initial.rotation();
        RegistrationResult fit;
        std::string why;
        // 用独立单帧双向验证，防止把邻近地图/当前扫描泄漏到配准目标而虚增重叠率。
        if (!verify_registration(now.points, frames_[target].points, initial, reg_, fit, why)) {
            pending_ = 0;
            RCLCPP_DEBUG(get_logger(), "回环 %zu -> %zu 拒绝: %s", target, current, why.c_str());
            return false;
        }
        const Pose innovation = predicted.inverse() * fit.transform;
        if (innovation.translation().norm() > reg_.max_translation_correction ||
            angle(innovation) > reg_.max_rotation_correction) {
            return reject();
        }
        // 连续当前帧须匹配不同历史帧，并给出一致修正；真实运动避免静止重复充当证据。
        const Pose correction = frames_[target].raw * fit.transform * now.raw.inverse();
        bool consistent =
            pending_ > 0 && pending_current_ + 1 == current && !pending_targets_.count(target) &&
            (frames_[target].raw.translation() - frames_[pending_target_].raw.translation())
                    .norm() < distinct_;
        if (consistent) {
            const Pose change = pending_correction_.inverse() * correction;
            consistent = change.translation().norm() <= confirm_translation_ &&
                         angle(change) <= confirm_rotation_;
        }
        if (!consistent) {
            pending_ = 1;
            pending_targets_.clear();
            pending_start_travel_ = now.travel;
            pending_correction_ = correction;
        } else {
            ++pending_;
        }
        pending_targets_.insert(target);
        pending_current_ = current;
        pending_target_ = target;
        if (pending_ < confirm_ || now.travel - pending_start_travel_ < confirm_motion_) {
            return false;
        }
        pending_ = 0;
        if (!graph_->try_loop(target, current, fit.transform, why)) {
            RCLCPP_WARN(get_logger(), "GTSAM回环事务拒绝: %s", why.c_str());
            return false;
        }
        last_loop_ = current;
        rmse = fit.rmse;
        overlap = fit.overlap;
        msg::LoopConstraint constraint;
        constraint.from_id = target;
        constraint.to_id = current;
        constraint.accepted_revision = revision_ + 1;
        constraint.relative_pose = message(fit.transform);
        constraint.scan_context_distance = sc;
        constraint.registration_rmse = rmse;
        constraint.overlap = overlap;
        constraints_.push_back(constraint);
        RCLCPP_INFO(get_logger(), "接受多帧确认回环 %zu -> %zu, SC=%.4f RMSE=%.3fm overlap=%.3f",
                    target, current, sc, rmse, overlap);
        return true;
    }

    void publish(bool closed, size_t from, size_t to, double sc, double rmse, double overlap) {
        msg::LoopUpdate update;
        update.header.frame_id = map_;
        update.session_id = session_;
        update.source_session_id = source_session_;
        update.odom_frame = odom_;
        update.body_frame = body_;
        update.origin_stamp = rclcpp::Time(origin_.stamp);
        update.origin_pose = message(origin_.body_in_odom);
        update.revision = ++revision_;
        update.loop_closed = closed;
        update.loop_from = from;
        update.loop_to = to;
        update.scan_context_distance = sc;
        update.registration_rmse = rmse;
        update.overlap = overlap;
        update.constraints = constraints_;
        const auto poses = graph_->poses();
        std::ostringstream csv;
        csv << std::setprecision(17)
            << "id,stamp_ns,odom_x,odom_y,odom_z,odom_qx,odom_qy,odom_qz,odom_qw,map_x,map_y,map_z,"
               "map_qx,"
               "map_qy,map_qz,map_qw\n";
        auto write_pose = [&](const Pose& t) {
            Eigen::Quaterniond q(t.rotation());
            csv << ',' << t.translation().x() << ',' << t.translation().y() << ','
                << t.translation().z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
                << q.w();
        };
        for (size_t i = 0; i < frames_.size(); ++i) {
            msg::KeyframePose p;
            p.id = i;
            p.stamp = rclcpp::Time(frames_[i].stamp);
            p.odom_pose = message(frames_[i].raw);
            p.map_pose = message(poses[i]);
            update.poses.push_back(p);
            csv << i << ',' << frames_[i].stamp;
            write_pose(frames_[i].raw);
            write_pose(poses[i]);
            csv << '\n';
        }
        if (!frames_.empty()) {
            update.header.stamp = update.poses.back().stamp;
        }
        // 完整历史优化位姿及原始位姿持久化；PCD单独保存，不依赖迟到实时消息恢复地图。
        std::ostringstream loops;
        loops << std::setprecision(17)
              << "from_id,to_id,accepted_revision,x,y,z,qx,qy,qz,qw,scan_context_distance,"
                 "registration_rmse,"
                 "overlap\n";
        for (const auto& edge : constraints_) {
            loops << edge.from_id << ',' << edge.to_id << ',' << edge.accepted_revision;
            const auto& p = edge.relative_pose;
            loops << ',' << p.position.x << ',' << p.position.y << ',' << p.position.z << ','
                  << p.orientation.x << ',' << p.orientation.y << ',' << p.orientation.z << ','
                  << p.orientation.w;
            loops << ',' << edge.scan_context_distance << ',' << edge.registration_rmse << ','
                  << edge.overlap << '\n';
        }
        write_text(directory_ / "loops.csv", loops.str());
        write_text(directory_ / "poses.csv", csv.str());
        graph_->save((directory_ / "graph.g2o.tmp").string());
        std::filesystem::rename(directory_ / "graph.g2o.tmp", directory_ / "graph.g2o");
        const Pose odom_to_map = origin_.body_in_odom.inverse();
        const Eigen::Quaterniond map_rotation(odom_to_map.rotation());
        std::ostringstream meta;
        meta << std::setprecision(17)
             << "session_id: " << session_ << "\nsource_session_id: " << source_session_
             << "\nrevision: " << revision_ << "\nmap_frame: " << map_ << "\nodom_frame: " << odom_
             << "\nbody_frame: " << body_ << "\nmap_origin: first_calibrated_baselink"
             << "\norigin_stamp_ns: " << origin_.stamp
             << "\nmap_from_odom_translation: [" << odom_to_map.translation().x() << ", "
             << odom_to_map.translation().y() << ", " << odom_to_map.translation().z() << "]"
             << "\nmap_from_odom_quaternion_xyzw: [" << map_rotation.x() << ", "
             << map_rotation.y() << ", " << map_rotation.z() << ", " << map_rotation.w()
             << "]\n";
        write_text(directory_ / "session.yaml", meta.str());
        update_pub_->publish(update);
    }

    // 固定配置与当前图会话。
    std::string cloud_topic_;
    std::string odom_;
    std::string body_;
    std::string map_;
    std::string archive_;
    std::string source_session_;
    std::string session_;
    std::filesystem::path directory_;
    std::set<std::string> retired_;
    Origin origin_;
    double distance_;
    double rotation_;
    double interval_;
    double leaf_;
    double odometry_wait_;
    double radius_;
    double sc_threshold_;
    double sc_margin_;
    double sc_ratio_;
    double min_time_;
    double min_travel_;
    double candidate_radius_;
    double distinct_;
    double confirm_translation_;
    double confirm_rotation_;
    double confirm_motion_;
    int min_points_;
    int max_frames_;
    int queue_limit_;
    int min_bins_;
    int min_sectors_;
    int candidates_;
    int exclude_;
    int confirm_;
    int cooldown_;

    // 工作线程更新；关键帧服务读取时持有 state_mutex_。
    RegistrationSettings reg_;
    GraphSettings graph_settings_;
    std::unique_ptr<PoseGraph> graph_;
    std::vector<Frame> frames_;
    std::vector<msg::LoopConstraint> constraints_;
    std::set<size_t> pending_targets_;
    uint64_t revision_ = 0;
    size_t last_loop_ = 0;
    size_t pending_current_ = 0;
    size_t pending_target_ = 0;
    int pending_ = 0;
    double pending_start_travel_ = 0;
    Pose pending_correction_ = Pose::Identity();
    bool faulted_ = false;

    // ROS 通信及输入队列；析构先通知停止，再等待工作线程退出。
    OdometryBuffer odometry_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<ly_lio::msg::ScanSession>::SharedPtr session_sub_;
    rclcpp::Publisher<msg::LoopUpdate>::SharedPtr update_pub_;
    rclcpp::Publisher<msg::Keyframe>::SharedPtr keyframe_pub_;
    rclcpp::Service<srv::GetKeyframe>::SharedPtr fetch_;
    std::mutex input_mutex_;
    std::mutex state_mutex_;
    std::condition_variable cv_;
    std::deque<Input> queue_;
    std::map<int64_t, std::string> sessions_;
    std::map<std::string, Origin> origins_;
    std::set<std::string> invalid_origins_;
    std::set<int64_t> ambiguous_;
    std::thread worker_;
    bool stop_ = false;
};
}  // namespace ly_locate

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ly_locate::LoopNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("location_loop"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
