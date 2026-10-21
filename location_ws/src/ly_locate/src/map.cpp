// Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
#include <pcl/io/pcd_io.h>
#include <tf2_ros/transform_broadcaster.h>

#include <condition_variable>
#include <deque>
#include <ly_lio/msg/scan_session.hpp>
#include <ly_locate/msg/keyframe.hpp>
#include <ly_locate/msg/loop_update.hpp>
#include <ly_locate/srv/get_keyframe.hpp>
#include <map>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <set>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "ly_locate/common.hpp"
#include "ly_locate/odometry_buffer.hpp"

namespace ly_locate {
// 历史地图按完整图版本重建；真实机体Odometry经已提交图修正后直接输出map→baselink。
class MapNode final : public rclcpp::Node {
    struct Stored {
        msg::Keyframe::ConstSharedPtr message;
        Cloud::Ptr local;
    };

    struct Live {
        std_msgs::msg::Header header;
        std::chrono::steady_clock::time_point arrival;
    };

public:
    MapNode()
        : Node("location_map"),
          broadcaster_(*this) {
        Config c(*this, "location_map");
        const auto odometry_topic = c.get<std::string>("odometry_topic", "/ly_lio/odometry");
        const auto session_topic = c.get<std::string>("session_topic", "/ly_lio/scan_session");
        odom_ = c.get<std::string>("odom_frame", "odom");
        body_ = c.get<std::string>("body_frame", "baselink");
        map_ = c.get<std::string>("map_frame", "map");
        leaf_ = c.get("map_leaf", 0.2);
        publish_interval_ = c.get("map_publish_interval", 2.0);
        save_directory_ = c.get<std::string>("save_directory", "maps/location_export");
        save_on_shutdown_ = c.get("save_on_shutdown", true);
        odometry_wait_ = c.get("odometry_wait_seconds", 2.0);
        queue_limit_ = c.get("input_queue_size", 100);
        positive(leaf_, "map_leaf");
        positive(publish_interval_, "map_publish_interval");
        positive(odometry_wait_, "odometry_wait_seconds");
        if (odom_.empty() || map_.empty() || body_.empty() || odom_ == map_ || odom_ == body_ ||
            map_ == body_ || save_directory_.empty() || queue_limit_ < 1) {
            throw std::invalid_argument("建图参数非法");
        }
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/location/map", rclcpp::QoS(1).reliable().transient_local());
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/location/path", rclcpp::QoS(1).reliable().transient_local());
        update_sub_ = create_subscription<msg::LoopUpdate>(
            "/location/loop", rclcpp::QoS(1).reliable().transient_local(),
            [this](msg::LoopUpdate::ConstSharedPtr m) {
                try {
                    accept_update(m);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "拒绝图快照: %s", e.what());
                }
            });
        keyframe_sub_ = create_subscription<msg::Keyframe>(
            "/location/keyframe", rclcpp::QoS(10).reliable(),
            [this](msg::Keyframe::ConstSharedPtr m) {
                try {
                    accept_keyframe(m);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "拒绝关键帧: %s", e.what());
                }
            });
        session_sub_ = create_subscription<ly_lio::msg::ScanSession>(
            session_topic, rclcpp::QoS(1000).reliable().transient_local(),
            [this](const ly_lio::msg::ScanSession& m) {
                try {
                    if (m.header.frame_id != odom_ || m.session_id.empty()) {
                        return;
                    }
                    const auto stamp = ns(m.header.stamp);
                    std::lock_guard lock(mutex_);
                    if (retired_sources_.count(m.session_id)) {
                        return;
                    }
                    if (!source_.empty() && source_ != m.session_id) {
                        retired_sources_.insert(source_);
                        reset_locked();
                    }
                    source_ = m.session_id;
                    const auto it = tags_.find(stamp);
                    if (it != tags_.end() && it->second != source_) {
                        ambiguous_.insert(stamp);
                    }
                    tags_[stamp] = source_;
                    while (tags_.size() > 10000) {
                        tags_.erase(tags_.begin());
                    }
                    cv_.notify_one();
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "会话拒绝: %s", e.what());
                }
            });
        // 只收轻量位姿；不订阅、转换或缓存整帧注册云。
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, rclcpp::QoS(1000).reliable(),
            [this](const nav_msgs::msg::Odometry& m) {
                try {
                    odometry_.insert(m, odom_, body_);
                    std::lock_guard lock(mutex_);
                    auto header = m.header;
                    header.frame_id = body_;
                    enqueue_live_locked({header, std::chrono::steady_clock::now()});
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                         "拒绝Odometry: %s", e.what());
                }
            });
        fetch_ = create_client<srv::GetKeyframe>("/location/get_keyframe");
        fetch_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
            fetch_missing();
        });
        save_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        save_ = create_service<std_srvs::srv::Trigger>(
            "/location/map_save",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
                try {
                    r->message = save();
                    r->success = true;
                } catch (const std::exception& e) {
                    r->message = e.what();
                }
            },
            rmw_qos_profile_services_default, save_group_);
        worker_ = std::thread([this] {
            work();
        });
    }

    ~MapNode() override { stop_worker(); }

    // executor停止后调用：先等工作线程结束，再做纯文件导出；不依赖已关闭的ROS服务。
    bool save_before_exit() {
        stop_worker();
        if (!save_on_shutdown_) {
            RCLCPP_INFO(get_logger(), "退出自动保存已关闭（save_on_shutdown=false）");
            return true;
        }
        {
            std::lock_guard lock(mutex_);
            if (!graph_ || graph_->poses.empty()) {
                RCLCPP_INFO(get_logger(), "退出时无有效关键帧地图，跳过自动保存");
                return true;
            }
        }
        try {
            // 已停止接收和重建，可直接重建最新完整快照，不受周期发布限频阻挡。
            RCLCPP_INFO(get_logger(), "退出前自动保存地图，请等待完成…");
            RCLCPP_INFO(get_logger(), "%s", save(false).c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(),
                         "退出自动保存失败: %s；回环关键帧归档仍在，未生成完整地图导出", e.what());
            return false;
        }
    }

private:
    void enqueue_live_locked(Live live) {
        if (live_queue_.size() >= size_t(queue_limit_)) {
            live_queue_.pop_front();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "实时位姿队列溢出；历史关键帧另由持久节点恢复");
        }
        live_queue_.push_back(std::move(live));
        cv_.notify_one();
    }

    void stop_worker() {
        {
            std::lock_guard lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    void reset_locked() {
        if (graph_) {
            retired_graphs_.insert(graph_->session_id);
        }
        graph_.reset();
        keys_.clear();
        staged_.clear();
        committed_.reset();
        latest_live_pose_.reset();
        latest_live_header_.reset();
        last_tf_stamp_ = -1;
        dirty_ = true;
        // 发布空图使晚到订阅者不会继续显示旧odom原点地图。
        std_msgs::msg::Header h;
        h.frame_id = map_;
        map_pub_->publish(message(Cloud{}, h));
        nav_msgs::msg::Path p;
        p.header = h;
        path_pub_->publish(p);
    }

    // 先验证结构，再持锁检查会话、版本和历史不可变项。
    // 不按到达时刻给回环盖时间戳，不让旧版本覆盖新地图。
    void accept_update(const msg::LoopUpdate::ConstSharedPtr& m) {
        if (m->header.frame_id != map_ || m->odom_frame != odom_ || m->body_frame != body_ ||
            m->session_id.empty() || m->source_session_id.empty() || m->revision == 0) {
            throw std::invalid_argument("图坐标系/会话/revision不匹配");
        }
        int64_t last = -1;
        for (size_t i = 0; i < m->poses.size(); ++i) {
            const auto& p = m->poses[i];
            const auto stamp = ns(p.stamp);
            if (p.id != i || stamp <= last) {
                throw std::invalid_argument("图关键帧ID/时间不连续递增");
            }
            pose(p.odom_pose);
            pose(p.map_pose);
            last = stamp;
        }
        if (!m->poses.empty() && ns(m->header.stamp) != last) {
            throw std::invalid_argument("图截止时间与末关键帧不一致");
        }
        if (m->loop_closed &&
            (m->loop_from >= m->loop_to || m->loop_to >= m->poses.size() ||
             !std::isfinite(m->registration_rmse) || !std::isfinite(m->overlap))) {
            throw std::invalid_argument("回环元数据非法");
        }
        uint64_t accepted_revision = 0;
        for (const auto& edge : m->constraints) {
            if (edge.from_id >= edge.to_id || edge.to_id >= m->poses.size() ||
                edge.accepted_revision <= accepted_revision ||
                edge.accepted_revision > m->revision ||
                !std::isfinite(edge.scan_context_distance) ||
                !std::isfinite(edge.registration_rmse) || !std::isfinite(edge.overlap) ||
                edge.scan_context_distance < 0 || edge.scan_context_distance > 1 ||
                edge.registration_rmse < 0 || edge.overlap <= 0 || edge.overlap > 1) {
                throw std::invalid_argument("回环约束历史非法");
            }
            pose(edge.relative_pose);
            accepted_revision = edge.accepted_revision;
        }
        std::lock_guard lock(mutex_);
        if (retired_sources_.count(m->source_session_id) || retired_graphs_.count(m->session_id)) {
            return;
        }
        // 图快照自身带原始会话，可先于实时标签到达；新源仍必须由逐帧标签确认。
        if (source_.empty()) {
            source_ = m->source_session_id;
        }
        if (source_ != m->source_session_id) {
            return;
        }
        if (graph_ && graph_->session_id == m->session_id) {
            if (m->revision <= graph_->revision) {
                return;
            }
            if (m->poses.size() < graph_->poses.size() ||
                m->constraints.size() < graph_->constraints.size()) {
                throw std::invalid_argument("同会话历史缩短");
            }
            for (size_t i = 0; i < graph_->constraints.size(); ++i) {
                const auto& a = graph_->constraints[i];
                const auto& b = m->constraints[i];
                if (a.from_id != b.from_id || a.to_id != b.to_id ||
                    a.accepted_revision != b.accepted_revision ||
                    !pose(a.relative_pose)
                         .matrix()
                         .isApprox(pose(b.relative_pose).matrix(), 1e-9)) {
                    throw std::invalid_argument("已接受回环约束被修改");
                }
            }

            for (size_t i = 0; i < graph_->poses.size(); ++i) {
                const auto& a = graph_->poses[i];
                const auto& b = m->poses[i];
                if (ns(a.stamp) != ns(b.stamp) ||
                    !pose(a.odom_pose).matrix().isApprox(pose(b.odom_pose).matrix(), 1e-9)) {
                    throw std::invalid_argument("历史原始位姿被修改");
                }
            }
        } else {
            if (graph_) {
                retired_graphs_.insert(graph_->session_id);
            }
            reset_locked();
            const auto stage = staged_sessions_.find(m->session_id);
            if (stage != staged_sessions_.end()) {
                staged_ = std::move(stage->second);
                staged_sessions_.erase(stage);
            }
        }
        graph_ = m;
        for (auto it = staged_.begin(); it != staged_.end();) {
            if (it->first < m->poses.size()) {
                try {
                    validate_keyframe(*it->second, *m);
                    auto local = cloud(it->second->cloud);
                    if (local->empty()) {
                        throw std::invalid_argument("空暂存关键帧");
                    }
                    keys_[it->first] = {it->second, local};
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "丢弃非法暂存关键帧，后续补取: %s", e.what());
                }
                it = staged_.erase(it);
            } else {
                ++it;
            }
        }
        dirty_ = true;
        cv_.notify_one();
    }

    void validate_keyframe(const msg::Keyframe& m, const msg::LoopUpdate& graph) const {
        if (m.session_id != graph.session_id || m.id >= graph.poses.size() ||
            m.header.frame_id != body_ || m.cloud.header.frame_id != body_ ||
            ns(m.header.stamp) != ns(m.cloud.header.stamp) ||
            ns(m.header.stamp) != ns(graph.poses[m.id].stamp) ||
            !pose(m.odom_pose)
                 .matrix()
                 .isApprox(pose(graph.poses[m.id].odom_pose).matrix(), 1e-8)) {
            throw std::invalid_argument("关键帧与图的ID/stamp/原始位姿不匹配");
        }
    }

    void accept_keyframe(const msg::Keyframe::ConstSharedPtr& m) {
        if (m->session_id.empty() || m->header.frame_id != body_ ||
            m->cloud.header.frame_id != body_) {
            throw std::invalid_argument("局部关键帧坐标系非法");
        }
        std::lock_guard lock(mutex_);
        if (retired_graphs_.count(m->session_id)) {
            return;
        }
        if (!graph_ || graph_->session_id != m->session_id) {
            // 关键帧先于所属图到达；有界暂存，遗漏可按session/id服务补取。
            if (staged_sessions_.size() >= 4 && !staged_sessions_.count(m->session_id)) {
                staged_sessions_.erase(staged_sessions_.begin());
            }
            auto& s = staged_sessions_[m->session_id];
            s[m->id] = m;
            if (s.size() > 100) {
                s.erase(s.begin());
            }
            return;
        }
        if (m->id >= graph_->poses.size()) {
            staged_[m->id] = m;
            if (staged_.size() > 100) {
                staged_.erase(staged_.begin());
            }
            return;
        }
        validate_keyframe(*m, *graph_);
        if (keys_.count(m->id)) {
            return;
        }
        auto points = cloud(m->cloud);
        if (points->empty()) {
            throw std::invalid_argument("空关键帧");
        }
        keys_[m->id] = {m, points};
        dirty_ = true;
        cv_.notify_one();
    }

    // 图可能早于关键帧到达；按 session/id 补取，每次只保留一个在途请求。
    void fetch_missing() {
        std::shared_ptr<srv::GetKeyframe::Request> request;
        {
            std::lock_guard lock(mutex_);
            if (fetch_pending_ &&
                std::chrono::steady_clock::now() - fetch_started_ > std::chrono::seconds(5)) {
                fetch_->remove_pending_request(fetch_request_id_);
                fetch_pending_ = false;
                RCLCPP_WARN(get_logger(), "关键帧补取超时，清理请求并重试");
            }
            if (fetch_pending_ || !graph_ || !fetch_->service_is_ready()) {
                return;
            }
            for (size_t i = 0; i < graph_->poses.size(); ++i) {
                if (!keys_.count(i)) {
                    request = std::make_shared<srv::GetKeyframe::Request>();
                    request->session_id = graph_->session_id;
                    request->id = i;
                    break;
                }
            }
            if (!request) {
                return;
            }
            fetch_pending_ = true;
            fetch_started_ = std::chrono::steady_clock::now();
        }
        try {
            auto pending = fetch_->async_send_request(
                request, [this](rclcpp::Client<srv::GetKeyframe>::SharedFuture future) {
                    {
                        std::lock_guard lock(mutex_);
                        fetch_pending_ = false;
                    }
                    try {
                        const auto response = future.get();
                        if (response->success) {
                            accept_keyframe(std::make_shared<msg::Keyframe>(response->keyframe));
                        } else {
                            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                                 "缺失关键帧暂不可恢复: %s",
                                                 response->message.c_str());
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(get_logger(), "关键帧恢复失败: %s", e.what());
                    }
                });
            std::lock_guard lock(mutex_);
            fetch_request_id_ = pending.request_id;
        } catch (const std::exception& e) {
            std::lock_guard lock(mutex_);
            fetch_pending_ = false;
            RCLCPP_WARN(get_logger(), "补取请求发送失败: %s", e.what());
        }
    }

    // 每帧单独使用优化位姿：p_map = T_MB_k * p_body_k。
    // 回环改变整段历史，不能只用最新T_MO修正整张旧点云。
    Cloud::Ptr build(const msg::LoopUpdate& graph, const std::vector<Stored>& keys) const {
        auto aggregate = std::make_shared<Cloud>();
        for (size_t i = 0; i < keys.size(); ++i) {
            auto p = transformed(keys[i].local, pose(graph.poses[i].map_pose));
            *aggregate += *p;
            // 分段降采样限制中间聚合峰值，不将最新修正刚体套到整张旧图。
            if (aggregate->size() > 2000000) {
                aggregate = downsample(aggregate, leaf_);
            }
        }
        return aggregate->empty() ? aggregate : downsample(aggregate, leaf_);
    }

    void work() {
        auto next_build = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            msg::LoopUpdate::ConstSharedPtr graph;
            std::vector<Stored> keys;
            Live live;
            bool has_live = false;
            bool do_build = false;
            {
                std::unique_lock lock(mutex_);
                if (live_queue_.empty()) {
                    cv_.wait_for(lock, std::chrono::milliseconds(10));
                }
                if (stop_) {
                    return;
                }
                if (!live_queue_.empty()) {
                    live = live_queue_.front();
                    has_live = true;
                }
                if (dirty_ && graph_ && keys_.size() == graph_->poses.size() &&
                    std::chrono::steady_clock::now() >= next_build) {
                    graph = graph_;
                    for (size_t i = 0; i < graph->poses.size(); ++i) {
                        auto it = keys_.find(i);
                        if (it == keys_.end()) {
                            keys.clear();
                            break;
                        }
                        keys.push_back(it->second);
                    }
                    do_build = keys.size() == graph->poses.size();
                }
            }
            if (has_live) {
                process_live(live);
            }
            if (do_build) {
                try {
                    // 锁外重建，提交时再次核对会话，避免 reset 期间串图。
                    auto points = build(*graph, keys);
                    nav_msgs::msg::Path path;
                    path.header = graph->header;
                    for (const auto& p : graph->poses) {
                        geometry_msgs::msg::PoseStamped v;
                        v.header = graph->header;
                        v.header.stamp = p.stamp;
                        v.pose = p.map_pose;
                        path.poses.push_back(v);
                    }
                    std::lock_guard lock(mutex_);
                    if (graph_ && graph_->session_id == graph->session_id &&
                        (!committed_ || graph->revision >= committed_->revision)) {
                        // 整张图、整条路径来自同一revision。后到的新图另行重建，不混合两版历史。
                        map_pub_->publish(message(*points, graph->header));
                        path_pub_->publish(path);
                        committed_ = graph;
                        if (graph_->revision == graph->revision) {
                            dirty_ = false;
                        }
                        RCLCPP_INFO(get_logger(),
                                    "地图原子提交 revision=%lu keyframes=%zu points=%zu",
                                    graph->revision, keys.size(), points->size());
                        publish_live_locked();
                    }
                    next_build = std::chrono::steady_clock::now() +
                                 std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                     std::chrono::duration<double>(publish_interval_));
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "地图重建失败，保留上一完整版本: %s", e.what());
                    next_build = std::chrono::steady_clock::now() + std::chrono::seconds(2);
                }
            }
        }
    }

    void process_live(const Live& live) {
        try {
            const int64_t stamp = ns(live.header.stamp);
            std::string tag;
            {
                std::lock_guard lock(mutex_);
                auto it = tags_.find(stamp);
                if (it != tags_.end()) {
                    tag = it->second;
                }
                if (ambiguous_.count(stamp) || (!tag.empty() && tag != source_)) {
                    pop_live_locked(live);
                    return;
                }
            }
            if (tag.empty()) {
                throw std::runtime_error("尚无精确会话标签");
            }
            if (live.header.frame_id != body_) {
                throw std::invalid_argument("不是配置机体系的位姿");
            }
            const Pose raw = odometry_.lookup(stamp);
            std::lock_guard lock(mutex_);
            pop_live_locked(live);
            if (tag != source_) {
                return;
            }
            if (!latest_live_header_ || stamp > ns(latest_live_header_->stamp)) {
                latest_live_header_ = live.header;
                latest_live_pose_ = raw;
                publish_live_locked();
            }
        } catch (const std::exception& e) {
            const auto age =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - live.arrival)
                    .count();
            if (age >= odometry_wait_) {
                std::lock_guard lock(mutex_);
                pop_live_locked(live);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "实时位姿丢弃: %s",
                                     e.what());
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    }

    void pop_live_locked(const Live& v) {
        if (!live_queue_.empty() && live_queue_.front().arrival == v.arrival) {
            live_queue_.pop_front();
        }
    }

    // 调用者持 mutex_；修正与可视化地图来自同一 committed_ 版本。
    void publish_live_locked() {
        if (!committed_ || committed_->poses.empty() || !latest_live_header_ || !latest_live_pose_) {
            return;
        }
        const auto stamp = ns(latest_live_header_->stamp);
        const auto& tail = committed_->poses.back();
        if (stamp < ns(tail.stamp) || stamp < last_tf_stamp_) {
            return;
        }
        const Pose correction = pose(tail.map_pose) * pose(tail.odom_pose).inverse();
        geometry_msgs::msg::TransformStamped tf;
        tf.header = *latest_live_header_;
        tf.header.frame_id = map_;
        tf.child_frame_id = body_;
        tf.transform = transform(correction * *latest_live_pose_);
        // T_MB=T_MO*T_OB；实际位姿stamp，不依赖odom动态TF，不伪造now。
        broadcaster_.sendTransform(tf);
        last_tf_stamp_ = stamp;

    }

    // 冻结完整版本；局部云与元数据写完后，一次性重命名目录。
    std::string save(bool require_committed = true) {
        msg::LoopUpdate::ConstSharedPtr graph;
        std::vector<Stored> keys;
        {
            std::lock_guard lock(mutex_);
            if (!graph_ || graph_->poses.empty() ||
                (require_committed && (!committed_ ||
                 committed_->revision != graph_->revision ||
                 committed_->session_id != graph_->session_id))) {
                throw std::runtime_error("地图仍在等待关键帧/异步重建，请完成后重试");
            }
            graph = graph_;
            for (size_t i = 0; i < graph->poses.size(); ++i) {
                auto it = keys_.find(i);
                if (it == keys_.end()) {
                    throw std::runtime_error("关键帧缺失，禁止保存不完整地图");
                }
                keys.push_back(it->second);
            }
        }
        auto points = build(*graph, keys);
        const auto directory =
            std::filesystem::path(save_directory_) /
            (graph->session_id + "_r" + std::to_string(graph->revision) + "_" + unique_id());
        const auto temporary = std::filesystem::path(directory.string() + ".tmp");
        std::filesystem::create_directories(temporary / "keyframes");
        try {
            if (pcl::io::savePCDFileBinaryCompressed((temporary / "map.pcd").string(), *points) !=
                0) {
                throw std::runtime_error("map.pcd写入失败");
            }
            std::ostringstream csv;
            csv << std::setprecision(17)
                << "id,stamp_ns,odom_x,odom_y,odom_z,odom_qx,odom_qy,odom_qz,odom_qw,map_x,map_y,"
                   "map_z,map_"
                   "qx,map_qy,map_qz,map_qw\n";
            auto append = [&](const geometry_msgs::msg::Pose& p) {
                csv << ',' << p.position.x << ',' << p.position.y << ',' << p.position.z << ','
                    << p.orientation.x << ',' << p.orientation.y << ',' << p.orientation.z << ','
                    << p.orientation.w;
            };
            for (size_t i = 0; i < keys.size(); ++i) {
                const auto& p = graph->poses[i];
                csv << i << ',' << ns(p.stamp);
                append(p.odom_pose);
                append(p.map_pose);
                csv << '\n';
                if (pcl::io::savePCDFileBinaryCompressed(
                        (temporary / "keyframes" / (std::to_string(i) + ".pcd")).string(),
                        *keys[i].local) != 0) {
                    throw std::runtime_error("局部PCD写入失败");
                }
            }
            write_text(temporary / "poses.csv", csv.str());
            std::ostringstream loops;
            loops << std::setprecision(17)
                  << "from_id,to_id,accepted_revision,x,y,z,qx,qy,qz,qw,scan_context_distance,"
                     "registration_"
                     "rmse,overlap\n";
            for (const auto& edge : graph->constraints) {
                loops << edge.from_id << ',' << edge.to_id << ',' << edge.accepted_revision;
                const auto& p = edge.relative_pose;
                loops << ',' << p.position.x << ',' << p.position.y << ',' << p.position.z << ','
                      << p.orientation.x << ',' << p.orientation.y << ',' << p.orientation.z << ','
                      << p.orientation.w;
                loops << ',' << edge.scan_context_distance << ',' << edge.registration_rmse << ','
                      << edge.overlap << '\n';
            }
            write_text(temporary / "loops.csv", loops.str());
            std::ostringstream meta;
            meta << "format: ly_locate_keyframes_v1\nsession_id: " << graph->session_id
                 << "\nsource_session_id: " << graph->source_session_id
                 << "\nrevision: " << graph->revision << "\nmap_frame: " << map_
                 << "\nodom_frame: " << odom_ << "\nbody_frame: " << body_
                 << "\nkeyframes: " << keys.size() << "\nmap_leaf: " << leaf_ << "\n";
            write_text(temporary / "manifest.yaml", meta.str());
            std::filesystem::rename(temporary, directory);
        } catch (...) {
            std::filesystem::remove_all(temporary);
            throw;
        }
        return "完整版本已保存: " + directory.string();
    }

    // 固定配置。
    std::string odom_;
    std::string body_;
    std::string map_;
    std::string source_;
    std::string save_directory_;
    bool save_on_shutdown_;
    double leaf_;
    double publish_interval_;
    double odometry_wait_;
    int queue_limit_;
    OdometryBuffer odometry_;
    tf2_ros::TransformBroadcaster broadcaster_;

    // 队列、会话和版本状态由 mutex_ 保护；重建时复制快照后释放锁。
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    bool stop_ = false;
    bool dirty_ = false;
    bool fetch_pending_ = false;
    int64_t fetch_request_id_ = 0;
    std::chrono::steady_clock::time_point fetch_started_;
    // graph_ 是最新接收版本，committed_ 是完整发布版本，允许暂时不同。
    msg::LoopUpdate::ConstSharedPtr graph_;
    msg::LoopUpdate::ConstSharedPtr committed_;
    std::map<uint64_t, Stored> keys_;
    std::map<uint64_t, msg::Keyframe::ConstSharedPtr> staged_;
    std::map<std::string, std::map<uint64_t, msg::Keyframe::ConstSharedPtr>> staged_sessions_;
    std::set<std::string> retired_sources_;
    std::set<std::string> retired_graphs_;
    std::map<int64_t, std::string> tags_;
    std::set<int64_t> ambiguous_;
    std::deque<Live> live_queue_;
    std::optional<Pose> latest_live_pose_;
    std::optional<std_msgs::msg::Header> latest_live_header_;
    int64_t last_tf_stamp_ = -1;
    rclcpp::Subscription<msg::LoopUpdate>::SharedPtr update_sub_;
    rclcpp::Subscription<msg::Keyframe>::SharedPtr keyframe_sub_;
    rclcpp::Subscription<ly_lio::msg::ScanSession>::SharedPtr session_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Client<srv::GetKeyframe>::SharedPtr fetch_;
    rclcpp::TimerBase::SharedPtr fetch_timer_;
    rclcpp::CallbackGroup::SharedPtr save_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_;
};
}  // namespace ly_locate

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto n = std::make_shared<ly_locate::MapNode>();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(n);
        executor.spin();
        if (!n->save_before_exit()) {
            rclcpp::shutdown();
            return 1;
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("location_map"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
