/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */



#include "ly_lio_node.hpp"
#include "ly_lio/body_output.h"
#include "io/pcd_io.h"
#include "lidar_adapter/custom_mid360_driver.h"
#include "lidar_adapter/livox_custom_msg.h"
#include "lidar_adapter/livox_pointcloud2.h"
#include "lidar_adapter/unitree_lidar.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>

namespace ly_lio {
static builtin_interfaces::msg::Time ros_time(double seconds) {
    // 四舍五入到整纳秒再拆分，避免浮点尾数导致 nanosec==1e9 的非法时间。
    return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)));
}
LyLioNode::LyLioNode(const rclcpp::NodeOptions &options) : Node("ly_lio", options) {
    const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/livox/imu");
    const auto lidar_type = declare_parameter<std::string>("lidar_type", "livox_pointcloud2");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "baselink");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "livox_frame");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "livox_frame");
    const bool publish_sensor_tf = declare_parameter<bool>("publish_sensor_tf", true);
    const auto body_path = declare_parameter<std::string>("body_extrinsic_path", "");
    const auto body_rotation = declare_parameter<std::vector<double>>("imu_to_body_R", std::vector<double>{});
    const auto body_translation = declare_parameter<std::vector<double>>("imu_to_body_T", std::vector<double>{});
    if (!body_path.empty()) {
        if (!body_rotation.empty() || !body_translation.empty()) {
            throw std::invalid_argument("机体外参文件和 imu_to_body_R/T 只能选择一种来源");
        }
        const auto config = YAML::LoadFile(body_path);
        const auto rotation = config["lidar_to_gimbal_rotation"];
        const auto translation = config["lidar_to_gimbal_translation"];
        if (!rotation.IsSequence() || rotation.size() != 3 ||
            !translation.IsSequence() || translation.size() != 3) {
            throw std::invalid_argument("机体外参文件需要 3x3 旋转与 3 元平移");
        }
        // 文件给出 T_BL；使用已配置的 T_IL 得到 T_BI = T_BL * inverse(T_IL)。
        // 因此雷达/IMU 原点不同也可由 extrinsic_R/T 明确描述，不能重复施加安装外参。
        Eigen::Matrix3d R_BL;
        Eigen::Vector3d t_BL;
        for (size_t row = 0; row < 3; ++row) {
            if (!rotation[row].IsSequence() || rotation[row].size() != 3) {
                throw std::invalid_argument("机体外参旋转每行必须有 3 个元素");
            }
            for (size_t col = 0; col < 3; ++col) { R_BL(row, col) = rotation[row][col].as<double>(); }
            t_BL(row) = translation[row].as<double>();
        }
        // 先声明滤波参数，读取经过校验的 lidar→IMU 标定，再组合固定安装关系。
        lio_ = std::make_unique<LyLio>(*this);
        const auto &initial = lio_->current_state();
        if (get_parameter("extrinsic_est_en").as_bool()) {
            throw std::invalid_argument("使用 lidar→机体文件推导 IMU 安装外参时需固定 extrinsic_R/T；在线标定请直接配置 imu_to_body_R/T");
        }
        R_BI_ = R_BL * initial.offset_R_L_I.transpose();
        t_BI_ = t_BL - R_BI_ * initial.offset_T_L_I;
    } else if (!body_rotation.empty() || !body_translation.empty()) {
        if (body_rotation.size() != 9 || body_translation.size() != 3) {
            throw std::invalid_argument("imu_to_body_R/T 的长度必须分别为 9/3");
        }
        R_BI_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(body_rotation.data());
        t_BI_ = Eigen::Map<const Eigen::Vector3d>(body_translation.data());
    } else if (imu_frame_ != body_frame_) {
        throw std::invalid_argument("imu_frame 与 body_frame 不同时必须配置 body_extrinsic_path 或 imu_to_body_R/T；launch 可传 body_config_path:=外参文件");
    }
    if (!R_BI_.allFinite() || !t_BI_.allFinite() ||
        !(R_BI_ * R_BI_.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-5) ||
        std::abs(R_BI_.determinant() - 1.0) > 1e-5 ||
        (imu_frame_ == body_frame_ &&
         (!R_BI_.isApprox(Eigen::Matrix3d::Identity(), 1e-5) || t_BI_.norm() > 1e-9))) {
        throw std::invalid_argument("IMU→机体外参必须是有限刚体变换；同名坐标系必须使用单位外参");
    }
    R_BI_ = Eigen::Quaterniond(R_BI_).normalized().toRotationMatrix();
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_path_ = declare_parameter<bool>("publish_path", true);
    path_max_poses_ = declare_parameter<int>("path_max_poses", 10000);
    path_publish_interval_ = declare_parameter<double>("path_publish_interval", 0.1);
    const bool save_pcd = declare_parameter<bool>("save_pcd", false);
    map_path_ = declare_parameter<std::string>("map_path", "maps/ly_lio.pcd");
    const double save_resolution = declare_parameter<double>("save_map_resolution", 0.02);
    save_resolution_ = save_resolution;
    const auto mode = declare_parameter<std::string>("timestamp_mode",
        lidar_type == "custom_mid360_driver" ? "absolute" : "relative");
    const double scale = declare_parameter<double>("timestamp_scale", lidar_type == "livox_pointcloud2" ? 1e-9 : 1.0);
    if (mode != "absolute" && mode != "relative") { throw std::invalid_argument("timestamp_mode 必须为 relative 或 absolute"); }
    if (!std::isfinite(scale) || scale <= 0 || !std::isfinite(save_resolution) || save_resolution <= 0 ||
        path_max_poses_ < 1 || !std::isfinite(path_publish_interval_) || path_publish_interval_ < 0 ||
        odom_frame_.empty() || body_frame_.empty() || imu_frame_.empty() || lidar_frame_.empty() ||
        odom_frame_ == body_frame_ || odom_frame_ == imu_frame_ || map_path_.empty()) {
        throw std::invalid_argument("帧名、时间缩放或发布/地图参数非法");
    }
    if (!lio_) { lio_ = std::make_unique<LyLio>(*this); }
    const bool terminal_enabled = declare_parameter<bool>("terminal.enabled", true);
    const bool terminal_clear = declare_parameter<bool>("terminal.clear_screen", true);
    const double terminal_rate = declare_parameter<double>("terminal.refresh_rate_hz", 10.0);
    const double terminal_timeout = declare_parameter<double>("terminal.stale_timeout", 1.0);
    if (!std::isfinite(terminal_rate) || terminal_rate <= 0 || terminal_rate > 10 ||
        !std::isfinite(terminal_timeout) || terminal_timeout <= 0) {
        throw std::invalid_argument("terminal.refresh_rate_hz必须在(0,10]，stale_timeout必须为正秒数");
    }
    if (terminal_enabled) {
        terminal_ = std::make_unique<TerminalDashboard>(terminal_clear, terminal_timeout,
            odom_frame_, body_frame_);
        terminal_->update(lio_->diagnostics());
        terminal_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // 与滤波组分开：终端写入即使阻塞，也不持有滤波回调或快照的锁。
        terminal_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / terminal_rate),
            [this]() { terminal_->print(); }, terminal_group_);
    }
    // 回放或积压恢复时可能连续处理多帧，保留足够的发布队列深度。
    const auto output_qos = rclcpp::QoS(100).reliable();
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("ly_lio/odometry", output_qos);
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("ly_lio/cloud_registered", output_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("ly_lio/path", rclcpp::QoS(10).reliable());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (publish_tf_ && publish_sensor_tf && body_frame_ != imu_frame_) {
        sensor_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = body_frame_;
        tf.child_frame_id = imu_frame_;
        tf.transform.translation.x = t_BI_.x();
        tf.transform.translation.y = t_BI_.y();
        tf.transform.translation.z = t_BI_.z();
        const Eigen::Quaterniond q(R_BI_);
        tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
        sensor_tf_broadcaster_->sendTransform(tf);
    }
    if (save_pcd) { saved_map_ = std::make_unique<util::PointcloudMapping>(save_resolution); }
    save_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    save_service_ = create_service<std_srvs::srv::Trigger>("ly_lio/map_save",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            std::vector<Eigen::Vector3f> points;
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (!saved_map_) { response->message = "save_pcd 未启用"; return; }
                points = saved_map_->get_points();
            }
            if (points.empty()) { response->message = "地图为空，尚未有完成的扫描"; return; }
            try {
                const auto file = std::filesystem::path(map_path_);
                if (!file.parent_path().empty()) { std::filesystem::create_directories(file.parent_path()); }
                // 服务返回成功时文件已写完；不使用无法跟踪错误的 detached 线程。
                response->success = io::pcd::write_pcd(file.string(), points,
                    io::pcd::WritePointCloudOption(io::pcd::WritePointCloudOption::IsAscii::Binary,
                        io::pcd::WritePointCloudOption::Compressed::Compressed));
                response->message = response->success ? "地图已保存：" + file.string() : "PCD 写入失败：" + file.string();
            } catch (const std::exception &error) { response->message = error.what(); }
        }, rmw_qos_profile_services_default, save_group_);
    reset_service_ = create_service<std_srvs::srv::Trigger>("ly_lio/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            lio_->reset();
            clear_output_history();
            response->success = true;
            response->message = "滤波状态、队列、地图与轨迹已重置";
        });
    lio_->set_odometry_callback([this](const common::Odometry &odom) { publish_odometry(odom); });
    lio_->set_pointcloud_callback([this](double stamp, const auto &points) { publish_cloud(stamp, points); });
    if (lidar_type == "livox_pointcloud2") { lidar_ = std::make_unique<LivoxPointCloud2Adapter>(); }
    else if (lidar_type == "custom_mid360_driver") { lidar_ = std::make_unique<CustomMid360DriverAdapter>(); }
    else if (lidar_type == "unilidar") { lidar_ = std::make_unique<UnilidarAdapter>(); }
    else if (lidar_type == "livox_custom_msg") {
#ifdef HAVE_LIVOX_DRIVER
        lidar_ = std::make_unique<LivoxCustomMsgAdapter>();
#else
        throw std::invalid_argument("自定义 Livox 消息未启用：先 source 驱动安装目录并重新编译 ly_lio");
#endif
    } else { throw std::invalid_argument("不支持的 lidar_type：" + lidar_type); }
    lidar_->setup_subscription(this, lidar_topic, [this](const auto &points) {
        lio_->on_point_cloud_callback(points);
        if (terminal_) { terminal_->lidar_received(lio_->diagnostics().latest_lidar_time); }
        process_pending();
    });
    if (get_parameter("imu_en").as_bool()) {
        // 点云处理期间暂存200Hz IMU；5条默认深度仅能覆盖25ms。
        imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS().keep_last(1000),
            [this](const sensor_msgs::msg::Imu &msg) {
                if (!msg.header.frame_id.empty() && msg.header.frame_id != imu_frame_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                        "IMU frame=%s 与 imu_frame=%s 不一致，拒绝不匹配的测量轴", msg.header.frame_id.c_str(), imu_frame_.c_str());
                    return;
                }
                common::ImuMsg imu;
                imu.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                imu.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                imu.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                lio_->on_imu_callback(imu);
                if (terminal_ && imu.angular_velocity.allFinite() && imu.linear_acceleration.allFinite()) {
                    terminal_->imu_received(imu.timestamp);
                }
                process_pending();
            });
    }
    RCLCPP_INFO(get_logger(), "接收原始 %s 和 %s；状态原点=%s；输出 %s→%s；逐点时间模式=%s，缩放=%g",
        lidar_topic.c_str(), imu_topic.c_str(), imu_frame_.c_str(), odom_frame_.c_str(), body_frame_.c_str(), mode.c_str(), scale);
}

void LyLioNode::clear_output_history() {
    path_.poses.clear();
    last_tf_time_ = last_path_time_ = -1;
    if (terminal_) {
        terminal_->reset_pose();
        terminal_->update(lio_->diagnostics());
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (saved_map_) { saved_map_ = std::make_unique<util::PointcloudMapping>(save_resolution_); }
}

void LyLioNode::process_pending() {
    const bool initialized_before = lio_->initialized();
    lio_->handle_once();
    if (initialized_before && !lio_->initialized()) {
        // IMU断流或队列溢出后重建odom原点，不能将旧Path/导出地图混入新轨迹。
        clear_output_history();
        RCLCPP_WARN(get_logger(), "传感器时序中断，已清空轨迹和导出地图，等待重新静止初始化");
    } else if (!initialized_before && lio_->initialized()) {
        RCLCPP_INFO(get_logger(), "静止标定和初始地图完成，开始逐点 Point-LIO 更新");
    }
    if (terminal_) { terminal_->update(lio_->diagnostics()); }
}

void LyLioNode::publish_odometry(const common::Odometry &imu_odometry) {
    // T_WB = T_WI * inverse(T_BI)，地图仍在滤波器建立的 odom 中，无需再变换。
    const auto odometry = body_odometry(imu_odometry.timestamp, lio_->current_state(),
        lio_->covariance(), R_BI_, t_BI_);
    nav_msgs::msg::Odometry msg;
    // stamp 是此位姿对应的传感器采样时刻，不能改为回调的 now() 掩盖输入批次延迟。
    msg.header.stamp = ros_time(odometry.timestamp);
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = body_frame_;
    msg.pose.pose.position.x = odometry.position.x();
    msg.pose.pose.position.y = odometry.position.y();
    msg.pose.pose.position.z = odometry.position.z();
    msg.pose.pose.orientation.x = odometry.orientation.x();
    msg.pose.pose.orientation.y = odometry.orientation.y();
    msg.pose.pose.orientation.z = odometry.orientation.z();
    msg.pose.pose.orientation.w = odometry.orientation.w();
    const Eigen::Vector3d velocity_body = odometry.orientation.conjugate() * odometry.velocity;
    msg.twist.twist.linear.x = velocity_body.x();
    msg.twist.twist.linear.y = velocity_body.y();
    msg.twist.twist.linear.z = velocity_body.z();
    msg.twist.twist.angular.x = odometry.angular_velocity.x();
    msg.twist.twist.angular.y = odometry.angular_velocity.y();
    msg.twist.twist.angular.z = odometry.angular_velocity.z();
    for (size_t row = 0; row < 6; ++row) {
        for (size_t col = 0; col < 6; ++col) {
            msg.pose.covariance[row * 6 + col] = odometry.pose_covariance(row, col);
            msg.twist.covariance[row * 6 + col] = odometry.twist_covariance(row, col);
        }
    }
    odometry_publisher_->publish(msg);
    const bool tf_sent = publish_tf_ && odometry.timestamp > last_tf_time_;
    if (tf_sent) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg.header;
        tf.child_frame_id = body_frame_;
        tf.transform.translation.x = odometry.position.x();
        tf.transform.translation.y = odometry.position.y();
        tf.transform.translation.z = odometry.position.z();
        tf.transform.rotation = msg.pose.pose.orientation;
        // TF 与里程计都使用转换后的 baselink 位姿，安装外参只组合一次。
        tf_broadcaster_->sendTransform(tf);
        last_tf_time_ = odometry.timestamp;
    }
    if (terminal_) {
        // 面板使用与已发布机体位姿同一时刻的状态，偏置只旋转表达轴，不伪造原点加速度。
        terminal_->odometry_published(odometry, tf_sent, lio_->current_state(), R_BI_);
    }
    if (publish_path_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose = msg.pose.pose;
        path_.header = msg.header;
        path_.poses.push_back(pose);
        if (path_.poses.size() > static_cast<size_t>(path_max_poses_)) {
            path_.poses.erase(path_.poses.begin());
        }
        // 每次发布的位姿都保留在Path内，仅降低重复整条历史的发送频率。
        // Path独立限频；逐点发布开启时也避免反复序列化完整轨迹挤占处理时间。
        if (last_path_time_ < 0 || odometry.timestamp - last_path_time_ >= path_publish_interval_) {
            path_publisher_->publish(path_);
            last_path_time_ = odometry.timestamp;
        }
    }
}

void LyLioNode::publish_cloud(double timestamp, const std::vector<Eigen::Vector3f> &points) {
    if (cloud_publisher_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = ros_time(timestamp);
        msg.header.frame_id = odom_frame_;
        msg.width = static_cast<uint32_t>(points.size());
        msg.height = 1;
        msg.point_step = 12;
        msg.row_step = msg.width * msg.point_step;
        msg.is_dense = true;
        const uint16_t native = 1;
        msg.is_bigendian = *reinterpret_cast<const uint8_t *>(&native) == 0;
        for (size_t axis = 0; axis < 3; ++axis) {
            sensor_msgs::msg::PointField field;
            field.name = std::array<std::string, 3>{"x", "y", "z"}[axis];
            field.offset = axis * sizeof(float);
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            msg.fields.push_back(field);
        }
        msg.data.resize(msg.row_step);
        for (size_t index = 0; index < points.size(); ++index) {
            // 点已在 odom 中；再次施加 lidar→机体外参会使地图和里程计不一致。
            std::memcpy(msg.data.data() + index * msg.point_step, points[index].data(), 3 * sizeof(float));
        }
        cloud_publisher_->publish(msg);
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (saved_map_) { saved_map_->add_pointcloud(points); }
}
}  // namespace ly_lio
RCLCPP_COMPONENTS_REGISTER_NODE(ly_lio::LyLioNode)
