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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <rcl/arguments.h>
#include <rcl/remap.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <string_view>
#include <yaml-cpp/yaml.h>

namespace ly_lio {
namespace {
// 从已定位的包目录向上找源码，不依赖当前工作目录或编译机器的绝对路径。
std::filesystem::path source_file_near(std::filesystem::path directory,
    const std::vector<std::filesystem::path> &relative_paths) {
    while (!directory.empty()) {
        for (const auto &relative : relative_paths) {
            std::error_code error;
            const auto candidate = directory / relative;
            if (std::filesystem::is_regular_file(candidate, error)) { return candidate; }
        }
        const auto parent = directory.parent_path();
        if (parent == directory) { break; }
        directory = parent;
    }
    return {};
}

std::filesystem::path default_config_file() {
    const auto share = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("ly_lio"));
    const auto source = source_file_near(share,
        {"src/ly_lio/config/mid360.yaml", "odom_ws/src/ly_lio/config/mid360.yaml"});
    // 源码存在时直接读取可编辑版本；独立安装部署则读取 share 中的版本。
    return source.empty() ? share / "config/mid360.yaml" : source;
}

std::string node_fqn(const rclcpp::NodeOptions &options,
    const rcl_arguments_t *global_arguments) {
    const auto *local_arguments = &options.get_rcl_node_options()->arguments;
    const auto allocator = options.get_rcl_node_options()->allocator;
    const auto free_string = [allocator](char *value) {
        if (value) { allocator.deallocate(value, allocator.state); }
    };
    char *raw_name = nullptr;
    const auto name_result = rcl_remap_node_name(local_arguments, global_arguments,
        "ly_lio", allocator, &raw_name);
    const std::unique_ptr<char, decltype(free_string)> remapped_name(raw_name, free_string);
    if (name_result != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(name_result, "无法解析节点名称重映射");
    }
    const std::string name = remapped_name ? remapped_name.get() : "ly_lio";
    char *raw_namespace = nullptr;
    const auto namespace_result = rcl_remap_node_namespace(local_arguments, global_arguments,
        name.c_str(), allocator, &raw_namespace);
    const std::unique_ptr<char, decltype(free_string)> remapped_namespace(raw_namespace, free_string);
    if (namespace_result != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(namespace_result, "无法解析节点命名空间重映射");
    }
    std::string namespace_name = remapped_namespace ? remapped_namespace.get() : "/";
    if (namespace_name.empty() || namespace_name.front() != '/') { namespace_name.insert(0, "/"); }
    if (namespace_name.back() != '/') { namespace_name += '/'; }
    return namespace_name + name;
}

rclcpp::NodeOptions with_default_configuration(const rclcpp::NodeOptions &options) {
    // 用 ROS 原生 YAML 解析器读取默认参数；不在 shell/launch 中解析或生成参数字典。
    // Humble 的 parameter_map_from_yaml_file 未释放原生解析对象，此处用 RAII 完整回收。
    const std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> parsed_defaults(
        rcl_yaml_node_struct_init(options.get_rcl_node_options()->allocator), rcl_yaml_node_struct_fini);
    if (!parsed_defaults) { throw std::bad_alloc(); }
    if (!rcl_parse_yaml_file(default_config_file().c_str(), parsed_defaults.get())) {
        rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR, "无法读取默认 mid360.yaml");
    }
    const auto defaults = rclcpp::parameter_map_from(parsed_defaults.get());
    const auto default_node = defaults.find("/ly_lio");
    if (default_node == defaults.end()) {
        throw std::invalid_argument("默认 mid360.yaml 必须包含 ly_lio.ros__parameters");
    }
    std::map<std::string, rclcpp::Parameter> parameters;
    const auto merge = [&parameters](const auto &values) {
        for (const auto &parameter : values) { parameters.insert_or_assign(parameter.get_name(), parameter); }
    };
    merge(default_node->second);
    const auto context = options.context()->get_rcl_context();
    const auto *global_arguments = options.use_global_arguments() ? &context->global_arguments : nullptr;
    const auto full_name = node_fqn(options, global_arguments);
    const auto merge_arguments = [&](const rcl_arguments_t *arguments) {
        if (!arguments) { return; }
        rcl_params_t *raw_parameters = nullptr;
        const auto result = rcl_arguments_get_param_overrides(arguments, &raw_parameters);
        const std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> parsed(
            raw_parameters, rcl_yaml_node_struct_fini);
        if (result != RCL_RET_OK) {
            rclcpp::exceptions::throw_from_rcl_error(result, "无法解析显式 ROS 参数");
        }
        if (!parsed) { return; }
        // 原生匹配保留 /**、节点名及命名空间的参数作用域。
        for (const auto &[name, values] : rclcpp::parameter_map_from(parsed.get(), full_name.c_str())) {
            merge(values);
        }
    };
    // 与 ROS 默认优先级一致：默认 YAML < 全局参数 < 局部参数 < 组件 NodeOptions。
    // 只注入默认 YAML 会让程序参数反过来屏蔽 CLI，因此先合并全部显式来源。
    merge_arguments(global_arguments);
    merge_arguments(&options.get_rcl_node_options()->arguments);
    merge(options.parameter_overrides());
    std::vector<rclcpp::Parameter> overrides;
    overrides.reserve(parameters.size());
    for (const auto &[name, parameter] : parameters) { overrides.push_back(parameter); }
    auto configured = options;
    configured.parameter_overrides(overrides);
    return configured;
}

void log_parameter_files(const rclcpp::Logger &logger, const rcl_arguments_t *arguments) {
    if (!arguments) { return; }
    const auto count = rcl_arguments_get_param_files_count(arguments);
    if (count <= 0) { return; }
    const auto allocator = rcl_get_default_allocator();
    char **raw_files = nullptr;
    const auto result = rcl_arguments_get_param_files(arguments, allocator, &raw_files);
    const auto free_files = [allocator, count](char **files) {
        if (!files) { return; }
        for (int index = 0; index < count; ++index) { allocator.deallocate(files[index], allocator.state); }
        allocator.deallocate(files, allocator.state);
    };
    const std::unique_ptr<char *, decltype(free_files)> files(raw_files, free_files);
    if (result != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(result, "无法读取显式参数文件路径");
    }
    for (int index = 0; index < count; ++index) {
        RCLCPP_INFO(logger, "显式 ROS 参数文件（覆盖默认值）：%s", files.get()[index]);
    }
}

std::string body_extrinsic_file(const std::string &configured_path) {
    constexpr std::string_view prefix = "package://";
    if (!configured_path.starts_with(prefix)) { return configured_path; }
    const auto package_end = configured_path.find('/', prefix.size());
    if (package_end == std::string::npos || package_end == prefix.size()) {
        throw std::invalid_argument("外参 package:// 路径需要包名和包内文件名");
    }
    const auto package = configured_path.substr(prefix.size(), package_end - prefix.size());
    const auto relative = std::filesystem::path(configured_path.substr(package_end + 1));
    if (relative.empty() || relative.is_absolute()) {
        throw std::invalid_argument("外参 package:// 路径需要相对包目录的文件名");
    }
    for (const auto &part : relative) {
        if (part == "..") { throw std::invalid_argument("外参 package:// 路径不能跳出包目录"); }
    }
    if (package == "pcl_publisher" && relative == "config/lidar2baselink.yaml") {
        // 两个工作区共用同一个物理安装文件，节点直接读取，不生成临时副本。
        const auto source = source_file_near(default_config_file().parent_path(),
            {"pcl_ws/config/lidar2baselink.yaml"});
        if (!source.empty()) { return source.string(); }
        const auto share = std::filesystem::path(ament_index_cpp::get_package_share_directory(package));
        const auto package_source = source_file_near(share, {"pcl_ws/config/lidar2baselink.yaml"});
        return (package_source.empty() ? share / relative : package_source).string();
    }
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory(package)) / relative).string();
}
}  // namespace

static builtin_interfaces::msg::Time ros_time(double seconds) {
    // 四舍五入到整纳秒再拆分，避免浮点尾数导致 nanosec==1e9 的非法时间。
    return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)));
}
LyLioNode::LyLioNode(const rclcpp::NodeOptions &options)
    : Node("ly_lio", with_default_configuration(options)) {
    RCLCPP_INFO(get_logger(), "默认 ROS 参数文件：%s", default_config_file().c_str());
    if (options.use_global_arguments()) {
        log_parameter_files(get_logger(), &options.context()->get_rcl_context()->global_arguments);
    }
    log_parameter_files(get_logger(), &options.get_rcl_node_options()->arguments);
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
        const auto body_file = body_extrinsic_file(body_path);
        RCLCPP_INFO(get_logger(), "直接读取固定安装外参：%s", body_file.c_str());
        const auto config = YAML::LoadFile(body_file);
        const auto rotation = config["lidar_to_gimbal_rotation"];
        const auto translation = config["lidar_to_gimbal_translation"];
        if (!rotation.IsSequence() || rotation.size() != 3 ||
            !translation.IsSequence() || translation.size() != 3) {
            throw std::invalid_argument("机体外参文件需要 3x3 旋转与 3 元平移");
        }
        // 文件给出 T_BL；使用已配置的 T_IL 得到 T_BI = T_BL * inverse(T_IL)。
        // 因此雷达/IMU 原点不同也可由 extrinsic_R/T 明确描述，不能重复施加安装外参。
        body_from_lidar_mount_ = true;
        for (size_t row = 0; row < 3; ++row) {
            if (!rotation[row].IsSequence() || rotation[row].size() != 3) {
                throw std::invalid_argument("机体外参旋转每行必须有 3 个元素");
            }
            for (size_t col = 0; col < 3; ++col) { R_BL_(row, col) = rotation[row][col].as<double>(); }
            t_BL_(row) = translation[row].as<double>();
        }
        if (!R_BL_.allFinite() || !t_BL_.allFinite() ||
            !(R_BL_ * R_BL_.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-5) ||
            std::abs(R_BL_.determinant() - 1.0) > 1e-5) {
            throw std::invalid_argument("激光→机体安装外参必须为有限刚体变换");
        }
        R_BL_ = Eigen::Quaterniond(R_BL_).normalized().toRotationMatrix();
        // 先声明滤波参数，读取经过校验的 lidar→IMU 标定，再组合固定安装关系。
        lio_ = std::make_unique<LyLio>(*this);
        R_BI_ = R_BL_ * lio_->lidar_to_imu_rotation().transpose();
        t_BI_ = t_BL_ - R_BI_ * lio_->lidar_to_imu_translation();
    } else if (!body_rotation.empty() || !body_translation.empty()) {
        if (body_rotation.size() != 9 || body_translation.size() != 3) {
            throw std::invalid_argument("imu_to_body_R/T 的长度必须分别为 9/3");
        }
        R_BI_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(body_rotation.data());
        t_BI_ = Eigen::Map<const Eigen::Vector3d>(body_translation.data());
    } else if (imu_frame_ != body_frame_) {
        throw std::invalid_argument("imu_frame 与 body_frame 不同时必须配置 body_extrinsic_path 或 imu_to_body_R/T");
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
        odom_frame_ == body_frame_ || odom_frame_ == imu_frame_ || odom_frame_ == lidar_frame_ || map_path_.empty()) {
        throw std::invalid_argument("帧名、时间缩放或发布/地图参数非法");
    }
    if (!lio_) { lio_ = std::make_unique<LyLio>(*this); }
    if (!body_from_lidar_mount_) {
        R_BL_ = R_BI_ * lio_->lidar_to_imu_rotation();
        t_BL_ = t_BI_ + R_BI_ * lio_->lidar_to_imu_translation();
    }
    if (body_frame_ == lidar_frame_ &&
        (!R_BL_.isApprox(Eigen::Matrix3d::Identity(), 1e-5) || t_BL_.norm() > 1e-9 ||
         (!body_from_lidar_mount_ && lio_->estimates_extrinsics()))) {
        throw std::invalid_argument("body_frame与lidar_frame同名时必须是同一固定原点与测量轴");
    }
    RCLCPP_INFO(get_logger(), "Point-LIO状态/P: %d维，外参%s；内部t_IL=[%.8f, %.8f, %.8f]m",
        state::DIM, lio_->estimates_extrinsics() ? "在线估计" : "固定",
        lio_->lidar_to_imu_translation().x(), lio_->lidar_to_imu_translation().y(),
        lio_->lidar_to_imu_translation().z());
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
    publish_dynamic_sensor_tf_ = publish_tf_ && publish_sensor_tf && body_frame_ != lidar_frame_ &&
        !body_from_lidar_mount_ && lio_->estimates_extrinsics();
    if (publish_tf_ && publish_sensor_tf && body_frame_ != lidar_frame_ && !publish_dynamic_sensor_tf_) {
        sensor_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = body_frame_;
        // 原始IMU的frame_id只表示测量轴；livox_frame仍是激光原点，发布真正的T_BL。
        tf.child_frame_id = lidar_frame_;
        tf.transform.translation.x = t_BL_.x();
        tf.transform.translation.y = t_BL_.y();
        tf.transform.translation.z = t_BL_.z();
        const Eigen::Quaterniond q(R_BL_);
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
        RCLCPP_INFO(get_logger(), "静止标定和初始地图完成，开始 Batch 点到地图联合更新");
    }
    if (terminal_) { terminal_->update(lio_->diagnostics()); }
}

void LyLioNode::publish_odometry(const common::Odometry &imu_odometry) {
    // T_WB = T_WI * inverse(T_BI)，地图仍在滤波器建立的 odom 中，无需再变换。
    if (body_from_lidar_mount_) {
        R_BI_ = R_BL_ * lio_->lidar_to_imu_rotation().transpose();
        t_BI_ = t_BL_ - R_BI_ * lio_->lidar_to_imu_translation();
    }
    const auto odometry = body_from_lidar_mount_ ?
        body_odometry_from_lidar_mount(imu_odometry.timestamp, lio_->current_state(), lio_->covariance(), R_BL_, t_BL_) :
        body_odometry(imu_odometry.timestamp, lio_->current_state(), lio_->covariance(), R_BI_, t_BI_);
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
        if (publish_dynamic_sensor_tf_) {
            // 显式固定T_BI时在线T_IL尚在变化，传感器关系随当前标定发布动态TF。
            geometry_msgs::msg::TransformStamped sensor_tf;
            sensor_tf.header.stamp = msg.header.stamp;
            sensor_tf.header.frame_id = body_frame_;
            sensor_tf.child_frame_id = lidar_frame_;
            const Eigen::Vector3d t = t_BI_ + R_BI_ * lio_->lidar_to_imu_translation();
            const Eigen::Quaterniond q(R_BI_ * lio_->lidar_to_imu_rotation());
            sensor_tf.transform.translation.x = t.x();
            sensor_tf.transform.translation.y = t.y();
            sensor_tf.transform.translation.z = t.z();
            sensor_tf.transform.rotation.x = q.x(); sensor_tf.transform.rotation.y = q.y();
            sensor_tf.transform.rotation.z = q.z(); sensor_tf.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(sensor_tf);
        }
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
        // Path独立限频；按批发布开启时也避免反复序列化完整轨迹挤占处理时间。
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
