// Copyright (c) 2026. Licensed under the MIT License.
#ifndef LIVOX_DRIVER_CONFIG_H_
#define LIVOX_DRIVER_CONFIG_H_

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace livox_ros {

// 由节点直接读取 ROS 参数 YAML；命令行和组件显式参数保持优先。
std::string LoadDriverParameters(rclcpp::Node& node);
std::string ResolveSdkConfigPath(const std::string& parameter_file,
                                 const std::string& sdk_config);
// JSON 只读校验始终执行；硬件检查可用于无硬件配置验证时关闭。
void CheckDriverConfiguration(const std::string& sdk_config,
                              const std::string& target_ip,
                              const std::string& interface,
                              bool require_ptp_sync, bool check_hardware);

}  // namespace livox_ros
#endif  // LIVOX_DRIVER_CONFIG_H_
