// Copyright (c) 2026. Licensed under the MIT License.
#include "driver_config.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>

extern char** environ;

namespace livox_ros {
namespace {

std::string ExistingPath(const std::string& path) {
  std::unique_ptr<char, decltype(&std::free)> absolute(
      realpath(path.c_str(), nullptr), &std::free);
  if (!absolute) { throw std::runtime_error("无法读取配置文件: " + path); }
  return absolute.get();
}

std::string DefaultParameterPath() {
  // 开发工作区重启即读取源码；部署时源码不可用才使用安装副本。
  // 从ament安装目录逐层寻找工作区，不固化编译机的绝对路径。
  const std::string share =
      ament_index_cpp::get_package_share_directory("livox_ros_driver2");
  std::string ancestor = ExistingPath(share);
  while (!ancestor.empty()) {
    for (const char* relative : {"/src/livox_ros_driver2/config/mid360.yaml",
                                 "/livox_sdk/src/livox_ros_driver2/config/mid360.yaml"}) {
      const std::string candidate = ancestor + relative;
      if (access(candidate.c_str(), R_OK) == 0) { return ExistingPath(candidate); }
    }
    const auto slash = ancestor.find_last_of('/');
    if (slash == std::string::npos || slash == 0) { break; }
    ancestor.erase(slash);
  }
  return share + "/config/mid360.yaml";
}

bool IsIpv4(const std::string& ip) {
  in_addr address{};
  return inet_pton(AF_INET, ip.c_str(), &address) == 1;
}

bool PtpServiceActive(const std::string& unit) {
  // 不经过 shell，且只查询已有服务，不启停或修改 PTP/系统时钟。
  const char* args[] = {"systemctl", "is-active", "--quiet", unit.c_str(), nullptr};
  pid_t child = -1;
  const int result = posix_spawnp(&child, args[0], nullptr, nullptr,
                                 const_cast<char**>(args), environ);
  if (result != 0) { return false; }
  int status = 0;
  pid_t waited;
  do { waited = waitpid(child, &status, 0); } while (waited < 0 && errno == EINTR);
  return waited == child && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

std::set<std::string> InterfaceAddresses(const std::string& interface) {
  ifaddrs* raw = nullptr;
  if (getifaddrs(&raw) != 0) { throw std::runtime_error("无法读取本机网卡地址"); }
  std::unique_ptr<ifaddrs, decltype(&freeifaddrs)> addresses(raw, &freeifaddrs);
  std::set<std::string> result;
  for (auto* item = raw; item; item = item->ifa_next) {
    if (!item->ifa_addr || item->ifa_addr->sa_family != AF_INET ||
        interface != item->ifa_name) { continue; }
    char ip[INET_ADDRSTRLEN]{};
    const auto* address = reinterpret_cast<const sockaddr_in*>(item->ifa_addr);
    if (inet_ntop(AF_INET, &address->sin_addr, ip, sizeof(ip))) { result.insert(ip); }
  }
  return result;
}

}  // namespace

std::string LoadDriverParameters(rclcpp::Node& node) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  if (!node.has_parameter("config_path")) {
    node.declare_parameter("config_path", DefaultParameterPath(), descriptor);
  }
  const std::string path = ExistingPath(node.get_parameter("config_path").as_string());
  std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> yaml(
      rcl_yaml_node_struct_init(rcutils_get_default_allocator()), &rcl_yaml_node_struct_fini);
  if (!yaml || !rcl_parse_yaml_file(path.c_str(), yaml.get())) {
    throw std::runtime_error("Livox ROS参数YAML解析失败: " + path);
  }
  const auto parameter_map = rclcpp::parameter_map_from(yaml.get(), node.get_fully_qualified_name());
  const auto found = parameter_map.find(node.get_fully_qualified_name());
  if (found == parameter_map.end() || found->second.empty()) {
    throw std::runtime_error("Livox参数YAML未匹配当前节点，推荐使用/**: " + path);
  }
  const std::set<std::string> supported{
      "use_sim_time", "xfer_format", "multi_topic", "data_src", "publish_freq",
      "output_data_type", "frame_id", "user_config_path", "cmdline_input_bd_code",
      "lvx_file_path", "require_ptp_sync", "ptp_utc_offset", "target_ip", "ptp_interface",
      "preflight_check", "config_check_only"};
  const auto& overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto& parameter : found->second) {
    const auto& name = parameter.get_name();
    if (!supported.count(name)) {
      throw std::runtime_error("未知Livox YAML参数: " + name);
    }
    if (!node.has_parameter(name)) {
      node.declare_parameter(name, parameter.get_parameter_value(), descriptor);
    } else if (name == "use_sim_time" && !overrides.count(name)) {
      node.set_parameter(parameter);
    }
  }
  for (const auto& name : supported) {
    if (!node.has_parameter(name)) { throw std::runtime_error("缺少Livox YAML参数: " + name); }
  }
  RCLCPP_INFO(node.get_logger(), "直接读取Livox参数: %s", path.c_str());
  return path;
}

std::string ResolveSdkConfigPath(const std::string& parameter_file,
                                 const std::string& sdk_config) {
  if (sdk_config.empty()) { throw std::runtime_error("user_config_path不能为空"); }
  if (sdk_config.front() == '/') { return ExistingPath(sdk_config); }
  return ExistingPath(parameter_file.substr(0, parameter_file.find_last_of('/') + 1) + sdk_config);
}

void CheckDriverConfiguration(const std::string& sdk_config,
                              const std::string& target_ip,
                              const std::string& interface,
                              bool require_ptp_sync, bool check_hardware) {
  if ((!target_ip.empty() && !IsIpv4(target_ip)) ||
      interface.empty() || interface.size() >= IFNAMSIZ) {
    throw std::runtime_error("target_ip或ptp_interface无效");
  }
  for (const unsigned char character : interface) {
    if (!std::isalnum(character) && character != '_' && character != '-' && character != '.') {
      throw std::runtime_error("ptp_interface网卡名称无效");
    }
  }
  std::unique_ptr<FILE, decltype(&std::fclose)> file(std::fopen(sdk_config.c_str(), "rb"), &std::fclose);
  if (!file) { throw std::runtime_error("无法打开Livox SDK JSON: " + sdk_config); }
  char buffer[8192];
  rapidjson::FileReadStream stream(file.get(), buffer, sizeof(buffer));
  rapidjson::Document json;
  if (json.ParseStream(stream).HasParseError() || !json.IsObject() ||
      !json.HasMember("lidar_configs") || !json["lidar_configs"].IsArray() ||
      json["lidar_configs"].Empty()) {
    throw std::runtime_error("Livox SDK JSON缺少合法lidar_configs: " + sdk_config);
  }
  bool target_found = target_ip.empty();
  for (const auto& lidar : json["lidar_configs"].GetArray()) {
    if (!lidar.IsObject() || !lidar.HasMember("ip") || !lidar["ip"].IsString() ||
        !IsIpv4(lidar["ip"].GetString())) {
      throw std::runtime_error("Livox SDK JSON中雷达IP无效");
    }
    if (target_ip == lidar["ip"].GetString()) { target_found = true; }
  }
  if (!target_found) {
    throw std::runtime_error("target_ip不在SDK JSON的lidar_configs中: " + target_ip);
  }

  std::set<std::string> host_ips;
  const auto collect_host = [&host_ips](const rapidjson::Value& host) {
    if (!host.IsObject()) { throw std::runtime_error("SDK host_net_info必须为对象"); }
    for (const char* key : {"cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"}) {
      if (!host.HasMember(key) || !host[key].IsString() || !IsIpv4(host[key].GetString())) {
        throw std::runtime_error(std::string("SDK主机接收IP无效: ") + key);
      }
      host_ips.insert(host[key].GetString());
    }
  };
  for (auto item = json.MemberBegin(); item != json.MemberEnd(); ++item) {
    const auto& device = item->value;
    if (!device.IsObject() || !device.HasMember("host_net_info")) { continue; }
    const auto& host = device["host_net_info"];
    if (host.IsArray()) {
      for (const auto& network : host.GetArray()) { collect_host(network); }
    } else { collect_host(host); }
  }
  if (host_ips.empty()) { throw std::runtime_error("SDK JSON缺少主机接收地址"); }
  if (!check_hardware) { return; }

  const auto addresses = InterfaceAddresses(interface);
  for (const auto& ip : host_ips) {
    if (!addresses.count(ip)) {
      throw std::runtime_error("网卡" + interface + "缺少SDK JSON要求的接收IP: " + ip);
    }
  }
  if (require_ptp_sync) {
    bool active = false;
    for (const char* prefix : {"sentry", "ly"}) {
      const std::string base = std::string(prefix) + "-";
      if (PtpServiceActive(base + "ptp4l@" + interface + ".service") &&
          PtpServiceActive(base + "phc2sys@" + interface + ".service")) {
        active = true;
        break;
      }
    }
    if (!active) { throw std::runtime_error("已有PTP/PHC服务未同时active，拒绝启动雷达"); }
  }
}

}  // namespace livox_ros
