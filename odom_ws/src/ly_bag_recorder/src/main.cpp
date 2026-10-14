#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <keyboard_handler/keyboard_handler.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
namespace
{
// Humble Recorder 无条件向 KeyboardHandler 注册回调，不能传 nullptr。
// 注入“不启用终端”的实现：不读 stdin、不修改终端，也不安装 SIGINT 处理器。
// SIGINT/SIGTERM 全部由 rclcpp 处理，退出后仍由主线程刷新原生 writer。
class InactiveKeyboardHandler final : public KeyboardHandler
{
public:
  InactiveKeyboardHandler()
  : KeyboardHandler(
      [](int, void *, size_t) -> ssize_t { return 0; },
      [](int) { return 0; },
      [](int, struct termios *) { return -1; },
      [](int, int, const struct termios *) { return -1; },
      false)
  {}
};

// 所有配置字段明确校验；拼写错误不能静默变成默认值。
void check_keys(const YAML::Node & node, std::initializer_list<const char *> names,
                const std::string & where)
{
  if (!node.IsMap()) throw std::runtime_error(where + "必须为YAML映射");
  std::set<std::string> allowed(names.begin(), names.end()), seen;
  for (const auto & entry : node) {
    if (!entry.first.IsScalar()) throw std::runtime_error(where + "字段名必须为字符串");
    const auto key = entry.first.as<std::string>();
    if (!allowed.count(key)) throw std::runtime_error(where + "未知字段：" + key);
    if (!seen.insert(key).second) throw std::runtime_error(where + "重复字段：" + key);
  }
  for (const auto & key : allowed) {
    if (!seen.count(key)) throw std::runtime_error(where + "缺少字段：" + key);
  }
}

std::string text(const YAML::Node & node, const std::string & where)
{
  if (!node.IsScalar()) throw std::runtime_error(where + "必须为字符串");
  return node.as<std::string>();
}

bool boolean(const YAML::Node & node, const std::string & where)
{
  const auto value = text(node, where);
  if (value == "true") return true;
  if (value == "false") return false;
  throw std::runtime_error(where + "必须为true或false");
}

uint64_t integer(const YAML::Node & node, const std::string & where)
{
  const auto value = text(node, where);
  if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char c) {
      return std::isdigit(c);
    })) throw std::runtime_error(where + "必须为非负整数");
  try { return node.as<uint64_t>(); }
  catch (const YAML::Exception &) { throw std::runtime_error(where + "整数超出uint64范围"); }
}

fs::path resolve(const std::string & value, const fs::path & base)
{
  fs::path path(value);
  return fs::absolute(path.is_absolute() ? path : base / path).lexically_normal();
}

std::string topic(const YAML::Node & node, const std::string & where)
{
  auto value = text(node, where);
  if (value.empty() || value.front() != '/') {
    throw std::runtime_error(where + "必须是以/开头的完整话题名");
  }
  if (rclcpp::expand_topic_or_service_name(value, "ly_bag_recorder", "/") != value) {
    throw std::runtime_error(where + "必须是无替换符的完整话题名");
  }
  return value;
}

struct Configuration
{
  fs::path output_directory;
  rosbag2_storage::StorageOptions storage;
  rosbag2_transport::RecordOptions record;
};

Configuration load_configuration(const fs::path & path, const std::string & output_override)
{
  const auto yaml = YAML::LoadFile(path.string());
  check_keys(yaml, {"output_directory", "storage", "record", "qos"}, "根配置");
  Configuration result;
  const auto output = output_override.empty() ?
    text(yaml["output_directory"], "output_directory") : output_override;
  if (output.empty()) throw std::runtime_error("output_directory不能为空");
  result.output_directory = resolve(output, path.parent_path());

  const auto storage = yaml["storage"];
  check_keys(storage, {"storage_id", "max_cache_size", "max_bagfile_size", "max_bagfile_duration",
    "storage_preset_profile", "storage_config_uri"}, "storage");
  auto & s = result.storage;
  s.storage_id = text(storage["storage_id"], "storage.storage_id");
  if (!std::regex_match(s.storage_id, std::regex("[A-Za-z_][A-Za-z0-9_]*"))) {
    throw std::runtime_error("storage.storage_id无效");
  }
  s.max_cache_size = integer(storage["max_cache_size"], "storage.max_cache_size");
  s.max_bagfile_size = integer(storage["max_bagfile_size"], "storage.max_bagfile_size");
  s.max_bagfile_duration = integer(storage["max_bagfile_duration"], "storage.max_bagfile_duration");
  if (s.max_bagfile_duration > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) / 1000000000ULL) {
    throw std::runtime_error("storage.max_bagfile_duration超出纳秒时间范围");
  }
  s.storage_preset_profile = text(storage["storage_preset_profile"], "storage.storage_preset_profile");
  const auto storage_config = text(storage["storage_config_uri"], "storage.storage_config_uri");
  if (!storage_config.empty()) {
    s.storage_config_uri = resolve(storage_config, path.parent_path()).string();
    if (!fs::is_regular_file(s.storage_config_uri)) {
      throw std::runtime_error("存储插件配置不存在：" + s.storage_config_uri);
    }
  }

  const auto record = yaml["record"];
  check_keys(record, {"all", "topics", "regex", "exclude", "include_hidden_topics",
    "include_unpublished_topics", "is_discovery_disabled", "topic_polling_interval_ms",
    "start_paused", "use_sim_time"}, "record");
  auto & r = result.record;
  // 与ros2 bag record一致，直接保存当前RMW原生序列化格式，避免格式转换。
  r.rmw_serialization_format = rmw_get_serialization_format();
  r.all = boolean(record["all"], "record.all");
  r.include_hidden_topics = boolean(record["include_hidden_topics"], "record.include_hidden_topics");
  r.include_unpublished_topics = boolean(record["include_unpublished_topics"], "record.include_unpublished_topics");
  r.is_discovery_disabled = boolean(record["is_discovery_disabled"], "record.is_discovery_disabled");
  r.start_paused = boolean(record["start_paused"], "record.start_paused");
  r.use_sim_time = boolean(record["use_sim_time"], "record.use_sim_time");
  const auto interval = integer(record["topic_polling_interval_ms"], "record.topic_polling_interval_ms");
  if (!interval || interval > static_cast<uint64_t>(std::chrono::milliseconds::max().count())) {
    throw std::runtime_error("record.topic_polling_interval_ms必须为可表示的正毫秒数");
  }
  r.topic_polling_interval = std::chrono::milliseconds(interval);
  if (!record["topics"].IsSequence()) throw std::runtime_error("record.topics必须为列表");
  std::set<std::string> topics;
  for (const auto & value : record["topics"]) {
    const auto name = topic(value, "record.topics");
    if (!topics.insert(name).second) throw std::runtime_error("record.topics重复话题：" + name);
    r.topics.push_back(name);
  }
  r.regex = text(record["regex"], "record.regex");
  r.exclude = text(record["exclude"], "record.exclude");
  if (!r.regex.empty()) std::regex{r.regex};
  if (!r.exclude.empty()) std::regex{r.exclude};
  if ((!r.all && r.topics.empty() && r.regex.empty()) ||
      (r.all && (!r.topics.empty() || !r.regex.empty())) ||
      (!r.exclude.empty() && (!r.topics.empty() || (!r.all && r.regex.empty())))) {
    throw std::runtime_error("record的all/topics/regex/exclude选择冲突或未选择任何话题");
  }
  if (r.use_sim_time && r.is_discovery_disabled) {
    throw std::runtime_error("use_sim_time需要发现/clock，不能同时禁用话题发现");
  }

  const auto qos = yaml["qos"];
  if (!qos.IsMap()) throw std::runtime_error("qos必须为话题到配置的映射");
  for (const auto & entry : qos) {
    const auto name = topic(entry.first, "qos话题");
    const auto profile = entry.second;
    check_keys(profile, {"history", "depth", "reliability", "durability"}, "qos." + name);
    const auto depth = integer(profile["depth"], "qos." + name + ".depth");
    if (!depth || depth > std::numeric_limits<size_t>::max()) {
      throw std::runtime_error("qos depth必须为可表示的正整数：" + name);
    }
    rclcpp::QoS value(static_cast<size_t>(depth));
    const auto history = text(profile["history"], "qos.history");
    if (history == "keep_last") value.keep_last(static_cast<size_t>(depth));
    else if (history == "keep_all") value.keep_all();
    else throw std::runtime_error("qos history只支持keep_last/keep_all：" + name);
    const auto reliability = text(profile["reliability"], "qos.reliability");
    if (reliability == "reliable") value.reliable();
    else if (reliability == "best_effort") value.best_effort();
    else throw std::runtime_error("qos reliability只支持reliable/best_effort：" + name);
    const auto durability = text(profile["durability"], "qos.durability");
    if (durability == "volatile") value.durability_volatile();
    else if (durability == "transient_local") value.transient_local();
    else throw std::runtime_error("qos durability只支持volatile/transient_local：" + name);
    if (!r.topic_qos_profile_overrides.emplace(name, value).second) {
      throw std::runtime_error("qos重复话题：" + name);
    }
  }
  return result;
}

fs::path default_config()
{
  const auto share = fs::absolute(
    ament_index_cpp::get_package_share_directory("ly_bag_recorder")).lexically_normal();
  // 从当前ament安装位置查找源码仓库，移动整个仓库后仍读取新位置的外部配置。
  // 源码目录标记用于跳过share下的安装副本，避免它抢先匹配config/rosbag.yaml。
  for (auto ancestor = share; !ancestor.empty();) {
    const auto source = ancestor / "config/rosbag.yaml";
    if (fs::is_directory(ancestor / "odom_ws/src/ly_bag_recorder") && fs::exists(source)) {
      return source;
    }
    const auto parent = ancestor.parent_path();
    if (parent == ancestor) break;
    ancestor = parent;
  }
  return share / "config/rosbag.yaml";
}

fs::path unique_bag_path(const fs::path & directory)
{
  // 显式UTC+8生成北京时间，不修改进程或系统时区，更不操作PTP。
  const auto beijing = std::time(nullptr) + 8 * 3600;
  std::tm civil{};
  if (!gmtime_r(&beijing, &civil)) throw std::runtime_error("无法取得北京时间");
  std::random_device random;
  for (int attempt = 0; attempt < 16; ++attempt) {
    std::ostringstream name;
    name << std::put_time(&civil, "odom_%Y%m%d_%H%M%S_")
         << std::hex << std::setw(8) << std::setfill('0') << random();
    auto path = directory / name.str();
    if (!fs::exists(path)) return path;
  }
  throw std::runtime_error("无法分配独立录包路径");
}
}  // namespace

int main(int argc, char ** argv)
{
  std::shared_ptr<rosbag2_transport::Recorder> recorder;
  int status = 0;
  try {
    // rclcpp原生SIGINT/SIGTERM使spin结束；真正写盘在主线程stop()中完成。
    rclcpp::init(argc, argv);
    fs::path config_path;
    std::string output_override;
    {
      // 只读取两个路径参数；配置内容仍由本节点直接从原文件读取。
      rclcpp::Node configuration_node("ly_bag_recorder");
      const auto explicit_path = configuration_node.declare_parameter<std::string>("config_file", "");
      output_override = configuration_node.declare_parameter<std::string>("output_dir", "");
      config_path = explicit_path.empty() ? default_config() : fs::absolute(explicit_path);
    }
    config_path = config_path.lexically_normal();
    auto config = load_configuration(config_path, output_override);
    if (!rclcpp::ok()) return 0;
    fs::create_directories(config.output_directory);
    config.storage.uri = unique_bag_path(config.output_directory).string();
    std::cout << "录包配置：" << config_path << "\nROS话题录制到："
              << config.storage.uri << std::endl;
    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    // Humble要求非空KeyboardHandler；关闭键盘输入且保留rclcpp信号处理。
    auto keyboard_handler = std::make_shared<InactiveKeyboardHandler>();
    recorder = std::make_shared<rosbag2_transport::Recorder>(
      writer, keyboard_handler, config.storage, config.record, "ly_bag_recorder");
    recorder->record();
    rclcpp::spin(recorder);
  } catch (const std::exception & error) {
    std::cerr << "录包失败：" << error.what() << std::endl;
    status = 1;
  }
  if (recorder) {
    try {
      // 原生stop停止发现及订阅，刷新双缓存，关闭writer并写metadata.yaml。
      recorder->stop();
      std::cout << "录包已停止，缓存与metadata已写盘。" << std::endl;
    } catch (const std::exception & error) {
      std::cerr << "录包关闭失败：" << error.what() << std::endl;
      status = 1;
    }
    recorder.reset();
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
