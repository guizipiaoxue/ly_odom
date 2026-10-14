// Copyright 2026 ly_odom contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <yaml-cpp/yaml.h>

namespace {
std::string default_extrinsics()
{
  const auto share = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("pcl_publisher"));
  for (auto ancestor = share; !ancestor.empty();) {
    for (const auto & relative : {"config/lidar2baselink.yaml", "pcl_ws/config/lidar2baselink.yaml"}) {
      const auto candidate = ancestor / relative;
      // 只匹配源码工作区，避免先读到 share 中的旧安装副本。
      const auto workspace = candidate.parent_path().parent_path();
      if (std::filesystem::is_directory(workspace / "src/pcl_publisher") &&
        std::filesystem::is_regular_file(candidate)) {return candidate.string();}
    }
    const auto parent = ancestor.parent_path();
    if (parent == ancestor) {break;}
    ancestor = parent;
  }
  return (share / "config/lidar2baselink.yaml").string();
}
}  // namespace

class PclPublisher : public rclcpp::Node
{
public:
  PclPublisher()
  : Node("pcl_publisher")
  {
    // 节点直接读共享安装文件，源码存在时优先使用，修改后重启即可。
    // 安装到其他机器且无源码时再读取包share中的副本，不依赖launch注入。
    const auto config_path = declare_parameter<std::string>("config_path", default_extrinsics());
    target_frame_ = declare_parameter<std::string>("target_frame", "baselink");
    if (target_frame_.empty()) {
      throw std::runtime_error("target_frame must not be empty");
    }
    load_transform(config_path);

    const auto input_qos = rclcpp::SensorDataQoS();
    // 点云变换与IMU回调共用执行线程；默认5条仅覆盖25ms，点云批次可挤掉IMU。
    // 加深队列保留原采样时间和到达顺序，仍以best effort兼容驱动发布端。
    const auto imu_input_qos = rclcpp::SensorDataQoS().keep_last(1000);
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/bashlink/pcl", output_qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/baselink/imu", output_qos);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", input_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {publish_cloud(*msg);});
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", imu_input_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {publish_imu(*msg);});
    RCLCPP_INFO(
      get_logger(), "Loaded %s; publishing point clouds and IMU in frame %s",
      config_path.c_str(), target_frame_.c_str());
  }

private:
  void load_transform(const std::string & path)
  {
    const auto config = YAML::LoadFile(path);
    const auto rotation = config["lidar_to_gimbal_rotation"];
    const auto translation = config["lidar_to_gimbal_translation"];
    if (!rotation.IsSequence() || rotation.size() != 3 ||
      !translation.IsSequence() || translation.size() != 3)
    {
      throw std::runtime_error("Expected a 3x3 rotation and a 3-element translation in " + path);
    }
    for (std::size_t row = 0; row < 3; ++row) {
      if (!rotation[row].IsSequence() || rotation[row].size() != 3) {
        throw std::runtime_error("Rotation must have three elements per row");
      }
      for (std::size_t col = 0; col < 3; ++col) {
        rotation_(row, col) = rotation[row][col].as<double>();
      }
      translation_(row) = translation[row].as<double>();
    }
    if (!rotation_.allFinite() || !translation_.allFinite() ||
      !(rotation_ * rotation_.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-5) ||
      std::abs(rotation_.determinant() - 1.0) > 1e-5)
    {
      throw std::runtime_error("Extrinsics must contain finite values and a proper rotation");
    }
    rotation_quaternion_ = Eigen::Quaterniond(rotation_).normalized();
  }

  static std::uint32_t coordinate_offset(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == name) {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1 ||
          static_cast<std::uint64_t>(field.offset) + sizeof(float) > cloud.point_step)
        {
          throw std::runtime_error("PointCloud2 field " + name + " must be a scalar FLOAT32");
        }
        return field.offset;
      }
    }
    throw std::runtime_error("PointCloud2 is missing coordinate field " + name);
  }

  static bool native_bigendian()
  {
    const std::uint16_t value = 1;
    return *reinterpret_cast<const std::uint8_t *>(&value) == 0;
  }

  static float read_float(const std::uint8_t * data, bool swap)
  {
    std::array<std::uint8_t, sizeof(float)> bytes;
    std::copy_n(data, bytes.size(), bytes.begin());
    if (swap) {
      std::reverse(bytes.begin(), bytes.end());
    }
    float value;
    std::memcpy(&value, bytes.data(), sizeof(value));
    return value;
  }

  static void write_float(std::uint8_t * data, float value, bool swap)
  {
    std::array<std::uint8_t, sizeof(float)> bytes;
    std::memcpy(bytes.data(), &value, sizeof(value));
    if (swap) {
      std::reverse(bytes.begin(), bytes.end());
    }
    std::copy(bytes.begin(), bytes.end(), data);
  }

  void publish_cloud(const sensor_msgs::msg::PointCloud2 & input)
  {
    auto output = input;  // Preserve timestamps, organization and all non-XYZ fields.
    output.header.frame_id = target_frame_;
    try {
      if (input.width != 0 && input.height != 0) {
        if (input.point_step == 0 ||
          static_cast<std::uint64_t>(input.width) * input.point_step > input.row_step ||
          static_cast<std::uint64_t>(input.height) * input.row_step > input.data.size())
        {
          throw std::runtime_error("PointCloud2 has inconsistent dimensions or data size");
        }
        const std::array<std::uint32_t, 3> offsets = {
          coordinate_offset(input, "x"), coordinate_offset(input, "y"),
          coordinate_offset(input, "z")};
        const bool swap = input.is_bigendian != native_bigendian();
        for (std::uint32_t row = 0; row < input.height; ++row) {
          for (std::uint32_t col = 0; col < input.width; ++col) {
            auto * point = output.data.data() + static_cast<std::size_t>(row) * input.row_step +
              static_cast<std::size_t>(col) * input.point_step;
            const Eigen::Vector3d xyz(
              read_float(point + offsets[0], swap), read_float(point + offsets[1], swap),
              read_float(point + offsets[2], swap));
            if (!xyz.allFinite()) {
              output.is_dense = false;
              continue;
            }
            const Eigen::Vector3d transformed = rotation_ * xyz + translation_;
            for (std::size_t axis = 0; axis < 3; ++axis) {
              write_float(point + offsets[axis], static_cast<float>(transformed(axis)), swap);
            }
          }
        }
      }
      cloud_publisher_->publish(output);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping invalid point cloud: %s", error.what());
    }
  }

  void rotate_vector(geometry_msgs::msg::Vector3 & vector) const
  {
    const Eigen::Vector3d transformed = rotation_ * Eigen::Vector3d(vector.x, vector.y, vector.z);
    vector.x = transformed.x();
    vector.y = transformed.y();
    vector.z = transformed.z();
  }

  void rotate_covariance(std::array<double, 9> & covariance) const
  {
    if (covariance[0] == -1.0) {
      return;  // ROS sentinel: this measurement is unavailable.
    }
    using RowMajorMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    Eigen::Map<RowMajorMatrix> matrix(covariance.data());
    matrix = (rotation_ * matrix * rotation_.transpose()).eval();
  }

  void publish_imu(const sensor_msgs::msg::Imu & input)
  {
    auto output = input;
    output.header.frame_id = target_frame_;
    rotate_vector(output.angular_velocity);
    rotate_vector(output.linear_acceleration);
    rotate_covariance(output.angular_velocity_covariance);
    rotate_covariance(output.linear_acceleration_covariance);

    const auto & orientation = input.orientation;
    const Eigen::Quaterniond world_from_lidar(
      orientation.w, orientation.x, orientation.y, orientation.z);
    if (input.orientation_covariance[0] != -1.0 &&
      world_from_lidar.coeffs().allFinite() && world_from_lidar.norm() > 1e-12)
    {
      // IMU orientation maps its local axes to world; change those local axes to baselink.
      const auto world_from_baselink =
        (world_from_lidar.normalized() * rotation_quaternion_.conjugate()).normalized();
      output.orientation.x = world_from_baselink.x();
      output.orientation.y = world_from_baselink.y();
      output.orientation.z = world_from_baselink.z();
      output.orientation.w = world_from_baselink.w();
      rotate_covariance(output.orientation_covariance);
    } else {
      // Preserve missing attitude, and also reject zero/non-finite input quaternions.
      output.orientation = geometry_msgs::msg::Quaternion();
      output.orientation.w = 1.0;
      output.orientation_covariance.fill(0.0);
      output.orientation_covariance[0] = -1.0;
    }
    imu_publisher_->publish(output);
  }

  Eigen::Matrix3d rotation_;
  Eigen::Vector3d translation_;
  Eigen::Quaterniond rotation_quaternion_;
  std::string target_frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<PclPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pcl_publisher"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
