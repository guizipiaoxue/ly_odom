// Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
#pragma once
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <iomanip>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>

#include "ly_locate/pose_graph.hpp"

namespace ly_locate {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

// 历史数据按传感器采样时间关联，不能把零时间解释为 TF 的 latest。
inline int64_t ns(const builtin_interfaces::msg::Time& t) {
    if (t.sec < 0 || t.nanosec >= 1000000000) {
        throw std::invalid_argument("非法采样时间");
    }
    const int64_t stamp = int64_t(t.sec) * 1000000000 + t.nanosec;
    if (stamp <= 0) {
        throw std::invalid_argument("采样时间必须>0，TF零时间代表latest不可用于建图");
    }
    return stamp;
}

inline Pose pose(const geometry_msgs::msg::Pose& p) {
    const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Pose t = Pose::Identity();
    t.translation() << p.position.x, p.position.y, p.position.z;
    if (!q.coeffs().allFinite() || !t.translation().allFinite() ||
        std::abs(q.norm() - 1.0) > 1e-3) {
        throw std::invalid_argument("非法位姿/四元数");
    }
    t.linear() = q.normalized().toRotationMatrix();
    return t;
}

inline Pose pose(const geometry_msgs::msg::Transform& p) {
    geometry_msgs::msg::Pose v;
    v.position.x = p.translation.x;
    v.position.y = p.translation.y;
    v.position.z = p.translation.z;
    v.orientation = p.rotation;
    return pose(v);
}

inline geometry_msgs::msg::Pose message(const Pose& p) {
    geometry_msgs::msg::Pose v;
    v.position.x = p.translation().x();
    v.position.y = p.translation().y();
    v.position.z = p.translation().z();
    const Eigen::Quaterniond q(p.rotation());
    v.orientation.x = q.x();
    v.orientation.y = q.y();
    v.orientation.z = q.z();
    v.orientation.w = q.w();
    return v;
}

inline geometry_msgs::msg::Transform transform(const Pose& p) {
    const auto v = message(p);
    geometry_msgs::msg::Transform t;
    t.translation.x = v.position.x;
    t.translation.y = v.position.y;
    t.translation.z = v.position.z;
    t.rotation = v.orientation;
    return t;
}

inline double angle(const Pose& p) {
    return Eigen::AngleAxisd(p.rotation()).angle();
}

// t 表示源坐标系到目标坐标系的变换：p_target = t * p_source。
inline Cloud::Ptr transformed(const Cloud::ConstPtr& c, const Pose& t) {
    auto out = std::make_shared<Cloud>();
    pcl::transformPointCloud(*c, *out, t.matrix().cast<float>());
    return out;
}

inline Cloud::Ptr downsample(const Cloud::ConstPtr& c, double leaf) {
    auto out = std::make_shared<Cloud>();
    pcl::VoxelGrid<pcl::PointXYZ> v;
    v.setInputCloud(c);
    v.setLeafSize(leaf, leaf, leaf);
    v.filter(*out);
    return out;
}

// 注册云当前只有XYZ；显式解析PC2，拒绝布局错误，兼容大小端与行填充。
inline Cloud::Ptr cloud(const sensor_msgs::msg::PointCloud2& m) {
    using F = sensor_msgs::msg::PointField;
    std::array<F, 3> fields;
    std::array<bool, 3> found{};
    for (const auto& f : m.fields) {
        for (size_t i = 0; i < 3; ++i) {
            if (f.name == std::array<std::string, 3>{"x", "y", "z"}[i]) {
                fields[i] = f;
                found[i] = true;
            }
        }
    }
    if (m.point_step == 0 || uint64_t(m.row_step) < uint64_t(m.width) * m.point_step ||
        uint64_t(m.data.size()) < uint64_t(m.height) * m.row_step) {
        throw std::invalid_argument("点云布局/长度非法");
    }
    for (size_t i = 0; i < 3; ++i) {
        const auto& f = fields[i];
        size_t size = f.datatype == F::FLOAT32 ? 4 : (f.datatype == F::FLOAT64 ? 8 : 0);
        if (!found[i] || !size || f.count != 1 || uint64_t(f.offset) + size > m.point_step) {
            throw std::invalid_argument("点云XYZ字段非法");
        }
    }
    auto out = std::make_shared<Cloud>();
    out->reserve(size_t(m.width) * m.height);
    const uint16_t native = 1;
    const bool swap = m.is_bigendian != (*reinterpret_cast<const uint8_t*>(&native) == 0);
    for (size_t r = 0; r < m.height; ++r) {
        for (size_t c = 0; c < m.width; ++c) {
            Eigen::Vector3f p;
            for (size_t i = 0; i < 3; ++i) {
                const auto& f = fields[i];
                const size_t size = f.datatype == F::FLOAT32 ? 4 : 8;
                std::array<uint8_t, 8> b{};
                std::memcpy(b.data(), m.data.data() + r * m.row_step + c * m.point_step + f.offset,
                            size);
                if (swap) {
                    std::reverse(b.begin(), b.begin() + size);
                }
                if (size == 4) {
                    float v;
                    std::memcpy(&v, b.data(), 4);
                    p[i] = v;
                } else {
                    double v;
                    std::memcpy(&v, b.data(), 8);
                    p[i] = static_cast<float>(v);
                }
            }
            if (p.allFinite()) {
                out->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
            }
        }
    }
    out->is_dense = true;
    return out;
}

inline sensor_msgs::msg::PointCloud2 message(const Cloud& c, const std_msgs::msg::Header& h) {
    sensor_msgs::msg::PointCloud2 m;
    m.header = h;
    m.height = 1;
    m.width = c.size();
    m.point_step = 12;
    m.row_step = 12 * m.width;
    m.is_dense = true;
    const uint16_t native = 1;
    m.is_bigendian = *reinterpret_cast<const uint8_t*>(&native) == 0;
    for (size_t i = 0; i < 3; ++i) {
        sensor_msgs::msg::PointField f;
        f.name = std::array<std::string, 3>{"x", "y", "z"}[i];
        f.offset = 4 * i;
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count = 1;
        m.fields.push_back(f);
    }
    m.data.resize(m.row_step);
    for (size_t i = 0; i < c.size(); ++i) {
        float p[3] = {c[i].x, c[i].y, c[i].z};
        std::memcpy(m.data.data() + 12 * i, p, 12);
    }
    return m;
}

inline std::string unique_id() {
    std::random_device rd;
    std::ostringstream s;
    s << std::hex;
    for (int i = 0; i < 4; ++i) {
        s << std::setw(8) << std::setfill('0') << rd();
    }
    return s.str();
}

// 先完成临时文件写入，再在同一目录重命名，避免读者看到半份元数据。
inline void write_text(const std::filesystem::path& path, const std::string& content) {
    auto tmp = path;
    tmp += ".tmp";
    std::ofstream f(tmp);
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f << content;
    f.close();
    std::filesystem::rename(tmp, path);
}

// 配置从源码目录优先查找，安装目录回退；declare_parameter 保留 ROS 原生覆盖。
class Config {
public:
    Config(rclcpp::Node& n, const std::string& key) : node_(n) {
        const auto share =
            std::filesystem::path(ament_index_cpp::get_package_share_directory("ly_locate"));
        auto path = share / "config/location.yaml";
        for (auto d = share; !d.empty();) {
            for (const auto& r : {"src/ly_locate/config/location.yaml",
                                  "location_ws/src/ly_locate/config/location.yaml"}) {
                if (std::filesystem::is_regular_file(d / r)) {
                    path = d / r;
                    goto located;
                }
            }
            if (d.parent_path() == d) {
                break;
            }
            d = d.parent_path();
        }
    located:
        defaults_ = YAML::LoadFile(path.string())[key]["ros__parameters"];
        if (!defaults_ || !defaults_.IsMap()) {
            throw std::invalid_argument("location.yaml缺少节点参数: " + key);
        }
        RCLCPP_INFO(node_.get_logger(), "默认配置: %s；ROS原生覆盖优先，修改重启生效",
                    path.c_str());
    }

    template <class T>
    T get(const std::string& key, T fallback) {
        T value =
            node_.declare_parameter<T>(key, defaults_[key] ? defaults_[key].as<T>() : fallback);
        if constexpr (std::is_floating_point_v<T>) {
            if (!std::isfinite(value)) {
                throw std::invalid_argument("参数非有限: " + key);
            }
        }
        return value;
    }

private:
    rclcpp::Node& node_;
    YAML::Node defaults_;
};

inline void positive(double x, const std::string& name) {
    if (!(x > 0) || !std::isfinite(x)) {
        throw std::invalid_argument(name + "必须有限且>0");
    }
}
}  // namespace ly_locate
