/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */


#pragma once
#include "common/common.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace ly_lio {
// 时间配置显式区分“相对包头偏移”和“绝对时间”，禁止猜测不同驱动的时间语义。
struct PointCloudTime {
    bool relative = true;
    double scale = 1e-9;
    bool filter_livox_tag = true;
};
class PointCloudDecoder {
    using Cloud = sensor_msgs::msg::PointCloud2;
    using Field = sensor_msgs::msg::PointField;
    static size_t field_size(uint8_t type) {
        switch (type) {
            case Field::INT8: case Field::UINT8: return 1;
            case Field::INT16: case Field::UINT16: return 2;
            case Field::INT32: case Field::UINT32: case Field::FLOAT32: return 4;
            case Field::FLOAT64: return 8;
            default: throw std::invalid_argument("不支持的 PointCloud2 数据类型");
        }
    }
    template<class T> static T read(const uint8_t *data, bool swap) {
        std::array<uint8_t, sizeof(T)> bytes;
        std::copy_n(data, bytes.size(), bytes.begin());
        if (swap) { std::reverse(bytes.begin(), bytes.end()); }
        T value;
        std::memcpy(&value, bytes.data(), sizeof(T));
        return value;
    }
    static double number(const uint8_t *data, uint8_t type, bool swap) {
        switch (type) {
            case Field::INT8: return read<int8_t>(data, swap);
            case Field::UINT8: return read<uint8_t>(data, swap);
            case Field::INT16: return read<int16_t>(data, swap);
            case Field::UINT16: return read<uint16_t>(data, swap);
            case Field::INT32: return read<int32_t>(data, swap);
            case Field::UINT32: return read<uint32_t>(data, swap);
            case Field::FLOAT32: return read<float>(data, swap);
            case Field::FLOAT64: return read<double>(data, swap);
            default: throw std::invalid_argument("无效点字段");
        }
    }
    static const Field *field(const Cloud &cloud, const std::string &name) {
        for (const auto &candidate : cloud.fields) {
            if (candidate.name != name) { continue; }
            if (candidate.count != 1 || uint64_t(candidate.offset) + field_size(candidate.datatype) > cloud.point_step) {
                throw std::invalid_argument("字段越界或不是标量：" + name);
            }
            return &candidate;
        }
        return nullptr;
    }
public:
    static std::vector<common::Point> decode(const Cloud &cloud, const PointCloudTime &time) {
        if (!std::isfinite(time.scale) || time.scale <= 0) { throw std::invalid_argument("timestamp_scale 必须为正"); }
        if (cloud.width == 0 || cloud.height == 0) { return {}; }
        if (cloud.point_step == 0 || uint64_t(cloud.width) * cloud.point_step > cloud.row_step ||
            uint64_t(cloud.height) * cloud.row_step > cloud.data.size()) {
            throw std::invalid_argument("PointCloud2 行布局或数据长度不一致");
        }
        const auto *x = field(cloud, "x"), *y = field(cloud, "y"), *z = field(cloud, "z");
        const auto *stamp = field(cloud, "timestamp");
        if (!stamp) { stamp = field(cloud, "time"); }
        if (!stamp) { stamp = field(cloud, "t"); }
        const auto *tag = time.filter_livox_tag ? field(cloud, "tag") : nullptr;
        if (!x || !y || !z || !stamp) { throw std::invalid_argument("缺少 XYZ 或逐点时间字段 timestamp/time/t"); }
        if (time.filter_livox_tag && !tag) { throw std::invalid_argument("Livox 点云缺少 tag 字段"); }
        if (tag && tag->datatype != Field::UINT8) { throw std::invalid_argument("Livox tag 必须为 UINT8"); }
        const uint16_t native = 1;
        const bool native_big = *reinterpret_cast<const uint8_t *>(&native) == 0;
        const bool swap = cloud.is_bigendian != native_big;
        const double base = time.relative ? cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9 : 0.0;
        std::vector<common::Point> points;
        points.reserve(size_t(cloud.width) * cloud.height);
        for (uint32_t row = 0; row < cloud.height; ++row) {
            for (uint32_t col = 0; col < cloud.width; ++col) {
                const auto *data = cloud.data.data() + size_t(row) * cloud.row_step + size_t(col) * cloud.point_step;
                if (tag) {
                    const auto value = static_cast<uint8_t>(number(data + tag->offset, tag->datatype, swap));
                    // 允许正常的第 0/1 回波，排除噪声置信度标记；兼容 MID360 的 tag=0x10。
                    if ((value & 0x30) > 0x10 || (value & 0x0f) != 0) { continue; }
                }
                common::Point point;
                point.position = Eigen::Vector3f(number(data + x->offset, x->datatype, swap),
                    number(data + y->offset, y->datatype, swap), number(data + z->offset, z->datatype, swap));
                const double raw_time = number(data + stamp->offset, stamp->datatype, swap);
                point.timestamp = base + raw_time * time.scale;
                if (!point.position.allFinite() || !std::isfinite(point.timestamp) ||
                    (time.relative && raw_time < 0)) { continue; }
                points.push_back(point);
            }
        }
        return points;
    }
};
}  // namespace ly_lio
