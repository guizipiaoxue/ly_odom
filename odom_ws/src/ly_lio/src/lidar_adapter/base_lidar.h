/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include <pch.h>

namespace ly_lio {

    class LidarAdapterBase {
    public:
        virtual ~LidarAdapterBase() = default;
        virtual void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) = 0;
    };

}// namespace ly_lio
