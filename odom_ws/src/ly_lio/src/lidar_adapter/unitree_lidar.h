/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */


#pragma once
#include "pointcloud_adapter.h"
namespace ly_lio {
// 保留 Small Point-LIO 的雷达适配入口；具体时间单位由节点参数明确指定。
class UnilidarAdapter : public PointCloudAdapter {
public:
    UnilidarAdapter() : PointCloudAdapter(PointCloudTime{true, 1.0, false}) {}
};
}  // namespace ly_lio
