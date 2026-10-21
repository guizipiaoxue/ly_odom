#!/usr/bin/env bash
# Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PROFILE=map

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<'EOF'
用法：./scripts/map.sh [ROS launch 参数]
启动驱动、ly_lio、Scan Context/GTSAM回环、建图和唯一全话题Recorder。
launch统一位于scripts/launch；节点直接读取所属包内配置。
start_lidar:=false 使用外部驱动/回放；record_bag:=false 关闭本链路录包。
回放示例：start_lidar:=false record_bag:=false use_sim_time:=true
录包配置：odom_ws/src/ly_bag_recorder/config/rosbag.yaml；默认输出仓库bags/。
Ctrl+C自动导出地图到maps/location_export/，等待保存和Recorder刷新完成后再关闭终端。
maps/location/是回环关键帧归档；不启停PTP或重复启动已有链路。
EOF
    exit 0
fi
source "$SCRIPT_DIR/lib/launch.sh"
