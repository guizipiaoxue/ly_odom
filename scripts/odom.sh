#!/usr/bin/env bash
# Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PROFILE=odom

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<'EOF'
用法：./scripts/odom.sh [ROS launch 参数]
启动驱动、ly_lio和唯一全话题Recorder；去畸变注册云由ly_lio直接发布到baselink。
launch统一位于scripts/launch；节点直接读取所属包内配置。
start_lidar:=false 使用外部驱动/回放；record_bag:=false 关闭本链路录包。
回放示例：start_lidar:=false record_bag:=false use_sim_time:=true
录包配置：odom_ws/src/ly_bag_recorder/config/rosbag.yaml；默认输出仓库bags/。
Ctrl+C统一退出并等待Recorder刷新缓存；不启停PTP或重复启动已有链路。
EOF
    exit 0
fi
source "$SCRIPT_DIR/lib/launch.sh"
