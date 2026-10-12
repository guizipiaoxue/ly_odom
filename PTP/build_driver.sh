#!/usr/bin/env bash
# 增量构建严格 PTP 驱动；不用上游会删除 build/install 的 build.sh。
set -eo pipefail
PTP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$PTP_DIR/.." && pwd)"
DRIVER="$ROOT/livox_sdk/src/livox_ros_driver2"
source /opt/ros/humble/setup.bash
if [[ ! -f "$DRIVER/package.xml" ]]; then
    cp "$DRIVER/package_ROS2.xml" "$DRIVER/package.xml"
fi
# 安装后的 launch 必须与修改后的源码一致，保留其他自定义 launch 文件。
mkdir -p "$DRIVER/launch"
cp "$DRIVER"/launch_ROS2/*.py "$DRIVER/launch/"
cd "$ROOT/livox_sdk"
exec colcon build --packages-select livox_ros_driver2 --parallel-workers 1 --symlink-install \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=OFF
