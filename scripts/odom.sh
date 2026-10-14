#!/usr/bin/env bash
# 保留 baselink 点云/IMU 发布；ly_lio 直接接原始数据，在 IMU 原点滤波后输出基座位姿。
set -eo pipefail

# 使用脚本自身的位置定位工作区，支持从任意目录执行。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    cat <<'EOF'
用法：./scripts/odom.sh [ROS launch 参数]
启动 MID360 驱动、pcl_publisher、ly_lio，并将全部 ROS 话题录制到 rosbag。
Ctrl+C 统一停止录包并退出 ROS 节点，等待录包完成写盘后再关闭终端。
PTP 由 systemd 开机自启，驱动代码只读检查服务，不启动、重启或停止 PTP。
里程计直接订阅原始话题，保留 /bashlink/pcl、/baselink/imu 发布。
算法/IMU/地图/终端：odom_ws/src/ly_lio/config/mid360.yaml
Livox参数：livox_sdk/src/livox_ros_driver2/config/mid360.yaml
Livox网络：livox_sdk/src/livox_ros_driver2/config/MID360_config.json（SDK原生格式）
安装外参：pcl_ws/config/lidar2baselink.yaml
录包参数与QoS：config/rosbag.yaml（ROS工作区外）
节点代码直接读取配置；本脚本只加载环境并管理进程，不解析/生成/注入配置。
回放已有rosbag：./scripts/odom.sh start_lidar:=false record_bag:=false
默认录包保存到仓库 bags/odom_日期_时间_随机后缀/，保存目录在config/rosbag.yaml修改。
关闭录包：./scripts/odom.sh record_bag:=false
EOF
    exit 0
fi

# 先检查全部环境文件，缺少构建产物时给出对应的构建命令。
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo '错误：未找到 ROS 2 Humble 环境 /opt/ros/humble/setup.bash。' >&2
    exit 1
fi
workspaces=(pcl_ws odom_ws)
# 是否启硬件由launch开关决定；各节点自行读取配置。
# Livox已安装则加载可选CustomMsg库；回放不启动驱动或查询PTP。
if [[ -f "$PROJECT_ROOT/livox_sdk/install/local_setup.bash" ]]; then
    workspaces=(livox_sdk "${workspaces[@]}")
fi
for workspace in "${workspaces[@]}"; do
    if [[ ! -f "$PROJECT_ROOT/$workspace/install/local_setup.bash" ]]; then
        package=pcl_publisher
        [[ "$workspace" != odom_ws ]] || package="ly_lio ly_bag_recorder"
        [[ "$workspace" != livox_sdk ]] || package=livox_ros_driver2
        echo "错误：$workspace 尚未构建，请先执行：" >&2
        if [[ "$workspace" == livox_sdk ]]; then
            echo "  $PROJECT_ROOT/PTP/build_driver.sh" >&2
            exit 1
        fi
        echo "  source /opt/ros/humble/setup.bash" >&2
        echo "  cd \"$PROJECT_ROOT/$workspace\"" >&2
        echo "  colcon build --packages-select $package --symlink-install" >&2
        exit 1
    fi
done

# ROS 环境脚本可能引用未定义变量，因此不启用 bash 的 nounset。
source /opt/ros/humble/setup.bash
for workspace in "${workspaces[@]}"; do
    source "$PROJECT_ROOT/$workspace/install/local_setup.bash"
done
packages=(pcl_publisher ly_lio ly_bag_recorder)
[[ "${workspaces[0]}" != livox_sdk ]] || packages=(livox_ros_driver2 "${packages[@]}")
for package in "${packages[@]}"; do
    if ! ros2 pkg prefix "$package" >/dev/null; then
        echo "错误：未找到 ROS 包 $package，请在对应工作区重新构建。" >&2
        exit 1
    fi
done

# 固定工作目录，使里程计配置中的相对地图保存路径可预测。
cd -- "$PROJECT_ROOT"
# 用同一个 ROS launch 管理驱动、转换、里程计和全话题录包；PTP 由 systemd 管理。
exec ros2 launch "$SCRIPT_DIR/odom.launch.py" "$@"
