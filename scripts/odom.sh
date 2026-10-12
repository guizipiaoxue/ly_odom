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
PTP 由 systemd 开机自启，本脚本只检查服务，不启动、重启或停止 PTP。
里程计直接订阅原始话题，保留 /bashlink/pcl、/baselink/imu 发布。
转换与里程计共用 pcl_ws/config/lidar2baselink.yaml，里程计参数读取 ly_lio/config/mid360.yaml。
可覆盖配置：./scripts/odom.sh config:=/绝对路径/里程计.yaml config_path:=/绝对路径/外参.yaml
更换网卡：./scripts/odom.sh ptp_interface:=enp86s0
回放 rosbag 或使用外部驱动：./scripts/odom.sh start_lidar:=false（跳过 PTP/网卡检查）
默认保存到仓库 bags/odom_日期_时间_随机后缀/，终端会显示完整路径。
更改保存目录：./scripts/odom.sh bag_dir:=/绝对路径/录包目录
关闭录包：./scripts/odom.sh record_bag:=false
关闭里程计终端面板：./scripts/odom.sh terminal_enabled:=false
保留终端日志不清屏：./scripts/odom.sh terminal_clear_screen:=false terminal_rate:=1.0
EOF
    exit 0
fi

# 先检查全部环境文件，缺少构建产物时给出对应的构建命令。
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo '错误：未找到 ROS 2 Humble 环境 /opt/ros/humble/setup.bash。' >&2
    exit 1
fi
workspaces=(pcl_ws odom_ws)
# 回放模式不要求存在 Livox 构建产物；若已安装，加载可选CustomMsg的运行库。
# 加载消息库不会启动驱动；是否启动硬件仍由launch中的start_lidar决定。
start_lidar=true
for arg in "$@"; do
    case "$arg" in
        start_lidar:=*) start_lidar="${arg#start_lidar:=}" ;;
    esac
done
case "$start_lidar" in
    true|True|1) workspaces=(livox_sdk "${workspaces[@]}") ;;
    false|False|0)
        if [[ -f "$PROJECT_ROOT/livox_sdk/install/local_setup.bash" ]]; then
            workspaces=(livox_sdk "${workspaces[@]}")
        fi
        ;;
    *) echo '错误：start_lidar 只能为 true/false 或 1/0。' >&2; exit 2 ;;
esac
for workspace in "${workspaces[@]}"; do
    if [[ ! -f "$PROJECT_ROOT/$workspace/install/local_setup.bash" ]]; then
        package=pcl_publisher
        [[ "$workspace" != odom_ws ]] || package=ly_lio
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
packages=(pcl_publisher ly_lio)
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
