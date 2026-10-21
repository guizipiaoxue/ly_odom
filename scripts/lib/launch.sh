#!/usr/bin/env bash
# 由 scripts/odom.sh 和 scripts/map.sh 加载；不单独启动。
# 两个入口共享进程锁，禁止重复启动本仓库的驱动、建图和 Recorder。
# 锁随进程退出由内核释放；不根据旧 PID 文件误杀其他进程。
exec 9>"/tmp/ly_odom_stack_${UID}.lock"
if ! flock -n 9; then
    echo '错误：odom.sh 或 map.sh 已在运行，请使用现有链路，不重复启动。' >&2
    exit 1
fi

# 先检查全部环境文件，缺少构建产物时给出对应的构建命令。
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo '错误：未找到 ROS 2 Humble 环境 /opt/ros/humble/setup.bash。' >&2
    exit 1
fi
workspaces=(odom_ws)
# 是否启硬件由launch开关决定；各节点自行读取配置。
# Livox已安装则加载可选CustomMsg库；回放不启动驱动或查询PTP。
if [[ -f "$PROJECT_ROOT/livox_sdk/install/local_setup.bash" ]]; then
    workspaces=(livox_sdk "${workspaces[@]}")
fi
# 只加载回环消息的运行环境，不在这里重复启动 loop/map 或录包节点。
# 原生 Recorder 需要能找到 ly_locate 自定义消息的类型支持库。
if [[ "$PROFILE" == map || -f "$PROJECT_ROOT/location_ws/install/local_setup.bash" ]]; then
    workspaces+=(location_ws)
fi
for workspace in "${workspaces[@]}"; do
    if [[ ! -f "$PROJECT_ROOT/$workspace/install/local_setup.bash" ]]; then
        package="ly_lio ly_bag_recorder"
        [[ "$workspace" != location_ws ]] || package=ly_locate
        [[ "$workspace" != livox_sdk ]] || package=livox_ros_driver2
        echo "错误：$workspace 尚未构建，请先执行：" >&2
        if [[ "$workspace" == livox_sdk ]]; then
            echo "  source /opt/ros/humble/setup.bash" >&2
            echo "  cd \"$PROJECT_ROOT/livox_sdk\"" >&2
            echo '  colcon build --packages-select livox_ros_driver2 --parallel-workers 1 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble' >&2
            echo "  首次构建准备见 $PROJECT_ROOT/PTP/AGENTS.md。" >&2
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
packages=(ly_lio ly_bag_recorder)
[[ "$PROFILE" != map ]] || packages+=(ly_locate)
[[ "${workspaces[0]}" != livox_sdk ]] || packages=(livox_ros_driver2 "${packages[@]}")
for package in "${packages[@]}"; do
    if ! ros2 pkg prefix "$package" >/dev/null; then
        echo "错误：未找到 ROS 包 $package，请在对应工作区重新构建。" >&2
        exit 1
    fi
done

# 固定工作目录，使里程计配置中的相对地图保存路径可预测。
cd -- "$PROJECT_ROOT"
# 用同一个 ROS launch 管理驱动、里程计和全话题录包；PTP 由 systemd 管理。
exec ros2 launch "$SCRIPT_DIR/launch/$PROFILE.launch.py" "$@"
