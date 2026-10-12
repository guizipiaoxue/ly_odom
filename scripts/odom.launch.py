"""统一启动雷达、坐标变换、里程计及全话题录包；PTP 独立开机自启。"""

from datetime import datetime
from pathlib import Path
import re
import shutil
import subprocess
import sys
from uuid import uuid4
from zoneinfo import ZoneInfo

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def recorder_exited(event, context):
    # 录包失败或被单独停止时，停止采集，避免系统继续运行却没有保存数据。
    # 正常 Ctrl+C 已在 shutdown 中，无需重复发出关闭事件。
    if not context.is_shutdown:
        return [EmitEvent(event=Shutdown(
            reason=f'rosbag 录制意外退出（code={event.returncode}），停止采集'))]
    return []


def prepare_processes(context, root):
    # 先完成 PTP/网络检查，再准备输出路径；检查失败不启动任何进程。
    lidar_actions = launch_lidar(context, root)
    if not IfCondition(LaunchConfiguration('record_bag')).evaluate(context):
        return lidar_actions
    if shutil.which('ros2') is None:
        raise RuntimeError('未找到 ros2，请先加载 ROS 2 Humble 环境')
    subprocess.run(['ros2', 'pkg', 'prefix', 'rosbag2_transport'],
                   check=True, stdout=subprocess.DEVNULL, timeout=5)
    bag_dir = Path(LaunchConfiguration('bag_dir').perform(context)).expanduser()
    # 相对保存目录始终相对于项目根目录，避免调用脚本的工作目录影响保存位置。
    if not bag_dir.is_absolute():
        bag_dir = root / bag_dir
    bag_dir.mkdir(parents=True, exist_ok=True)
    # 用北京时间标记运行时间；随机后缀避免同秒启动覆盖已有录包。
    name = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('odom_%Y%m%d_%H%M%S')
    bag_path = bag_dir / f'{name}_{uuid4().hex[:8]}'
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--all', '--include-hidden-topics',
             '--storage', 'sqlite3', '--output', str(bag_path),
             '--qos-profile-overrides-path', str(root / 'scripts/rosbag_qos.yaml')],
        name='rosbag_recorder', output='screen',
        # SIGINT 先让 rosbag 刷新缓存、写 metadata；大量数据时增加退出宽限。
        sigterm_timeout='30', sigkill_timeout='30', on_exit=recorder_exited)
    # 先安排录包，再启动雷达；自动发现后续出现的话题，不使用 --no-discovery。
    return [LogInfo(msg=f'全部 ROS 话题录制到：{bag_path}'), recorder, *lidar_actions]


def launch_lidar(context, root):
    # 回放原始 rosbag 时不启动驱动，也不依赖本机 PTP 或雷达网卡。
    if not IfCondition(LaunchConfiguration('start_lidar')).evaluate(context):
        return []
    interface = LaunchConfiguration('ptp_interface').perform(context)
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', interface):
        raise RuntimeError('ptp_interface 网卡名称无效')
    # 同一部署的两个服务必须同时运行，不能混用 sentry 与 ly 的实例。
    # 这里只读服务状态；启动失败时不尝试校时或启动另一套 PTP。
    for prefix in ('sentry', 'ly'):
        units = [f'{prefix}-ptp4l@{interface}.service',
                 f'{prefix}-phc2sys@{interface}.service']
        # systemctl 多单元查询只要任一 active 就返回 0，因此须逐个检查。
        if all(
                subprocess.run(['systemctl', 'is-active', '--quiet', unit],
                               check=False, timeout=5).returncode == 0
                for unit in units):
            break
    else:
        raise RuntimeError(
            f'{interface} 的 PTP/PHC 服务未同时运行；请检查开机自启服务。'
            '首次部署执行 sudo ./PTP/install.sh ' + interface)
    subprocess.run([
        sys.executable, str(root / 'PTP/host_check.py'),
        '--interface', interface, '--require-network',
    ], check=True, timeout=30)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py',
        ])),
        # 驱动 launch 仍强制 PTP，未同步数据不回退为主机收包时间。
        launch_arguments={
            'ptp_utc_offset': LaunchConfiguration('ptp_utc_offset'),
            'target_ip': LaunchConfiguration('target_ip'),
        }.items())]


def generate_launch_description():
    root = Path(__file__).resolve().parent.parent
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='启动 MID360 驱动；回放或外部驱动设 false'),
        DeclareLaunchArgument('ptp_interface', default_value='enp86s0',
                              description='运行开机自启 PTP 服务的有线网卡'),
        DeclareLaunchArgument('ptp_utc_offset', default_value='37',
                              description='设备 PTP/TAI 转 ROS UTC 的秒差'),
        DeclareLaunchArgument('target_ip', default_value='192.168.1.3',
                              description='只接收并配置此 MID360，需与驱动 JSON 一致'),
        DeclareLaunchArgument('record_bag', default_value='true',
                              description='录制全部 ROS 话题；仅调试时可设 false'),
        DeclareLaunchArgument('terminal_enabled', default_value='',
                              description='里程计终端面板true/false；留空使用YAML'),
        DeclareLaunchArgument('terminal_clear_screen', default_value='',
                              description='交互终端清屏true/false；留空使用YAML'),
        DeclareLaunchArgument('terminal_rate', default_value='',
                              description='面板刷新Hz；留空使用YAML'),
        DeclareLaunchArgument('bag_dir', default_value=str(root / 'bags'),
                              description='录包父目录，每次运行创建独立的时间戳子目录'),
        # 读取源码配置，修改外参或算法参数后无需重新安装配置文件。
        DeclareLaunchArgument(
            'config_path',
            default_value=str(root / 'pcl_ws/config/lidar2baselink.yaml'),
            description='雷达到 baselink 的外参文件'),
        DeclareLaunchArgument(
            'config',
            default_value=str(root / 'odom_ws/src/ly_lio/config/mid360.yaml'),
            description='ly_lio 里程计参数文件'),
        # 检查必须发生在任何 ROS 节点启动前，避免半套系统运行。
        OpaqueFunction(function=prepare_processes, args=[root]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pcl_publisher'), 'launch', 'pcl_publisher_launch.py',
            ])),
            launch_arguments={'config_path': LaunchConfiguration('config_path')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ly_lio'), 'launch', 'ly_lio.launch.py',
            ])),
            # 两节点使用同一份安装标定；里程计直接接原始数据，保留转换节点供其他使用者。
            launch_arguments={
                'config': LaunchConfiguration('config'),
                'body_config_path': LaunchConfiguration('config_path'),
                'terminal_enabled': LaunchConfiguration('terminal_enabled'),
                'terminal_clear_screen': LaunchConfiguration('terminal_clear_screen'),
                'terminal_rate': LaunchConfiguration('terminal_rate'),
            }.items()),
    ])
