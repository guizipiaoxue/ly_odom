"""统一管理进程；各节点在代码内直接读取所属配置，不在这里转换或注入参数。"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def process_exited(event, context):
    if not context.is_shutdown:
        return [EmitEvent(event=Shutdown(
            reason=f'采集进程退出（code={event.returncode}），停止其余节点并刷新录包'))]
    return []


def launch_processes(context):
    interactive = sys.stdout.isatty() and os.environ.get('TERM', 'dumb') != 'dumb'
    nodes = [
        Node(package='pcl_publisher', executable='pcl_publisher', name='pcl_publisher',
             output='screen', on_exit=process_exited),
        Node(package='ly_lio', executable='ly_lio_node', name='ly_lio',
             output='screen', output_format='{line}', emulate_tty=interactive,
             on_exit=process_exited),
    ]
    if LaunchConfiguration('record_bag').perform(context).lower() == 'true':
        # 独立录包节点直接读取仓库config/rosbag.yaml，保留原生rosbag2发现和写盘。
        nodes.insert(0, Node(
            package='ly_bag_recorder', executable='ly_bag_recorder', name='ly_bag_recorder',
            output='screen', sigterm_timeout='30', sigkill_timeout='30',
            on_exit=process_exited))
    if LaunchConfiguration('start_lidar').perform(context).lower() == 'true':
        # 驱动代码读取自己的mid360.yaml和SDK JSON并进行只读PTP/接收IP检查。
        nodes.append(Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen', on_exit=process_exited))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='true', choices=['true', 'false'],
                              description='启动MID360；回放或使用外部驱动时设false'),
        DeclareLaunchArgument('record_bag', default_value='true', choices=['true', 'false'],
                              description='启动全话题录包节点；录制参数在外层config/rosbag.yaml'),
        OpaqueFunction(function=launch_processes),
    ])
