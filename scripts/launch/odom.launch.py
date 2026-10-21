# Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
"""统一管理采集进程；配置仍由各节点直接读取，不在 launch 中转换。"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def process_exited(event, context):
    # 任一业务进程提前退出时联动停止，避免只有录包还活着造成采集正常的假象。
    if not context.is_shutdown:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=f'采集进程退出（code={event.returncode}），停止其余节点并刷新录包'
                )
            )
        ]
    return []


def launch_processes(context):
    interactive = sys.stdout.isatty() and os.environ.get('TERM', 'dumb') != 'dumb'
    clock_parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    nodes = [
        Node(
            package='ly_lio',
            executable='ly_lio_node',
            name='ly_lio',
            output='screen',
            output_format='{line}',
            emulate_tty=interactive,
            parameters=clock_parameters,
            on_exit=process_exited,
        ),
    ]
    if LaunchConfiguration('record_bag').perform(context).lower() == 'true':
        # 仅此处创建 Recorder；建图入口包含本 launch，不重复定义录包节点。
        # 配置由 Recorder 直接读取，退出留足时间刷新缓存和 metadata。
        nodes.insert(
            0,
            Node(
                package='ly_bag_recorder',
                executable='ly_bag_recorder',
                name='ly_bag_recorder',
                output='screen',
                sigterm_timeout='30',
                sigkill_timeout='30',
                on_exit=process_exited,
            ),
        )
    if LaunchConfiguration('start_lidar').perform(context).lower() == 'true':
        # 驱动自行读 YAML/SDK JSON 并只读检查 PTP；不启停授时服务。
        nodes.append(
            Node(
                package='livox_ros_driver2',
                executable='livox_ros_driver2_node',
                name='livox_lidar_publisher',
                output='screen',
                parameters=clock_parameters,
                on_exit=process_exited,
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_lidar',
                default_value='true',
                choices=['true', 'false'],
                description='启动MID360；回放或使用外部驱动时设false',
            ),
            DeclareLaunchArgument(
                'record_bag',
                default_value='true',
                choices=['true', 'false'],
                description='启动唯一全话题Recorder；配置在ly_bag_recorder/config/rosbag.yaml',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='算法节点使用/clock；回放时设true',
            ),
            OpaqueFunction(function=launch_processes),
        ]
    )
