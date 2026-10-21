# Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from launch_ros.actions import Node


def process_exited(event, context):
    if not context.is_shutdown:
        return [EmitEvent(event=Shutdown(
            reason=f'回环/建图进程退出（code={event.returncode}），停止链路并刷新录包'))]
    return []


def generate_launch_description():
    # 两个节点使用相同的 ROS 时钟；回放时由 /clock 驱动采样时间关联。
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            Node(
                package='ly_locate',
                executable='loop_node',
                name='location_loop',
                output='screen',
                on_exit=process_exited,
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            ),
            Node(
                package='ly_locate',
                executable='map_node',
                # Ctrl+C后等待完整地图PCD/元数据原子导出，避免默认5秒过早升级终止信号。
                sigterm_timeout='30',
                sigkill_timeout='30',
                name='location_map',
                output='screen',
                on_exit=process_exited,
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            ),
        ]
    )
