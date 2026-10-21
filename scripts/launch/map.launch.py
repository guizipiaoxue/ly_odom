# Copyright (c) 2026 ly_odom contributors. SPDX-License-Identifier: MIT
"""复用里程计与回环 launch，仅由里程计链路创建一个 Recorder。"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('record_bag', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(directory / 'odom.launch.py')),
            launch_arguments={
                'start_lidar': LaunchConfiguration('start_lidar'),
                'record_bag': LaunchConfiguration('record_bag'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(directory / 'location.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
