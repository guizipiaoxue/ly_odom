# Copyright 2026 ly_odom contributors
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('pcl_publisher'), 'config', 'lidar2baselink.yaml',
            ]),
            description='Lidar to baselink extrinsics YAML',
        ),
        Node(
            package='pcl_publisher',
            executable='pcl_publisher',
            output='screen',
            parameters=[{'config_path': LaunchConfiguration('config_path')}],
        ),
    ])
