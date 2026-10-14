"""MID360与RViz进程入口；驱动参数由C++直接读取本包config/mid360.yaml。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config', 'display_point_cloud_ROS2.rviz')
    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]),
    ])
