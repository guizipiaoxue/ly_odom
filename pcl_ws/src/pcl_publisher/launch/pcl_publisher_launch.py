# 共享安装外参由节点直接定位和读取，不由launch传入。
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='pcl_publisher', executable='pcl_publisher',
             name='pcl_publisher', output='screen'),
    ])
