from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 驱动构造函数自行读取config/mid360.yaml和持久SDK JSON。
    # 本入口只启动进程；参数可用ROS --params-file/-p显式覆盖。
    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
        ),
    ])
