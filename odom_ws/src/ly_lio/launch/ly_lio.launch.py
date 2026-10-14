# 参数由节点C++直接读取包内YAML，launch只负责启动进程。
import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    interactive = sys.stdout.isatty() and os.environ.get('TERM', 'dumb') != 'dumb'
    return LaunchDescription([
        Node(package='ly_lio', executable='ly_lio_node', name='ly_lio',
             output='screen', output_format='{line}', emulate_tty=interactive),
    ])
