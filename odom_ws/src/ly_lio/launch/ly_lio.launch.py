# 默认接入原始传感器数据；机体安装外参只用于输出位姿。
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_node(context):
    overrides = {
        'body_extrinsic_path': ParameterValue(
            LaunchConfiguration('body_config_path'), value_type=str),
    }
    # 空值沿用YAML，显式launch参数才覆盖；不让默认launch值屏蔽用户配置。
    for argument, parameter, value_type in (
            ('terminal_enabled', 'terminal.enabled', bool),
            ('terminal_clear_screen', 'terminal.clear_screen', bool),
            ('terminal_rate', 'terminal.refresh_rate_hz', float)):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            overrides[parameter] = ParameterValue(LaunchConfiguration(argument), value_type=value_type)
    # 仅外层终端是真TTY时创建伪终端；重定向到文件时不得因emulate_tty误启ANSI。
    interactive = sys.stdout.isatty() and os.environ.get('TERM', 'dumb') != 'dumb'
    return [Node(package='ly_lio', executable='ly_lio_node', name='ly_lio',
                 output='screen', output_format='{line}', emulate_tty=interactive,
                 parameters=[LaunchConfiguration('config'), overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=PathJoinSubstitution([
                FindPackageShare('ly_lio'), 'config', 'mid360.yaml',
            ]), description='逐点激光惯性里程计配置文件'),
        DeclareLaunchArgument(
            'body_config_path', default_value='',
            description='lidar→baselink 安装外参文件；MID360 输出机体位姿时必须指定'),
        DeclareLaunchArgument('terminal_enabled', default_value='',
                              description='终端面板true/false；留空使用YAML'),
        DeclareLaunchArgument('terminal_clear_screen', default_value='',
                              description='交互TTY刷新时清屏true/false；留空使用YAML'),
        DeclareLaunchArgument('terminal_rate', default_value='',
                              description='面板刷新Hz，(0,10]；留空使用YAML'),
        OpaqueFunction(function=launch_node),
    ])
