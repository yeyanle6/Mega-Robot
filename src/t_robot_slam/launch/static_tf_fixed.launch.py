#!/usr/bin/env python3

"""Legacy FAST-LIO static TF setup.

Maintained as documentation of the previous LiDAR mounting assumption. The
RTAB-Map bringup will ship its own TF publishers.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 静态变换：描述 FAST-LIO 的 camera_init 传感器框架相对于底盘 base_link 的安装位置
    # 约定激光雷达位于机器人中心上方 0.4 米，方向与 base_link 对齐。
    base_link_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_init',
        arguments=['0', '0', '0.4', '0', '0', '0', 'base_link', 'camera_init'],
        output='screen'
    )

    return LaunchDescription([
        base_link_to_camera_init
    ])
