#!/usr/bin/env python3

"""Legacy FAST-LIO transform launcher.

This file keeps the old FAST-LIO TF bridging for reference and is scheduled for
removal once the RTAB-Map workflow is finalized.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 静态变换：将map坐标系连接到camera_init坐标系
    map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        output='screen'
    )
    
    # 动态变换：将camera_init坐标系连接到odom坐标系
    # 这样FAST-LIO的camera_init坐标系就能正确映射到odom坐标系
    map_odom_tf_publisher = Node(
        package='t_robot_slam',
        executable='map_odom_tf_publisher_v2',
        name='map_odom_tf_publisher_v2',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        map_to_camera_init,
        map_odom_tf_publisher
    ])
