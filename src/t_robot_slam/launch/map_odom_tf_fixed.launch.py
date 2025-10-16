#!/usr/bin/env python3

"""Deprecated FAST-LIO map->odom bridge.

Retained for historical reference while the RTAB-Map-based transform chain is
being implemented. Do not include in new launch flows.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 启动map到odom的TF变换发布器
    # 这个节点将FAST-LIO的camera_init->body变换转换为map->odom变换
    map_odom_tf_publisher = Node(
        package='t_robot_slam',
        executable='map_odom_tf_publisher',
        name='map_odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        map_odom_tf_publisher
    ])
