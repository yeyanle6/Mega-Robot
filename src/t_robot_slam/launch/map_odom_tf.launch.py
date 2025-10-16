#!/usr/bin/env python3

"""Legacy FAST-LIO map->odom publisher.

Kept for archival purposes; newer workflows should rely on RTAB-Map to manage
the TF tree.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Dynamic transform from map to odom
    # This node will publish the transform from map to odom based on FAST-LIO's camera_init -> body transform
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        map_odom_tf
    ])
