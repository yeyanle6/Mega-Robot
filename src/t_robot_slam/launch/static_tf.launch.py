#!/usr/bin/env python3

"""Archive of the old FAST-LIO TF chain.

The RTAB-Map workflow will define its own static transforms; keep this file only
as background reference.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Static transform from map to camera_init (FAST-LIO coordinate system)
    # This connects the map frame to FAST-LIO's coordinate system
    map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        output='screen'
    )

    return LaunchDescription([
        map_to_camera_init
    ])
