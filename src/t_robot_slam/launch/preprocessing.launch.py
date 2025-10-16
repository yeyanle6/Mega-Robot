#!/usr/bin/env python3
"""
Point cloud preprocessing pipeline for MID360 LiDAR.
Applies voxel filtering, outlier removal, and ground segmentation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('t_robot_slam')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'preprocessing_params.yaml'
        ]),
        description='Path to preprocessing parameters file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    # Point cloud preprocessor node
    preprocessor_node = Node(
        package='t_robot_slam',
        executable='pointcloud_preprocessor',
        name='pointcloud_preprocessor',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        preprocessor_node,
    ])
