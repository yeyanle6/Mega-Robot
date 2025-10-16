#!/usr/bin/env python3
"""RTAB-Map localization mode launch file (skeleton)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('t_robot_slam')
    bringup_share = FindPackageShare('t_robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    database_path = LaunchConfiguration('database_path')
    use_viz = LaunchConfiguration('use_viz')
    launch_bringup = LaunchConfiguration('launch_bringup')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time (Gazebo) if true.')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'params', 'rtabmap.yaml']),
        description='Full path to the RTAB-Map parameter file.')
    declare_database_path = DeclareLaunchArgument(
        'database_path', default_value='~/.ros/rtabmap.db',
        description='Existing RTAB-Map database used for localization.')
    declare_use_viz = DeclareLaunchArgument(
        'use_viz', default_value='false',
        description='Launch rtabmapviz for visualization.')
    declare_launch_bringup = DeclareLaunchArgument(
        'launch_bringup', default_value='false',
        description='Include t_robot_bringup/launch/bringup.launch.py in this launch.')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'bringup.launch.py'])
        ),
        condition=IfCondition(launch_bringup)
    )

    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'database_path': database_path,
                'delete_db_on_start': False,
                'localization': True,
                'Mem/IncrementalMemory': False,
                'queue_size': 50,
            },
        ],
        remappings=[
            ('odom', '/odometry/filtered'),
            ('scan_cloud', '/mid360/points_filtered'),
            ('imu', '/mid360/imu')
        ],
    )

    rtabmap_viz_node = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        name='rtabmapviz',
        output='screen',
        condition=IfCondition(use_viz),
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_database_path,
        declare_use_viz,
        declare_launch_bringup,
        bringup_launch,
        rtabmap_node,
        rtabmap_viz_node,
    ])
