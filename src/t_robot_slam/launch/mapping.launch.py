#!/usr/bin/env python3
"""
RTAB-Map 3D LiDAR SLAM - Mapping Mode Launch File
Optimized for Livox MID360 with odometry input
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # Package directories
    pkg_share = FindPackageShare('t_robot_slam')
    bringup_share = FindPackageShare('t_robot_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    database_path = LaunchConfiguration('database_path')
    delete_db = LaunchConfiguration('delete_db')
    use_viz = LaunchConfiguration('use_viz')
    launch_bringup = LaunchConfiguration('launch_bringup')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (Gazebo) if true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'params', 'rtabmap.yaml']),
        description='Full path to the RTAB-Map parameter file'
    )

    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value=str(Path.home() / '.ros' / 'rtabmap.db'),
        description='Database path for RTAB-Map (uses $HOME/.ros/rtabmap.db by default)'
    )

    declare_delete_db = DeclareLaunchArgument(
        'delete_db',
        default_value='false',
        description='Delete existing database on start (true for fresh mapping)'
    )

    declare_use_viz = DeclareLaunchArgument(
        'use_viz',
        default_value='false',
        description='Launch rtabmapviz for 3D visualization'
    )

    declare_launch_bringup = DeclareLaunchArgument(
        'launch_bringup',
        default_value='false',
        description='Include t_robot_bringup/launch/bringup.launch.py'
    )


    # Optional: Include bringup if requested
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'bringup.launch.py'])
        ),
        condition=IfCondition(launch_bringup)
    )

    # Point cloud preprocessor - filters and transforms MID360 data
    # Optimized for: 30° tilt angle + 60cm robot height
    pointcloud_preprocessor = Node(
        package='t_robot_slam',
        executable='pointcloud_preprocessor',
        name='pointcloud_preprocessor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/livox/lidar',
            'output_filtered_topic': '/mid360/points_filtered',
            'output_obstacle_topic': '/cloud/obstacles',
            'output_ground_topic': '/cloud/ground',
            'target_frame': 'base_link',
            'source_frame': 'livox_frame',
            'transform_timeout': 0.2,
            # Range filter - 机器人高度60cm，只关心±50cm范围
            'range_filter.enabled': True,
            'range_filter.min_range': 0.3,      # 最小距离减小（避免过滤机器人本体附近）
            'range_filter.max_range': 15.0,     # 最大距离减小到15m（室内足够）
            'range_filter.min_height': -0.3,    # 地面最低-30cm（容忍不平整）
            'range_filter.max_height': 1.0,     # 最高1m（机器人60cm，障碍物最高考虑到1m）
            # Voxel grid downsampling
            'voxel_grid.enabled': True,
            'voxel_grid.leaf_size': 0.05,
            # Statistical outlier removal
            'outlier_removal.enabled': True,
            'outlier_removal.mean_k': 50,
            'outlier_removal.stddev_mul': 1.0,
            # Ground segmentation - 针对30°倾角优化
            'ground_segmentation.enabled': True,
            'ground_segmentation.ransac_max_iterations': 200,    # 增加迭代次数提高准确性
            'ground_segmentation.ransac_distance_threshold': 0.03,  # 放宽阈值以适应倾斜
            'verbose': False,
        }]
    )

    # RTAB-Map SLAM node
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'database_path': database_path,
                'delete_db_on_start': delete_db,
            }
        ],
        remappings=[
            ('odom', '/odometry/filtered'),
            ('scan_cloud', '/mid360/points_filtered'),
        ],
        arguments=['--delete_db_on_start'] if delete_db else []
    )

    # RTAB-Map visualization (optional)
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmapviz',
        output='screen',
        condition=IfCondition(use_viz),
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_scan_cloud': True,
            'approx_sync': True,
        }],
        remappings=[
            ('odom', '/odometry/filtered'),
            ('scan_cloud', '/mid360/points_filtered'),
        ]
    )

    return LaunchDescription([
        # Environment variable for RTAB-Map logging
        SetEnvironmentVariable('RTABMAP_CONSOLE_OUTPUT', 'true'),

        # Launch arguments
        declare_use_sim_time,
        declare_params_file,
        declare_database_path,
        declare_delete_db,
        declare_use_viz,
        declare_launch_bringup,

        # Nodes
        bringup_launch,
        pointcloud_preprocessor,
        rtabmap_node,
        rtabmap_viz_node,
    ])
