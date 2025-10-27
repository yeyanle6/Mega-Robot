#!/usr/bin/env python3
"""
MegaRover3 RTABMAP SLAM Launch File

This launch file integrates:
- MegaRover3 robot platform
- Livox MID360 LiDAR
- Intel RealSense D455i depth camera
- RTABMAP SLAM system

Usage:
    ros2 launch megarover_rtabmap.launch.py

Arguments:
    use_sim_time: Use simulation time (default: false)
    rviz: Launch RViz for visualization (default: true)
    localization: Use localization mode instead of mapping (default: false)
    database_path: Path to RTABMAP database (default: ~/.ros/rtabmap.db)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Use localization mode instead of mapping'
    )

    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to RTABMAP database'
    )

    # Robot base launch (MegaRover3)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('megarover3_bringup'),
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={
            'rover': 'mega3',
            'option': ''  # No additional options for basic configuration
        }.items()
    )

    # Livox MID360 LiDAR launch
    livox_launch = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,  # PointCloud2
            'multi_topic': 0,  # Single topic
            'data_src': 0,     # LiDAR source
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'mid360_frame',
            'user_config_path': os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'config',
                'MID360_config.json'
            )
        }],
        remappings=[
            ('livox/lidar', '/livox/lidar'),
            ('livox/imu', '/livox/imu')
        ]
    )

    # RealSense D455i camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'd455',
            'camera_namespace': 'camera',
            'serial_no': "''",
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_imu': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'align_depth.enable': 'true',
            'initial_reset': 'true',
            'use_sim_time': use_sim_time
        }.items()
    )

    # RTABMAP parameters
    rtabmap_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        'use_sim_time': use_sim_time,

        # Subscriptions
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan_cloud': True,
        'subscribe_odom_info': False,
        'approx_sync': False,

        # Topics
        'rgb_topic': '/camera/color/image_raw',
        'depth_topic': '/camera/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera/color/camera_info',
        'scan_cloud_topic': '/livox/lidar',
        'odom_topic': '/odom',
        'imu_topic': '/livox/imu',

        # Queue sizes
        'queue_size': 10,
        'sync_queue_size': 10,

        # QoS
        'qos_image': 1,        # Reliable
        'qos_camera_info': 1,  # Reliable
        'qos_scan': 1,         # Reliable
        'qos_odom': 1,         # Reliable
        'qos_imu': 1,          # Reliable

        # RTABMAP parameters
        'database_path': database_path,
        'Rtabmap/DetectionRate': 1.0,
        'Rtabmap/CreateIntermediateNodes': True,
        'Rtabmap/MaxRetrieved': 2,
        'Rtabmap/MemoryThr': 0,

        # Visual features
        'Vis/FeatureType': 6,  # GFTT/BRIEF
        'Vis/MaxFeatures': 1000,
        'Vis/MinInliers': 10,

        # Registration
        'Reg/Strategy': 1,     # ICP
        'Reg/Force3DoF': True,  # 2D motion constraint

        # ICP parameters
        'Icp/MaxCorrespondenceDistance': 0.1,
        'Icp/Iterations': 30,
        'Icp/VoxelSize': 0.05,

        # Grid map
        'Grid/3D': False,      # 2D grid map
        'Grid/RayTracing': True,
        'Grid/RangeMax': 20.0,
        'Grid/CellSize': 0.05,
        'Grid/PreVoxelFiltering': True,
        'Grid/FromDepth': False,  # Use laser scan

        # Memory management
        'Mem/UseOdomFeatures': False,
        'Mem/NotLinkedNodesKept': False,
    }

    # RTABMAP node
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan_cloud', '/livox/lidar'),
            ('odom', '/odom'),
            ('imu', '/livox/imu')
        ],
        arguments=[
            '--delete_db_on_start' if not localization else ''
        ]
    )

    # Optional: RGB-D synchronization node
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': False,
            'queue_size': 10,
            'depth_scale': 1.0,
            'compressed_rate': 0.0
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('rgbd_image', '/rtabmap/rgbd_image')
        ]
    )

    # Optional: Point cloud to laser scan converter
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 1.0,
            'angle_min': -3.14159,  # -pi
            'angle_max': 3.14159,    # pi
            'angle_increment': 0.00872665,  # pi/360
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True
        }],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan')
        ]
    )

    # RViz configuration
    rviz_config = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'config',
        'rtabmap.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen'
    )

    # Log info
    log_info = LogInfo(
        msg=['Launching MegaRover3 with RTABMAP SLAM\n',
             'Configuration:\n',
             '  - Robot: MegaRover3\n',
             '  - LiDAR: Livox MID360\n',
             '  - Camera: Intel RealSense D455i\n',
             '  - SLAM: RTABMAP\n',
             '  - Localization mode: ', localization, '\n',
             '  - Database: ', database_path, '\n']
    )

    return LaunchDescription([
        # Declare launch arguments
        declare_use_sim_time,
        declare_rviz,
        declare_localization,
        declare_database_path,

        # Log launch information
        log_info,

        # Launch nodes
        robot_launch,
        livox_launch,
        realsense_launch,
        rtabmap_node,
        rgbd_sync_node,
        pointcloud_to_laserscan_node,
        rviz_node
    ])