#!/usr/bin/env python3
"""
Livox MID360激光雷达启动文件（简洁版）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Livox配置文件
    livox_config = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config',
        'MID360_config.json'
    )

    # Livox驱动节点
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_mid360',
        output='screen',
        parameters=[{
            'xfer_format': 0,           # 0=PointCloud2
            'multi_topic': 0,           # 0=单话题
            'data_src': 0,              # 0=激光雷达
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'mid360_frame',
            'enable_imu': True,
            'user_config_path': livox_config,
            'use_sim_time': use_sim_time,
        }],
        respawn=True,
        respawn_delay=2
    )

    # 点云转2D激光扫描
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,  # 0.5度
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        livox_driver,
        pointcloud_to_laserscan,
    ])
