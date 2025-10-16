#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('t_robot_slam')
    
    # Create the launch configuration variables
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    min_height = LaunchConfiguration('min_height')
    max_height = LaunchConfiguration('max_height')
    angle_min = LaunchConfiguration('angle_min')
    angle_max = LaunchConfiguration('angle_max')
    angle_increment = LaunchConfiguration('angle_increment')
    scan_time = LaunchConfiguration('scan_time')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    use_inf = LaunchConfiguration('use_inf')
    inf_epsilon = LaunchConfiguration('inf_epsilon')
    concurrency_level = LaunchConfiguration('concurrency_level')
    
    # Declare the launch arguments
    declare_cloud_topic_cmd = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/cloud_registered',
        description='Topic for the input point cloud')
    
    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic for the output laser scan')
    
    # 修正后的高度参数 - 基于机器人导航需求
    declare_min_height_cmd = DeclareLaunchArgument(
        'min_height',
        default_value='-0.04',  # 相对于机器人base_link的最小高度
        description='Minimum height to consider for laser scan relative to base_link')
    
    declare_max_height_cmd = DeclareLaunchArgument(
        'max_height',
        default_value='0.35',  # 相对于机器人base_link的最大高度
        description='Maximum height to consider for laser scan relative to base_link')
    
    declare_angle_min_cmd = DeclareLaunchArgument(
        'angle_min',
        default_value='-3.14159',
        description='Minimum angle for laser scan')
    
    declare_angle_max_cmd = DeclareLaunchArgument(
        'angle_max',
        default_value='3.14159',
        description='Maximum angle for laser scan')
    
    declare_angle_increment_cmd = DeclareLaunchArgument(
        'angle_increment',
        default_value='0.01',
        description='Angle increment for laser scan')
    
    declare_scan_time_cmd = DeclareLaunchArgument(
        'scan_time',
        default_value='0.1',
        description='Scan time for laser scan')
    
    declare_range_min_cmd = DeclareLaunchArgument(
        'range_min',
        default_value='0.1',
        description='Minimum range for laser scan')
    
    declare_range_max_cmd = DeclareLaunchArgument(
        'range_max',
        default_value='50.0',
        description='Maximum range for laser scan')
    
    declare_use_inf_cmd = DeclareLaunchArgument(
        'use_inf',
        default_value='true',
        description='Use infinity values')
    
    declare_inf_epsilon_cmd = DeclareLaunchArgument(
        'inf_epsilon',
        default_value='1.0',
        description='Infinity epsilon value')
    
    declare_concurrency_level_cmd = DeclareLaunchArgument(
        'concurrency_level',
        default_value='1',
        description='Concurrency level for processing')

    # Start the pointcloud_to_laserscan node
    pointcloud_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='corrected_pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'cloud_topic': cloud_topic,
            'scan_topic': scan_topic,
            'min_height': min_height,
            'max_height': max_height,
            'angle_min': angle_min,
            'angle_max': angle_max,
            'angle_increment': angle_increment,
            'scan_time': scan_time,
            'range_min': range_min,
            'range_max': range_max,
            'use_inf': use_inf,
            'inf_epsilon': inf_epsilon,
            'concurrency_level': concurrency_level,
            # QoS修复: 使用 'system_default' (RELIABLE) 以兼容 Nav2
            # 原来的 'sensor_data' (BEST_EFFORT) 导致 Nav2 无法订阅
            'qos_overrides./scan.publisher.reliability': 'reliable',
            'qos_overrides./scan.publisher.durability': 'volatile',
            'qos_overrides./scan.publisher.history': 'keep_last',
            'qos_overrides./scan.publisher.depth': 10,
            'target_frame': 'base_link'
        }],
        remappings=[('cloud_in', cloud_topic),
                    ('scan', scan_topic)])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_cloud_topic_cmd)
    ld.add_action(declare_scan_topic_cmd)
    ld.add_action(declare_min_height_cmd)
    ld.add_action(declare_max_height_cmd)
    ld.add_action(declare_angle_min_cmd)
    ld.add_action(declare_angle_max_cmd)
    ld.add_action(declare_angle_increment_cmd)
    ld.add_action(declare_scan_time_cmd)
    ld.add_action(declare_range_min_cmd)
    ld.add_action(declare_range_max_cmd)
    ld.add_action(declare_use_inf_cmd)
    ld.add_action(declare_inf_epsilon_cmd)
    ld.add_action(declare_concurrency_level_cmd)

    # Add the commands to the launch description
    ld.add_action(pointcloud_to_laserscan_cmd)

    return ld