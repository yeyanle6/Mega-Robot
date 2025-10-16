#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('t_robot_slam')
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    params_file = LaunchConfiguration('params_file')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_dir, 'config', 'navigation', 'map.yaml'),
        description='Full path to map yaml file to load')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'navigation', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    # Include the static TF launch file
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'static_tf_fixed.launch.py')
        )
    )

    # Include the corrected pointcloud to laserscan launch file
    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'corrected_pointcloud_to_laserscan.launch.py')
        ),
        launch_arguments={
            'cloud_topic': '/cloud_registered',
            'scan_topic': '/scan',
            'min_height': '-0.04',
            'max_height': '0.35',
            'angle_min': '-3.14159',
            'angle_max': '3.14159',
            'angle_increment': '0.01',
            'scan_time': '0.1',
            'range_min': '0.1',
            'range_max': '50.0'
        }.items()
    )

    # Include the navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # Start RViz2 for navigation visualization
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'],
        parameters=[{'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add the commands to the launch description
    ld.add_action(static_tf_launch)
    ld.add_action(pointcloud_to_laserscan_launch)
    ld.add_action(navigation_launch)
    ld.add_action(rviz_cmd)

    return ld