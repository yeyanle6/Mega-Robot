"""
点云转2D地图 Launch文件

将FASTLIO2的3D点云实时转换为2D激光扫描和占用栅格地图

依赖:
  sudo apt install ros-humble-pointcloud-to-laserscan ros-humble-slam-toolbox
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # 点云转2D激光扫描
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.1,
                'min_height': -0.1,       # 最小高度 (相对base_link，允许略低于base_link)
                'max_height': 1.5,        # 最大高度
                'angle_min': -3.14159,    # -180度
                'angle_max': 3.14159,     # 180度
                'angle_increment': 0.00436,  # 约0.25度
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cloud_in', '/world_cloud'),
                ('scan', '/scan_from_pointcloud'),
            ],
        ),

        # SLAM Toolbox 在线建图
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan_from_pointcloud',
                'mode': 'mapping',
                'resolution': 0.05,
                'max_laser_range': 20.0,
                'minimum_travel_distance': 0.3,
                'minimum_travel_heading': 0.3,
                'map_update_interval': 2.0,
                'transform_publish_period': 0.0,  # 不发布TF，使用FASTLIO2的
                'tf_buffer_duration': 30.0,
            }],
        ),
    ])
