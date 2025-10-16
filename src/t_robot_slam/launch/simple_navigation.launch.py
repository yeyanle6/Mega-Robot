#!/usr/bin/env python3

"""Legacy FAST-LIO navigation helper.

This launch file depends on FAST-LIO topics (e.g. `/cloud_registered` and the
`camera_init` frame). It is retained only for reference while the RTAB-Map
pipeline is under construction and should not be used in new deployments.
"""

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
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_dir, 'config', 'navigation', 'map.yaml'),
        description='Full path to map yaml file to load')

    # 静态变换：map -> camera_init (FAST-LIO坐标系)
    map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        output='screen'
    )
    
    # 动态变换：camera_init -> odom (将FAST-LIO的camera_init映射到odom)
    map_odom_tf_publisher = Node(
        package='t_robot_slam',
        executable='map_odom_tf_publisher_v2',
        name='map_odom_tf_publisher_v2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 点云转激光扫描
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'cloud_topic': '/cloud_registered',
            'scan_topic': '/scan',
            'min_height': -0.04,
            'max_height': 0.35,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.01,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 50.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[('cloud_in', '/cloud_registered'),
                    ('scan', '/scan')]
    )

    # 地图服务器
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }]
    )

    # AMCL定位
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_link',
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'scan_topic': 'scan',
            'max_particles': 2000,
            'min_particles': 500,
            'laser_model_type': 'likelihood_field',
            'laser_max_range': 100.0,
            'laser_min_range': -1.0,
            'laser_max_beams': 60,
            'laser_likelihood_max_dist': 2.0,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25
        }]
    )

    # 生命周期管理器
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # 启动Navigation2
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_dir, 'config', 'navigation', 'nav2_params.yaml'),
            'use_rviz': 'true'
        }.items()
    )

    # 创建启动描述
    ld = LaunchDescription()

    # 添加声明的启动参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_file_cmd)

    # 添加节点
    ld.add_action(map_to_camera_init)
    ld.add_action(map_odom_tf_publisher)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(lifecycle_manager)
    ld.add_action(nav2_bringup)

    return ld
