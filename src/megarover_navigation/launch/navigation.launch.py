#!/usr/bin/env python3
"""
MegaRover3导航系统主启动文件

使用方式:
  # 仅SLAM建图
  ros2 launch megarover_navigation navigation.launch.py mode:=slam_only

  # SLAM + 导航
  ros2 launch megarover_navigation navigation.launch.py mode:=slam_nav

  # 仅导航（使用已有地图）
  ros2 launch megarover_navigation navigation.launch.py mode:=nav_only map:=/path/to/map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('megarover_navigation')

    # 参数声明
    mode = LaunchConfiguration('mode')
    sensor_mode = LaunchConfiguration('sensor_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')

    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='slam_nav',
        choices=['slam_only', 'slam_nav', 'nav_only'],
        description='Operation mode: slam_only, slam_nav, or nav_only'
    )

    declare_sensor_mode = DeclareLaunchArgument(
        'sensor_mode',
        default_value='auto',
        choices=['auto', 'fusion', 'lidar_only', 'rgbd_only'],
        description='Sensor configuration mode'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map file for navigation mode'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    # SLAM系统（slam_only或slam_nav模式）
    slam_launch = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'slam_only' or '", mode, "' == 'slam_nav'"])
        ),
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'modular_rtabmap.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz': 'false',  # 使用Nav2的RViz
                    'force_mode': sensor_mode,
                    'enable_monitoring': 'true',
                }.items()
            )
        ]
    )

    # Nav2导航系统（slam_nav或nav_only模式）
    nav2_launch = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'slam_nav' or '", mode, "' == 'nav_only'"])
        ),
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'megarover_nav2_slam.launch.py')
                ),
                launch_arguments={
                    'mode': PythonExpression(["'slam' if '", mode, "' == 'slam_nav' else 'localization'"]),
                    'map': map_file,
                    'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
                    'use_sim_time': use_sim_time,
                    'namespace': namespace,
                    'rviz_config': os.path.join(pkg_dir, 'rviz', 'nav2_rviz_config.rviz') if rviz == 'true' else '',
                    'sensor_mode': sensor_mode,
                    'enable_nav2': 'true'
                }.items()
            )
        ]
    )

    # 创建launch描述
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_mode)
    ld.add_action(declare_sensor_mode)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz)
    ld.add_action(declare_map)
    ld.add_action(declare_namespace)

    # 添加日志信息
    ld.add_action(LogInfo(msg=[
        '\n========================================\n',
        'MegaRover3 Navigation System\n',
        '========================================\n',
        'Mode: ', mode, '\n',
        'Sensor Mode: ', sensor_mode, '\n',
        'Namespace: ', namespace, '\n',
        '========================================\n'
    ]))

    # 添加launch组件
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)

    return ld