#!/usr/bin/env python3
"""
MegaRover3 Nav2导航启动文件（简洁版）
启动Nav2导航栈，配合RTABMAP SLAM使用
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_megarover_nav = get_package_share_directory('megarover_navigation')

    # 参数文件
    nav2_params_file = os.path.join(pkg_megarover_nav, 'config', 'nav2_params.yaml')

    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    use_slam = LaunchConfiguration('use_slam')

    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='自动启动Nav2节点'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='启动RViz'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='地图文件路径（使用已有地图时）'
    )

    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='是否使用SLAM建图（false时使用AMCL定位）'
    )

    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    # RViz（可选）
    rviz_config = os.path.join(pkg_megarover_nav, 'rviz', 'nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        # 参数声明
        declare_use_sim_time,
        declare_autostart,
        declare_use_rviz,
        declare_map,
        declare_use_slam,

        # Nav2启动
        nav2_bringup,

        # RViz
        rviz_node,
    ])
