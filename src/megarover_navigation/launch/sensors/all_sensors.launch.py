#!/usr/bin/env python3
"""
所有传感器统一启动文件（简洁版）
根据参数选择性启动传感器
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_megarover_nav = get_package_share_directory('megarover_navigation')

    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_livox = LaunchConfiguration('enable_livox')
    enable_realsense = LaunchConfiguration('enable_realsense')

    # Livox启动文件
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_megarover_nav, 'launch', 'sensors', 'livox_mid360.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_livox)
    )

    # RealSense启动文件
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_megarover_nav, 'launch', 'sensors', 'realsense_d455.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_realsense)
    )

    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        DeclareLaunchArgument(
            'enable_livox',
            default_value='true',
            description='启动Livox MID360'
        ),
        DeclareLaunchArgument(
            'enable_realsense',
            default_value='true',
            description='启动RealSense D455'
        ),

        # 传感器启动
        livox_launch,
        realsense_launch,
    ])
