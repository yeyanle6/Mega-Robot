#!/usr/bin/env python3
"""
RealSense D455相机启动文件（简洁版）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time')

    # RealSense官方launch文件
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # 启动RealSense
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'camera_name': 'd455',
            'camera_namespace': 'camera',
            'serial_no': "''",
            'device_type': 'd455',
            # 深度和彩色流配置
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            # IMU配置
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',
            # 深度对齐
            'align_depth.enable': 'true',
            'initial_reset': 'true',
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        realsense_node,
    ])
