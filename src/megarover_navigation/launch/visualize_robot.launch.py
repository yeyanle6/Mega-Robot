#!/usr/bin/env python3
"""
机器人模型可视化Launch文件
用于查看URDF模型和TF树，特别是Mid-360的倾斜角度
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 获取URDF文件路径
    megarover_description_path = get_package_share_directory('megarover_description')
    xacro_file = os.path.join(megarover_description_path, 'urdf', 'mega3.xacro')

    # 使用xacro处理URDF
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot State Publisher - 发布TF树
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint State Publisher - 发布关节状态
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # RViz2 - 可视化
    rviz_config_file = os.path.join(
        get_package_share_directory('megarover_navigation'),
        'rviz',
        'robot_model.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
