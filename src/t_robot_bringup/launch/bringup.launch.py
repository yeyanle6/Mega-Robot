#!/usr/bin/env python3
"""Top-level bringup entry point combining base, sensors, and state estimation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('t_robot_bringup')

    use_base = LaunchConfiguration('use_base')
    use_sensors = LaunchConfiguration('use_sensors')
    use_state_estimation = LaunchConfiguration('use_state_estimation')

    declare_use_base = DeclareLaunchArgument(
        'use_base', default_value='true',
        description='Include Megarover3 base driver bringup.')
    declare_use_sensors = DeclareLaunchArgument(
        'use_sensors', default_value='true',
        description='Include MID360 and auxiliary sensor bringup.')
    declare_use_state_estimation = DeclareLaunchArgument(
        'use_state_estimation', default_value='true',
        description='Launch robot_localization EKF pipeline.')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'base.launch.py'])
        ),
        condition=IfCondition(use_base)
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'sensors.launch.py'])
        ),
        condition=IfCondition(use_sensors)
    )

    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'state_estimation.launch.py'])
        ),
        condition=IfCondition(use_state_estimation)
    )

    ld = LaunchDescription([
        declare_use_base,
        declare_use_sensors,
        declare_use_state_estimation,
        base_launch,
        sensors_launch,
        state_estimation_launch,
    ])

    return ld
