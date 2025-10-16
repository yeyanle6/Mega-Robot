#!/usr/bin/env python3
"""
Megarover3 base driver launch file.
Starts robot_state_publisher, joint_state_publisher, and odometry publisher.
Includes megarover3_bringup for full chassis control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # Package shares
    megarover_desc_share = FindPackageShare('megarover_description')
    megarover3_bringup_share = FindPackageShare('megarover3_bringup')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    use_megarover_driver_arg = DeclareLaunchArgument(
        'use_megarover_driver',
        default_value='true',
        description='Launch megarover3 base driver (motor controller)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_megarover_driver = LaunchConfiguration('use_megarover_driver')

    # URDF path
    urdf_path = PathJoinSubstitution([
        megarover_desc_share,
        'urdf',
        'mega3.xacro'
    ])

    # Megarover3 base driver - motor controller and low-level control
    # NOTE: robot.launch.py includes robot_state_publisher, joint_state_publisher,
    # pub_odom, and rviz2, so we don't duplicate them here
    megarover3_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                megarover3_bringup_share,
                'launch',
                'robot.launch.py'
            ])
        ),
        condition=IfCondition(use_megarover_driver)
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_megarover_driver_arg,
        megarover3_driver_launch,
    ])
