#!/usr/bin/env python3
"""
MID360 LiDAR and auxiliary sensors launch file.
Starts Livox MID360 driver with calibrated parameters and static TF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('t_robot_bringup')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    use_mid360_arg = DeclareLaunchArgument(
        'use_mid360',
        default_value='true',
        description='Launch MID360 LiDAR driver'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mid360 = LaunchConfiguration('use_mid360')

    # MID360 LiDAR driver
    # Uses the calibrated frame_id from msg_MID360_launch.py
    # Note: Livox driver publishes to /livox/* topics by default
    # We remap them to /mid360/* for consistency
    mid360_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_mid360)
    )

    # Relay Livox topics (/livox/*) to standard MID360 topics (/mid360/*)
    # Ensures downstream nodes subscribe to consistent names
    lidar_relay = Node(
        package='topic_tools',
        executable='relay',
        name='mid360_lidar_relay',
        output='screen',
        arguments=['/livox/lidar', '/mid360/lidar', '--qos-profile', 'sensor_data'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_mid360)
    )

    # Static TF: base_link -> mid360_base (mount position)
    static_tf_mid360_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_mid360_base',
        output='log',
        arguments=[
            '0', '0', '0.25',  # x y z (25cm above base_link)
            '0', '0', '0', '1',  # qx qy qz qw (no rotation)
            'base_link',
            'mid360_base'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static TF: mid360_base -> mid360_lidar
    static_tf_mid360_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_mid360_lidar',
        output='log',
        arguments=[
            '0', '0', '0',  # x y z
            '0', '0', '0', '1',  # qx qy qz qw
            'mid360_base',
            'mid360_lidar'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static TF: mid360_base -> mid360_imu
    static_tf_mid360_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_mid360_imu',
        output='log',
        arguments=[
            '0', '0', '0',  # x y z
            '0', '0', '0', '1',  # qx qy qz qw
            'mid360_base',
            'mid360_imu'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_mid360_arg,
        mid360_launch,
        lidar_relay,
        static_tf_mid360_base,
        static_tf_mid360_lidar,
        static_tf_mid360_imu,
    ])
