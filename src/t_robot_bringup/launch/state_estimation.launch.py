#!/usr/bin/env python3
"""
Launch robot_localization EKF with static TF publishers and diagnostics.
Fuses wheel odometry and MID360 IMU for robust state estimation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('t_robot_bringup')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (Gazebo) if true'
    )

    ekf_param_file_arg = DeclareLaunchArgument(
        'ekf_param_file',
        default_value=PathJoinSubstitution([pkg_share, 'params', 'ekf.yaml']),
        description='Full path to robot_localization parameter file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_param_file = LaunchConfiguration('ekf_param_file')

    # EKF localization node - fuses /odom and /mid360/imu
    # Outputs: /odometry/filtered (used by RTAB-Map and Nav2)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_param_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    # Static TF: map -> odom (initially identity, updated by SLAM/localization)
    # This is a placeholder - in real navigation, this is published by AMCL or SLAM
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='log',
        arguments=[
            '0', '0', '0',  # translation x y z
            '0', '0', '0', '1',  # rotation qx qy qz qw
            'map',
            'odom'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # IMU relay - converts Livox IMU QoS from best_effort to reliable
    # Required because robot_localization EKF uses reliable QoS by default
    imu_relay = Node(
        package='t_robot_bringup',
        executable='imu_relay.py',
        name='imu_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # TF diagnostics - monitors TF tree health
    tf2_monitor = Node(
        package='t_robot_bringup',
        executable='time_sync_monitor.py',
        name='time_sync_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot localization diagnostics aggregator (optional)
    # Uncomment if you want system-level diagnostics
    # diagnostic_aggregator = Node(
    #     package='diagnostic_aggregator',
    #     executable='aggregator_node',
    #     name='diagnostic_aggregator',
    #     output='screen',
    #     parameters=[
    #         PathJoinSubstitution([pkg_share, 'params', 'diagnostics.yaml']),
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_param_file_arg,
        imu_relay,
        ekf_node,
        static_tf_map_odom,
        tf2_monitor,
        # diagnostic_aggregator,  # Uncomment if needed
    ])
