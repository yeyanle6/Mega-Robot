"""
FASTLIO2 Localization + MegaRover3 Navigation Launch File

LEGACY ENTRYPOINT
This file is kept for reference/backward compatibility.
For current on-robot use, prefer:
  ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=nav ...

Uses FASTLIO2's localizer for relocalization with a pre-built map.

Usage:
  ros2 launch megarover3_navigation fastlio2_localization.launch.py \
    pcd_map:=/path/to/map.pcd \
    nav_map:=/path/to/nav_map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    megarover3_nav_dir = get_package_share_directory('megarover3_navigation')
    megarover_desc_dir = get_package_share_directory('megarover_description')
    fastlio2_dir = get_package_share_directory('fastlio2')
    localizer_dir = get_package_share_directory('localizer')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pcd_map = LaunchConfiguration('pcd_map')
    nav_map = LaunchConfiguration('nav_map')
    rviz = LaunchConfiguration('rviz', default='true')

    # Config files
    robot_description_path = os.path.join(megarover_desc_dir, 'urdf', 'mega3.xacro')
    fastlio2_config = os.path.join(fastlio2_dir, 'config', 'lio_megarover.yaml')
    localizer_config = os.path.join(localizer_dir, 'config', 'localizer.yaml')
    nav2_params = os.path.join(megarover3_nav_dir, 'config', 'fastlio2_nav2_params.yaml')
    rviz_config = os.path.join(megarover3_nav_dir, 'rviz', 'nav2.rviz')

    # Param substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': nav_map
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'pcd_map',
            description='Full path to PCD map file for FASTLIO2 localization'
        ),
        DeclareLaunchArgument(
            'nav_map',
            description='Full path to 2D map yaml file for Nav2'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),

        # Robot description for base/sensor TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', str(robot_description_path)]), value_type=str
            )}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        # Static TF: lio_base -> base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lio_base_to_base_footprint',
            arguments=['--x', '0', '--y', '0.1', '--z', '0',
                      '--roll', '0', '--pitch', '0', '--yaw', '1.5708',
                      '--frame-id', 'lio_base', '--child-frame-id', 'base_footprint']
        ),

        # Static TF: base_link -> livox_frame
        # Keep this legacy launch aligned with lio_megarover.yaml:
        # x=0.09m forward, z=0.56m, yaw=+90deg
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox',
            arguments=['--x', '0.09', '--y', '0', '--z', '0.56',
                      '--roll', '0', '--pitch', '0', '--yaw', '1.5708',
                      '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']
        ),

        # FASTLIO2 LIO Node
        # 注意: FASTLIO2使用config_path参数指定YAML配置文件路径
        Node(
            package='fastlio2',
            executable='lio_node',
            name='fastlio2_lio',
            output='screen',
            parameters=[{
                'config_path': fastlio2_config,
                'use_sim_time': use_sim_time
            }]
        ),

        # FASTLIO2 Localizer Node
        Node(
            package='localizer',
            executable='localizer_node',
            name='fastlio2_localizer',
            output='screen',
            parameters=[
                localizer_config,
                {
                    'use_sim_time': use_sim_time,
                    'map_path': pcd_map
                }
            ]
        ),

        # Map Server for Nav2
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params]
        ),

        # Lifecycle manager for map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # Nav2 Navigation Stack
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params]
        ),

        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'smoother_server',
                    'velocity_smoother',
                    'waypoint_follower'
                ]
            }]
        ),

        # RViz
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
