#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('t_robot_slam')
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
    use_remappings = LaunchConfiguration('use_remappings')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'navigation', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_lifecycle_mgr_cmd = DeclareLaunchArgument(
        'use_lifecycle_mgr',
        default_value='true',
        description='Whether to launch the lifecycle manager')
    
    declare_use_remappings_cmd = DeclareLaunchArgument(
        'use_remappings',
        default_value='false',
        description='Arguments to pass to all nodes launched by the file')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'config', 'navigation', 'map.yaml'),
        description='Full path to map yaml file to load')

    # Create the remapped parameters
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Nodes launching commands
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother',
                       'map_server',
                       'amcl']

    # Start the nav2_controller node
    controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')])

    # Start the nav2_smoother node
    smoother_cmd = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=[configured_params])

    # Start the nav2_planner node
    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[configured_params])

    # Start the nav2_behaviors node
    behavior_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[configured_params])

    # Start the nav2_bt_navigator node
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[configured_params])

    # Start the nav2_waypoint_follower node
    waypoint_follower_cmd = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=[configured_params])

    # Start the nav2_velocity_smoother node
    velocity_smoother_cmd = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel_in', 'cmd_vel_nav'),
                    ('cmd_vel_out', 'cmd_vel')])

    # NOTE: map_server, amcl, and lifecycle_manager are now managed by nav2_bringup
    # to avoid duplicate node launches and lifecycle manager conflicts.
    # These nodes are commented out to prevent conflicts with navigation_launch.py

    # map_server_cmd = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     output='screen',
    #     parameters=[configured_params])

    # amcl_cmd = Node(
    #     package='nav2_amcl',
    #     executable='amcl',
    #     output='screen',
    #     parameters=[configured_params])

    # lifecycle_manager_cmd = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'autostart': autostart},
    #                 {'node_names': lifecycle_nodes}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_mgr_cmd)
    ld.add_action(declare_use_remappings_cmd)
    ld.add_action(declare_map_yaml_cmd)

    # Add the commands to the launch description
    # NOTE: All nodes are now launched via nav2_bringup to avoid conflicts
    # ld.add_action(controller_cmd)
    # ld.add_action(smoother_cmd)
    # ld.add_action(planner_cmd)
    # ld.add_action(behavior_cmd)
    # ld.add_action(bt_navigator_cmd)
    # ld.add_action(waypoint_follower_cmd)
    # ld.add_action(velocity_smoother_cmd)
    # ld.add_action(map_server_cmd)
    # ld.add_action(amcl_cmd)
    # ld.add_action(lifecycle_manager_cmd)

    # Include Nav2 localization (map_server + amcl)
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'map': map_yaml_file
        }.items()
    )

    # Include Nav2 navigation (planner, controller, bt_navigator, etc.)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    ld.add_action(nav2_localization_launch)
    ld.add_action(nav2_navigation_launch)

    return ld