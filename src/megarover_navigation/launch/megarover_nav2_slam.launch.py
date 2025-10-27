#!/usr/bin/env python3
"""
MegaRover3 RTABMAP SLAM + Nav2导航集成

这个launch文件同时启动：
- 模块化RTABMAP SLAM系统
- Nav2导航栈
- 可视化工具

使用方式：
  建图+导航: ros2 launch megarover_nav2_slam.launch.py mode:=slam
  定位+导航: ros2 launch megarover_nav2_slam.launch.py mode:=localization map:=/path/to/map.db
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('megarover_navigation')
    bringup_dir = os.path.dirname(os.path.realpath(__file__))

    # Launch配置
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    rviz_config = LaunchConfiguration('rviz_config')
    sensor_mode = LaunchConfiguration('sensor_mode')
    enable_nav2 = LaunchConfiguration('enable_nav2')

    # 创建参数声明
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        choices=['slam', 'localization'],
        description='Navigation mode: slam for mapping, localization for navigation with existing map')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map file (for localization mode) or database (for RTABMAP)')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed nav2 nodes')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='true',
        description='Whether to respawn if a node crashes')

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Full path to the RVIZ config file')

    declare_sensor_mode_cmd = DeclareLaunchArgument(
        'sensor_mode',
        default_value='auto',
        choices=['auto', 'fusion', 'lidar_only', 'rgbd_only'],
        description='Sensor mode for RTABMAP')

    declare_enable_nav2_cmd = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Enable Nav2 navigation stack')

    # 参数替换
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True
        ),
        allow_substs=True
    )

    # RTABMAP SLAM节点组
    rtabmap_group = GroupAction([
        PushRosNamespace(namespace),

        # 机器人底盘
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('megarover3_bringup'),
                    'launch',
                    'robot.launch.py'
                )
            ),
            launch_arguments={
                'rover': 'mega3',
                'option': ''
            }.items()
        ),

        # 模块化RTABMAP SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'modular_rtabmap.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false',  # 使用Nav2的RViz配置
                'force_mode': sensor_mode,
                'enable_monitoring': 'true',
                'database_path': map_file
            }.items()
        ),
    ])

    # Nav2节点组
    nav2_nodes = GroupAction([
        PushRosNamespace(namespace),

        # 如果在定位模式，启动AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
        ),

        # 地图服务器（仅定位模式）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, {'yaml_filename': map_file}],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
        ),

        # 控制器服务器
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[],  # cmd_vel will use standard topic
            condition=IfCondition(enable_nav2)
        ),

        # 规划器服务器
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            condition=IfCondition(enable_nav2)
        ),

        # 行为服务器
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            condition=IfCondition(enable_nav2)
        ),

        # 行为树导航器
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            condition=IfCondition(enable_nav2)
        ),

        # 航点跟随器
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            condition=IfCondition(enable_nav2)
        ),

        # 速度平滑器
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[('/cmd_vel_smoothed', '/cmd_vel')],
            condition=IfCondition(enable_nav2)
        ),

        # 生命周期管理器
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother'
                ] + (['amcl'] if mode == 'localization' else [])
            }],
            condition=IfCondition(enable_nav2)
        ),
    ])

    # RViz节点（Nav2配置）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        arguments=['-d', rviz_config] if rviz_config != '' else [],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 创建launch描述
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_sensor_mode_cmd)
    ld.add_action(declare_enable_nav2_cmd)

    # 添加日志信息
    ld.add_action(LogInfo(msg=[
        '========================================\n',
        'MegaRover3 SLAM + Navigation System\n',
        '========================================\n',
        'Mode: ', mode, '\n',
        'Nav2: ', enable_nav2, '\n',
        'Sensor Mode: ', sensor_mode, '\n',
        '========================================\n'
    ]))

    # 添加节点组
    ld.add_action(rtabmap_group)

    # 延迟启动Nav2以确保RTABMAP先启动
    ld.add_action(TimerAction(
        period=5.0,
        actions=[nav2_nodes]
    ))

    # 添加RViz
    ld.add_action(rviz_node)

    return ld