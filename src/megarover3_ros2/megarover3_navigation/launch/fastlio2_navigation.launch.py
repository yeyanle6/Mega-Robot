"""
FASTLIO2 + MegaRover3 Navigation Integration Launch File

LEGACY ENTRYPOINT
This file is kept for reference/backward compatibility.
For current on-robot use, prefer:
  ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py ...

This launch file combines:
1. Robot Description (URDF) for TF publishing (D455, MID360 frames)
2. FASTLIO2 for LiDAR-Inertial Odometry (SLAM/Localization)
3. Patchwork++ for ground segmentation
4. OctoMap Server2 for 3D→2D map generation
5. Nav2 for autonomous navigation

Data flow:
  MID360 → FASTLIO2 → /body_cloud → Patchwork++ → /patchworkpp/nonground ─┐
  D455 → /camera/d455_front/depth/color/points ─────────────────────────────┼→ Relay → /merged_cloud → OctoMap → /map
                                                                           └→ Relay → /d455_front_restamped → Nav2 costmap (obstacle avoidance)

Usage:
  # SLAM mode (building map):
  ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam

  # Navigation mode (with existing map):
  ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=nav pcd_map:=/path/to/map.pcd map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    megarover3_nav_dir = get_package_share_directory('megarover3_navigation')
    megarover_desc_dir = get_package_share_directory('megarover_description')
    fastlio2_dir = get_package_share_directory('fastlio2')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mode = LaunchConfiguration('mode', default='slam')
    map_yaml = LaunchConfiguration('map', default='')
    rviz = LaunchConfiguration('rviz', default='true')
    octomap = LaunchConfiguration('octomap', default='true')

    # Robot URDF (provides TF for D455, MID360, base_footprint etc.)
    robot_description_path = os.path.join(megarover_desc_dir, 'urdf', 'mega3.xacro')

    # FASTLIO2 config for MegaRover
    fastlio2_config = os.path.join(fastlio2_dir, 'config', 'lio_megarover.yaml')

    # Nav2 params for FASTLIO2 integration
    nav2_params = os.path.join(megarover3_nav_dir, 'config', 'fastlio2_nav2_params.yaml')

    # RViz config for FASTLIO2
    rviz_config = os.path.join(megarover3_nav_dir, 'rviz', 'fastlio2_nav.rviz')

    # Param substitutions for Nav2
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='slam',
            choices=['slam', 'nav'],
            description='Mode: slam (mapping) or nav (navigation with map)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file (required for nav mode)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'octomap',
            default_value='true',
            description='Launch OctoMap server (visualization only, not needed for navigation)'
        ),

        # ============================================================
        # Robot Description (URDF → TF)
        # 发布完整TF树: base_link → {base_footprint, D455系列frame, MID360系列frame, ...}
        # ============================================================
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

        # ============================================================
        # Static TF: lio_base -> base_footprint
        # Keep this legacy launch aligned with the current FAST-LIO2
        # body_frame convention in lio_megarover.yaml.
        # ============================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lio_base_to_base_footprint',
            arguments=['--x', '0', '--y', '0.1', '--z', '0',
                      '--roll', '0', '--pitch', '0', '--yaw', '1.5708',
                      '--frame-id', 'lio_base', '--child-frame-id', 'base_footprint']
        ),

        # ============================================================
        # Static TF: base_link -> livox_frame
        # FASTLIO2 使用 livox_frame（不是 URDF 中的 mid360_base）
        # MID-360 connector 朝后: livox X=左, Y=后, Z=上
        # 位置: x=0.09 (前方9cm), y=0, z=0.56 (高度56cm)
        # 旋转: yaw=+π/2 (livox X 指向 base +Y 即左侧)
        # ============================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox',
            arguments=['--x', '0.09', '--y', '0', '--z', '0.56',
                      '--roll', '0', '--pitch', '0', '--yaw', '1.5708',
                      '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']
        ),

        # ============================================================
        # FASTLIO2 LIO Node
        # 输入: /livox/lidar, /livox/imu
        # 输出: /lio_odom, /body_cloud, /world_cloud, TF odom→lio_base
        # ============================================================
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

        # ============================================================
        # Patchwork++ 地面分割
        # 输入: /body_cloud (base_link坐标系)
        # 输出: /patchworkpp/nonground (过滤地面后的点云)
        # ============================================================
        Node(
            package='patchworkpp',
            executable='patchworkpp_node',
            name='patchworkpp_node',
            output='screen',
            remappings=[
                ('pointcloud_topic', '/body_cloud'),
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',
                'sensor_height': 0.56,
                'num_iter': 3,
                'num_lpr': 20,
                'num_min_pts': 10,
                'th_seeds': 0.05,          # 地面种子阈值 | Previous: 0.125→0.07 (收紧后保留>5cm障碍物)
                'th_dist': 0.05,           # 地面厚度阈值 | Previous: 0.125→0.07 (收紧后保留>5cm障碍物)
                'th_seeds_v': 0.25,
                'th_dist_v': 0.1,
                'max_range': 30.0,
                'min_range': 0.5,
                'uprightness_thr': 0.707,
                'verbose': False,
            }],
        ),

        # ============================================================
        # Point Cloud Relay (LiDAR + D455 → merged_cloud)
        # LiDAR: direct passthrough
        # D455: throttled to ~5fps, voxel downsampled, range filtered
        # OctoMap uses header.frame_id for TF lookup, no transform needed
        # ============================================================
        Node(
            package='megarover3_navigation',
            executable='pointcloud_relay.py',
            name='pointcloud_relay',
            output='screen',
            parameters=[{
                'lidar_topic': '/patchworkpp/nonground',
                'camera_topic': '/camera/d455_front/depth/color/points',
                'output_topic': '/merged_cloud',
                'lidar_throttle_factor': 4,
                'camera_throttle_factor': 3,
                'camera_max_range': 2.0,
                'camera_voxel_size': 0.05,
                'enable_camera': True,
                'costmap_output_topic': '/d455_front_restamped',
                'costmap_throttle_factor': 1,
                'costmap_sample_step': 3,
                'debug_costmap_stats': True,
                'debug_costmap_stats_interval_sec': 2.0,
                'debug_costmap_stats_file': '/tmp/pointcloud_relay_costmap_stats.log',
            }],
        ),

        # ============================================================
        # OctoMap Server2 (源码编译版)
        # 输入: /merged_cloud (LiDAR + D455 via relay)
        # 输出: /map (2D占用栅格), /octomap_binary (3D OctoMap)
        # ============================================================
        Node(
            condition=IfCondition(octomap),
            package='octomap_server2',
            executable='octomap_server',
            name='octomap_server',
            output='screen',
            remappings=[
                ('cloud_in', '/merged_cloud'),
                ('projected_map', '/map'),
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'odom',
                'base_frame_id': 'base_link',
                'resolution': 0.10,
                'pointcloud_min_z': -0.5,
                'pointcloud_max_z': 1.2,
                'occupancy_min_z': 0.15,
                'occupancy_max_z': 1.2,
                'sensor_model/max_range': 30.0,
                'sensor_model/hit': 0.7,
                'sensor_model/miss': 0.4,
                'sensor_model/min': 0.12,
                'sensor_model/max': 0.97,
                'filter_ground': False,
                'compress_map': True,
                'incremental_2D_projection': True,
                'height_map': True,
            }],
        ),

        # ============================================================
        # Planar Compensation: map -> odom (SLAM mode only)
        # FASTLIO2は6DOF推定のためZ/Roll/Pitchがドリフト・ジッターする
        # このノードがmap→odom TFを発行し、地面ロボット用に平面拘束する
        # Z=初期値固定、Roll=0、Pitch=0、X/Y/Yawはそのまま保持
        # ============================================================
        Node(
            condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
            package='megarover3_navigation',
            executable='planar_compensation.py',
            name='planar_compensation',
            output='screen',
            parameters=[{
                'odom_topic': '/lio_odom',
                'map_frame': 'map',
                'odom_frame': 'odom',
            }],
        ),

        # Map Server (only in nav mode)
        Node(
            condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params]
        ),

        # Lifecycle manager for map_server (only in nav mode)
        Node(
            condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
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

        # ============================================================
        # Nav2 Navigation Stack
        # ============================================================
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
            parameters=[configured_params],
            remappings=[('cmd_vel_smoothed', '/rover_twist')]
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
