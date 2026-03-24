"""
FASTLIO2 + PGO + Localizer + MegaRover3 Navigation Launch File

支持三种模式：
1. slam - FAST-LIO2 + PGO (建图 + 回环检测 + 优化)
2. nav  - Localizer (基于预建点云地图定位) + Nav2
3. slam_simple - FAST-LIO2 简单SLAM (不用PGO，原始模式)

建图流程（slam模式）：
  MID360 → FAST-LIO2 → PGO (回环检测) → 优化后的点云地图
                     ↓
                  OctoMap → 2D栅格地图

导航流程（nav模式）：
  加载点云地图 → Localizer (scan-to-map定位) → 精确定位
  加载2D地图 → Nav2 → 路径规划

使用方法：
  # 建图模式（带PGO优化）:
  ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam

  # 保存地图：
  ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '$(pwd)/maps/my_pgo_map', save_patches: true}"

  # 导航模式（基于预建地图）:
  ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=nav \
    pcd_map:=/path/to/map.pcd map:=/path/to/map.yaml

  # 如果 MID-360 已由其他终端或控制面板启动，避免重复启动:
  ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam \
    start_lidar:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    megarover3_nav_dir = get_package_share_directory('megarover3_navigation')
    megarover_desc_dir = get_package_share_directory('megarover_description')
    fastlio2_dir = get_package_share_directory('fastlio2')
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mode = LaunchConfiguration('mode', default='slam')
    map_yaml = LaunchConfiguration('map', default='')
    pcd_map = LaunchConfiguration('pcd_map', default='')
    rviz = LaunchConfiguration('rviz', default='true')
    octomap = LaunchConfiguration('octomap', default='true')
    start_lidar = LaunchConfiguration('start_lidar', default='true')
    enable_bumper_safety = LaunchConfiguration('enable_bumper_safety', default='true')
    bumper_trigger_mask = LaunchConfiguration('bumper_trigger_mask', default='65535')
    bumper_stop_hold = LaunchConfiguration('bumper_stop_hold', default='0.3')
    bumper_retreat_distance = LaunchConfiguration('bumper_retreat_distance', default='0.05')
    bumper_retreat_speed = LaunchConfiguration('bumper_retreat_speed', default='0.05')
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='0.0')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='0.0')
    initial_pose_z = LaunchConfiguration('initial_pose_z', default='0.0')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='0.0')
    use_last_pose = LaunchConfiguration('use_last_pose', default='false')

    # Config files
    robot_description_path = os.path.join(megarover_desc_dir, 'urdf', 'mega3.xacro')
    fastlio2_config = os.path.join(fastlio2_dir, 'config', 'lio_megarover.yaml')
    default_nav2_params = os.path.join(megarover3_nav_dir, 'config', 'fastlio2_nav2_params.yaml')
    nav2_params = LaunchConfiguration('nav2_params_file', default=default_nav2_params)
    rviz_config = os.path.join(megarover3_nav_dir, 'rviz', 'fastlio2_nav.rviz')

    # PGO and Localizer configs
    pgo_config = PathJoinSubstitution([FindPackageShare("pgo"), "config", "pgo.yaml"])
    localizer_config = PathJoinSubstitution([FindPackageShare("localizer"), "config", "localizer.yaml"])

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
            choices=['slam', 'nav', 'slam_simple'],
            description='Mode: slam (PGO mapping), nav (localization), slam_simple (no PGO)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to 2D map yaml file (required for nav mode)'
        ),
        DeclareLaunchArgument(
            'pcd_map',
            default_value='',
            description='Full path to point cloud map file (required for nav mode)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Full path to Nav2 params yaml file for A/B testing'
        ),
        DeclareLaunchArgument(
            'octomap',
            default_value='true',
            description='Launch OctoMap server'
        ),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Launch the MID-360 Livox driver as part of this main entrypoint'
        ),
        DeclareLaunchArgument(
            'enable_bumper_safety',
            default_value='true',
            description='Latch-stop navigation when /rover_sensor.data[0] indicates a bumper trigger'
        ),
        DeclareLaunchArgument(
            'bumper_trigger_mask',
            default_value='65535',
            description='Bit mask applied to /rover_sensor.data[0] (MU16_IM_DI)'
        ),
        DeclareLaunchArgument(
            'bumper_stop_hold',
            default_value='0.3',
            description='Seconds to hold zero velocity after canceling navigation before retreating'
        ),
        DeclareLaunchArgument(
            'bumper_retreat_distance',
            default_value='0.05',
            description='Retreat distance in meters after a bumper trigger'
        ),
        DeclareLaunchArgument(
            'bumper_retreat_speed',
            default_value='0.05',
            description='Retreat speed in m/s after a bumper trigger'
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial X position for nav mode'
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial Y position for nav mode'
        ),
        DeclareLaunchArgument(
            'initial_pose_z',
            default_value='0.0',
            description='Initial Z position for nav mode'
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial yaw (radians) for nav mode'
        ),
        DeclareLaunchArgument(
            'use_last_pose',
            default_value='false',
            description='Use last saved pose on startup (nav mode)'
        ),

        # ============================================================
        # Optional MID-360 driver startup
        # Default on so this launch can be used as a standalone entrypoint.
        # Set start_lidar:=false when the driver is already running elsewhere.
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_driver_dir, 'launch_ROS2', 'msg_MID360_launch.py')
            ),
            condition=IfCondition(start_lidar),
        ),

        # ============================================================
        # Robot Description (URDF → TF)
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
        # FAST-LIO2 publishes odom -> lio_base. In lio_base frame,
        # +Y = physical forward (+X in base_footprint/ROS convention).
        # This TF corrects the 90° rotation so Nav2 sees +X = forward.
        # Translation (0, 0.1, 0) accounts for the URDF base_joint offset.
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
        # SLAM Mode: FAST-LIO2 + PGO
        # ============================================================
        GroupAction(
            condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
            actions=[
                # FAST-LIO2 LIO Node
                Node(
                    package='fastlio2',
                    executable='lio_node',
                    name='fastlio2_lio',
                    namespace='fastlio2',
                    output='screen',
                    parameters=[{
                        'config_path': fastlio2_config,
                        'use_sim_time': use_sim_time
                    }],
                    remappings=[
                        ('/fastlio2/body_cloud', '/body_cloud'),
                        ('/fastlio2/lio_odom', '/lio_odom'),
                    ]
                ),

                # PGO Node (回环检测 + 地图优化)
                # PGO 话题名从 pgo.yaml 配置读取（绝对路径），ROS2 remapping 无效
                Node(
                    package='pgo',
                    executable='pgo_node',
                    name='pgo_node',
                    namespace='pgo',
                    output='screen',
                    parameters=[{'config_path': pgo_config}],
                ),

                # Planar Compensation for ground robots
                Node(
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
            ]
        ),

        # ============================================================
        # Simple SLAM Mode: FAST-LIO2 only (原始模式)
        # ============================================================
        GroupAction(
            condition=IfCondition(PythonExpression(["'", mode, "' == 'slam_simple'"])),
            actions=[
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

                Node(
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
            ]
        ),

        # ============================================================
        # Navigation Mode: Localizer (基于预建点云地图)
        # ============================================================
        GroupAction(
            condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
            actions=[
                # FAST-LIO2 LIO Node
                Node(
                    package='fastlio2',
                    executable='lio_node',
                    name='fastlio2_lio',
                    namespace='fastlio2',
                    output='screen',
                    parameters=[{
                        'config_path': fastlio2_config,
                        'use_sim_time': use_sim_time
                    }],
                    remappings=[
                        ('/fastlio2/body_cloud', '/body_cloud'),
                        ('/fastlio2/lio_odom', '/lio_odom'),
                    ]
                ),

                # Localizer Node (scan-to-map 定位)
                Node(
                    # Localizer 话题名从 localizer.yaml 配置读取（绝对路径），ROS2 remapping 无效
                    package='localizer',
                    executable='localizer_node',
                    name='localizer_node',
                    namespace='localizer',
                    output='screen',
                    parameters=[{'config_path': localizer_config}],
                ),

                # Nav Initializer (calls relocalize service to load PCD map)
                Node(
                    package='megarover3_navigation',
                    executable='nav_initializer.py',
                    name='nav_initializer',
                    output='screen',
                    parameters=[{
                        'pcd_map_path': pcd_map,
                        'initial_x': initial_pose_x,
                        'initial_y': initial_pose_y,
                        'initial_z': initial_pose_z,
                        'initial_yaw': initial_pose_yaw,
                        'use_last_pose': use_last_pose,
                    }],
                ),

                # Map Server (2D map)
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[configured_params]
                ),

                Node(
                    package='megarover3_navigation',
                    executable='bumper_safety_monitor.py',
                    name='bumper_safety_monitor',
                    output='screen',
                    condition=IfCondition(enable_bumper_safety),
                    parameters=[{
                        'enabled': enable_bumper_safety,
                        'trigger_mask': bumper_trigger_mask,
                        'zero_cmd_rate_hz': 20.0,
                        'stop_hold_s': bumper_stop_hold,
                        'retreat_distance_m': bumper_retreat_distance,
                        'retreat_speed_mps': bumper_retreat_speed,
                    }],
                ),

            ]
        ),

        # ============================================================
        # Patchwork++ (所有模式都需要)
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
                'th_seeds': 0.05,
                'th_dist': 0.05,
                'th_seeds_v': 0.25,
                'th_dist_v': 0.1,
                'max_range': 30.0,
                'min_range': 0.5,
                'uprightness_thr': 0.707,
                'verbose': False,
            }],
        ),

        # ============================================================
        # Point Cloud Relay
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
                'costmap_min_height': 0.02,
                'costmap_max_height': 0.7,
                'costmap_sample_step': 1,
                'debug_costmap_stats': True,
                'debug_costmap_stats_interval_sec': 2.0,
                'debug_costmap_stats_file': '/tmp/pointcloud_relay_costmap_stats.log',
            }],
        ),

        # ============================================================
        # D455 Near-field Obstacle Filter
        # Extracts low-lying obstacles using distance-bin ground baseline.
        # Replaces raw D455 points in local_costmap with semantically
        # filtered obstacle-only output.
        # ============================================================
        Node(
            package='megarover3_navigation',
            executable='d455_nearfield_obstacle_filter.py',
            name='d455_nearfield_obstacle_filter',
            output='screen',
            parameters=[{
                'input_topic': '/camera/d455_front/depth/color/points',
                'output_topic': '/d455_front_obstacles',
                'target_frame': 'base_footprint',
                'throttle_factor': 2,
                'min_range': 0.2,
                'max_range': 1.2,
                'roi_x_min': 0.0,
                'roi_x_max': 1.2,
                'roi_y_abs_max': 0.45,
                'roi_z_max': 0.5,
                'ground_bin_size': 0.1,
                'ground_percentile': 10,
                'ground_margin': 0.03,
                'obstacle_min_relative_height': 0.02,
                'min_cluster_points': 8,
                'min_cluster_width': 0.06,
                'voxel_size': 0.05,
                'lidar_topic': '/patchworkpp/nonground',
                'debug': True,
            }],
        ),

        # ============================================================
        # YOLO Obstacle Detector (independent safety layer)
        # Detects transparent/reflective/thin obstacles via RGB.
        # ============================================================
        Node(
            package='megarover3_navigation',
            executable='yolo_obstacle_detector.py',
            name='yolo_obstacle_detector',
            output='screen',
            parameters=[{
                'model_path': os.path.join(
                    megarover3_nav_dir, 'models', 'yolov8n.onnx'),
                'output_topic': '/yolo_obstacles',
                'target_frame': 'base_footprint',
                'confidence_threshold': 0.30,
                'throttle_interval_sec': 0.25,
                'min_depth_m': 0.2,
                'max_depth_m': 2.5,
                'depth_roi_ratio': 0.5,
                'depth_percentile': 25,
                'block_grid_resolution': 0.05,
                'block_min_thickness': 0.10,
                'block_z_layers': [0.10, 0.20],
                'block_max_width': 1.5,
                'block_max_thickness': 0.8,
                'lidar_topic': '/patchworkpp/nonground',
                'rgb_topic': '/camera/d455_front/color/image_raw',
                'depth_topic': '/camera/d455_front/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/d455_front/color/camera_info',
                'obstacle_class_ids': [
                    0, 1, 2, 3, 5, 7,
                    8, 9, 10, 11, 12, 13, 14, 15, 16,
                    24, 25, 26, 27, 28, 31, 32, 33, 34,
                    39, 40, 41, 42, 43, 44, 45, 46,
                    56, 57, 58, 59, 60, 61, 62, 63,
                    64, 65, 66, 67, 68, 69, 70, 71,
                    72, 73, 74, 75, 76, 77, 78, 79
                ],
                'debug': True,
            }],
        ),

        # ============================================================
        # OctoMap Server (SLAM modes only)
        # ============================================================
        Node(
            condition=IfCondition(PythonExpression([
                "('", mode, "' == 'slam' or '", mode, "' == 'slam_simple') and '",
                octomap, "' == 'true'"
            ])),
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

        # Nav2 Lifecycle Manager (navigation-only modes)
        Node(
            condition=IfCondition(PythonExpression(["'", mode, "' != 'nav'"])),
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

        # Nav2 Lifecycle Manager (nav mode, includes map_server)
        Node(
            condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
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
