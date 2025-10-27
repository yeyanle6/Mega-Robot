#!/usr/bin/env python3
"""
模块化RTABMAP SLAM Launch文件

自动检测可用传感器并选择合适的配置：
- Fusion模式: MID360 + D455
- LiDAR模式: 仅MID360
- RGB-D模式: 仅D455
- 降级模式: 仅里程计
"""

import os
import yaml
import time
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import subprocess

def check_sensor_hardware():
    """检查传感器硬件"""
    sensors = {
        'livox_mid360': False,
        'realsense_d455': False
    }

    # 检查Livox MID360
    try:
        usb_devices = subprocess.check_output(['ls', '/dev/'], text=True)
        sensors['livox_mid360'] = 'ttyUSB' in usb_devices
    except:
        pass

    # 检查RealSense D455
    try:
        lsusb = subprocess.check_output(['lsusb'], text=True)
        sensors['realsense_d455'] = 'Intel' in lsusb and 'RealSense' in lsusb
    except:
        pass

    return sensors

def determine_slam_mode(sensors):
    """根据可用传感器确定SLAM模式"""
    if sensors['livox_mid360'] and sensors['realsense_d455']:
        return 'fusion', 'rtabmap_fusion.yaml'
    elif sensors['livox_mid360']:
        return 'lidar_only', 'rtabmap_lidar_only.yaml'
    elif sensors['realsense_d455']:
        return 'rgbd_only', 'rtabmap_rgbd_only.yaml'
    else:
        return 'odometry_only', 'rtabmap_odom_only.yaml'

def launch_setup(context, *args, **kwargs):
    """动态设置launch配置"""

    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    auto_mode = LaunchConfiguration('auto_mode')
    force_mode = LaunchConfiguration('force_mode')
    enable_monitoring = LaunchConfiguration('enable_monitoring')

    # 检查硬件
    sensors = check_sensor_hardware()

    # 确定SLAM模式
    if force_mode.perform(context) != 'auto':
        mode = force_mode.perform(context)
        config_map = {
            'fusion': 'rtabmap_fusion.yaml',
            'lidar_only': 'rtabmap_lidar_only.yaml',
            'rgbd_only': 'rtabmap_rgbd_only.yaml',
            'odometry_only': 'rtabmap_odom_only.yaml'
        }
        config_file = config_map.get(mode, 'rtabmap_fusion.yaml')
    else:
        mode, config_file = determine_slam_mode(sensors)

    # 日志输出
    log_msg = f"""
    ========================================
    RTABMAP模块化SLAM系统
    ========================================
    检测到的传感器:
      - Livox MID360: {'✓' if sensors['livox_mid360'] else '✗'}
      - RealSense D455: {'✓' if sensors['realsense_d455'] else '✗'}

    选择的SLAM模式: {mode}
    配置文件: {config_file}
    ========================================
    """

    nodes = []

    # 添加日志信息
    nodes.append(LogInfo(msg=log_msg))

    # 机器人底盘（始终启动）
    nodes.append(
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
        )
    )

    # 里程计融合节点（融合轮式里程计和IMU）
    nodes.append(
        Node(
            package='megarover_navigation',
            executable='odometry_fusion.py',
            name='odometry_fusion',
            output='screen',
            parameters=[{
                'wheel_odom_topic': '/rover_odo',
                'imu_topic': '/livox/imu' if sensors['livox_mid360'] else '/camera/imu',
                'output_odom_topic': '/odom',
                'publish_tf': True,
                'wheel_weight': 0.6,
                'imu_weight': 0.4,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom'
            }]
        )
    )

    # 传感器监控节点
    if enable_monitoring.perform(context) == 'true':
        nodes.append(
            Node(
                package='megarover_navigation',
                executable='sensor_detector.py',
                name='sensor_detector',
                output='screen',
                parameters=[{
                    'check_interval': 2.0,
                    'auto_restart': True
                }]
            )
        )

    # Livox MID360（条件启动）
    if sensors['livox_mid360'] and mode in ['fusion', 'lidar_only']:
        # Livox驱动节点
        nodes.append(
            Node(
                package='livox_ros_driver2',
                executable='livox_ros_driver2_node',
                name='livox_lidar_publisher',
                output='screen',
                parameters=[{
                    'xfer_format': 0,
                    'multi_topic': 0,
                    'data_src': 0,
                    'publish_freq': 10.0,
                    'output_data_type': 0,
                    'frame_id': 'mid360_lidar',  # 与URDF一致，点云和IMU都使用此frame
                    'user_config_path': os.path.join(
                        get_package_share_directory('livox_ros_driver2'),
                        'config',
                        'MID360_config.json'
                    )
                }],
                respawn=True,
                respawn_delay=5
            )
        )

        # 注意：Livox驱动默认发布到 'livox/lidar' 和 'livox/imu' (相对话题)
        # 由于我们使用的是全局话题名，不需要额外的remapping

        # 添加IMU frame的静态TF变换
        # mid360_imu与mid360_lidar位置重合，但URDF定义了独立的frame
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='mid360_imu_broadcaster',
                arguments=['0', '0', '0', '0', '0', '0',
                          'mid360_lidar', 'mid360_imu']
            )
        )

    # RealSense D455（条件启动）
    if sensors['realsense_d455'] and mode in ['fusion', 'rgbd_only']:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    )
                ),
                launch_arguments={
                    'camera_name': 'd455',
                    'camera_namespace': 'camera',
                    'serial_no': "''",
                    'depth_module.profile': '640x480x30',
                    'rgb_camera.profile': '640x480x30',
                    'enable_depth': 'true',
                    'enable_color': 'true',
                    'enable_infra1': 'false',
                    'enable_infra2': 'false',
                    'enable_imu': 'true',
                    'enable_gyro': 'true',
                    'enable_accel': 'true',
                    'align_depth.enable': 'true',
                    'initial_reset': 'true',
                    'use_sim_time': use_sim_time
                }.items()
            )
        )

    # 点云转换节点（仅激光雷达模式）
    if mode in ['fusion', 'lidar_only'] and sensors['livox_mid360']:
        nodes.append(
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.01,
                    'min_height': -0.5,
                    'max_height': 1.0,
                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'angle_increment': 0.00872665,
                    'scan_time': 0.1,
                    'range_min': 0.2,
                    'range_max': 20.0,
                    'use_inf': True
                }],
                remappings=[
                    ('cloud_in', '/livox/lidar'),
                    ('scan', '/scan')
                ]
            )
        )

    # 加载对应配置文件
    package_share_dir = get_package_share_directory('megarover_navigation')
    config_path = os.path.join(package_share_dir, 'config', config_file)

    # ICP Odometry节点（仅在lidar_only模式下）
    if mode == 'lidar_only':
        icp_odom_params = {
            'frame_id': 'base_link',  # 机器人base frame，TF: icp_odom → base_link
            'odom_frame_id': 'icp_odom',
            'wait_for_transform': 0.2,
            'expected_update_rate': 15.0,
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/VoxelSize': '0.1',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
            'Icp/CorrespondenceRatio': '0.01'
        }

        # 暂时禁用IMU（需要正确的TF树）
        # if sensors['livox_mid360']:
        #     icp_odom_params['wait_imu_to_init'] = True

        nodes.append(
            Node(
                package='rtabmap_odom',
                executable='icp_odometry',
                name='icp_odometry',
                output='screen',
                parameters=[icp_odom_params],
                remappings=[
                    ('scan_cloud', '/livox/lidar'),
                    # IMU暂时禁用
                    # ('imu', '/livox/imu' if sensors['livox_mid360'] else '')
                ],
                respawn=True,
                respawn_delay=5
            )
        )

    # RTABMAP节点话题映射配置
    # 注意：这些remappings会覆盖yaml配置文件中的*_topic参数
    rtabmap_remappings = [
        # RGB-D相机话题 (fusion和rgbd_only模式)
        ('rgb/image', '/camera/color/image_raw'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),

        # 激光雷达话题
        ('scan_cloud', '/livox/lidar'),  # 订阅Livox原始3D点云
        ('scan', '/scan'),               # 订阅2D激光扫描（由pointcloud_to_laserscan转换）

        # IMU话题（根据可用传感器选择）
        ('imu', '/livox/imu' if sensors['livox_mid360'] else '/camera/imu')
    ]

    # 里程计话题映射（根据模式选择）
    if mode == 'lidar_only':
        # lidar_only模式：使用ICP odometry
        rtabmap_remappings.append(('odom', 'icp_odom'))
    else:
        # 其他模式：使用轮式里程计（已融合IMU）
        rtabmap_remappings.append(('odom', '/odom'))

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[config_path],
        remappings=rtabmap_remappings,
        respawn=True,
        respawn_delay=5
    )
    nodes.append(rtabmap_node)

    # RGB-D同步节点（仅在有相机时）
    if sensors['realsense_d455'] and mode in ['fusion', 'rgbd_only']:
        nodes.append(
            Node(
                package='rtabmap_sync',
                executable='rgbd_sync',
                name='rgbd_sync',
                output='screen',
                parameters=[{
                    'approx_sync': False,
                    'queue_size': 10,
                    'depth_scale': 1.0
                }],
                remappings=[
                    ('rgb/image', '/camera/color/image_raw'),
                    ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('rgbd_image', '/rtabmap/rgbd_image')
                ]
            )
        )

    # RTABMAP专用可视化（推荐）
    if rviz.perform(context) == 'true':
        # rtabmap_viz需要订阅点云数据才能显示
        rtabmap_viz_params = [
            config_path,
            {
                'subscribe_scan_cloud': True,
                'subscribe_odom_info': True,
            }
        ]

        # rtabmap_viz需要订阅icp_odometry过滤后的点云，而不是原始点云
        rtabmap_viz_remappings = [
            ('odom', 'icp_odom' if mode == 'lidar_only' else '/odom'),
            ('scan_cloud', 'odom_filtered_input_scan'),  # icp_odometry输出的过滤点云
            ('imu', '/livox/imu' if sensors['livox_mid360'] else '/camera/imu')
        ]

        nodes.append(
            Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                name='rtabmap_viz',
                output='screen',
                parameters=rtabmap_viz_params,
                remappings=rtabmap_viz_remappings
            )
        )

    # 健康检查节点
    nodes.append(
        Node(
            package='megarover_navigation',
            executable='health_monitor.py',
            name='health_monitor',
            output='screen',
            parameters=[{
                'mode': mode,
                'check_interval': 5.0,
                'restart_on_failure': True
            }]
        )
    )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='启动RViz可视化'
        ),
        DeclareLaunchArgument(
            'auto_mode',
            default_value='true',
            description='自动选择SLAM模式'
        ),
        DeclareLaunchArgument(
            'force_mode',
            default_value='auto',
            description='强制使用特定模式: auto/fusion/lidar_only/rgbd_only/odometry_only'
        ),
        DeclareLaunchArgument(
            'enable_monitoring',
            default_value='true',
            description='启用传感器监控'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='RTABMAP数据库路径'
        ),

        # 执行动态设置
        OpaqueFunction(function=launch_setup)
    ])