#!/usr/bin/env python3
"""
RTABMAP LiDAR SLAM Launch文件 (基于官方例程重构)

核心改进：
1. ✅ 添加Lidar Deskewing (去畸变) - 消除运动畸变
2. ✅ 修正frame_id为mid360_lidar - 遵循官方设计
3. ✅ 修正ICP参数 - MaxCorrespondenceDistance = 1.0
4. ✅ 添加IMU stabilized frame - 提高ICP初值估计
5. ✅ 简化架构 - 移除不必要的里程计融合节点

适配硬件：
- 底盘: MegaRover Ver.3.0 (micro-ROS)
- 激光雷达: Livox Mid-360 (非重复扫描)
- IMU: Mid-360内置IMU

参考：
- rtabmap_ros/rtabmap_examples/launch/lidar3d.launch.py
"""

import os
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context: LaunchContext, *args, **kwargs):
    """动态launch设置"""

    # ========================================
    # Launch参数读取
    # ========================================
    frame_id = LaunchConfiguration('frame_id')
    frame_id_value = frame_id.perform(context)

    imu_topic = LaunchConfiguration('imu_topic')
    imu_used = imu_topic.perform(context) != ''

    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_value = float(voxel_size.perform(context))

    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_topic_value = lidar_topic.perform(context)
    lidar_topic_deskewed = lidar_topic_value + "/deskewed"

    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'true' or localization == 'True'

    deskewing = LaunchConfiguration('deskewing').perform(context)
    deskewing = deskewing == 'true' or deskewing == 'True'

    deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
    deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'

    rviz = LaunchConfiguration('rviz').perform(context)
    rviz = rviz == 'true' or rviz == 'True'

    # ========================================
    # 自动创建IMU stabilized frame
    # ========================================
    fixed_frame_from_imu = False
    fixed_frame_id = LaunchConfiguration('fixed_frame_id').perform(context)
    if not fixed_frame_id and imu_used:
        fixed_frame_from_imu = True
        fixed_frame_id = frame_id_value + "_stabilized"  # e.g., "mid360_lidar_stabilized"

    # 如果没有fixed_frame_id或不做去畸变，直接使用原始点云
    if not fixed_frame_id or not deskewing:
        lidar_topic_deskewed = lidar_topic

    # ========================================
    # ICP参数计算 (官方规则: max_correspondence = voxel_size * 10.0)
    # ========================================
    max_correspondence_distance = voxel_size_value * 10.0

    # ========================================
    # 共享参数 (icp_odometry和rtabmap都使用)
    # ========================================
    shared_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': frame_id,  # ✅ 修正：使用激光雷达frame (mid360_lidar)
        'qos': LaunchConfiguration('qos'),
        'approx_sync': False,  # LiDAR only模式不需要近似同步
        'wait_for_transform': 0.2,
        # RTAB-Map内部参数 (字符串格式)
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/VoxelSize': str(voxel_size_value),
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),  # ✅ 修正：1.0而不是0.15
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',  # ✅ 修正：0.7而不是0.65
    }

    # ========================================
    # ICP Odometry参数
    # ========================================
    icp_odometry_parameters = {
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),
        'deskewing': not fixed_frame_id and deskewing,  # 如果外部去畸变，此处设为False
        'odom_frame_id': 'icp_odom',
        'guess_frame_id': fixed_frame_id,  # ✅ 关键：使用IMU stabilized frame作为初值
        'deskewing_slerp': deskewing_slerp,
        # 里程计特定参数
        'Odom/ScanKeyFrameThr': '0.4',
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': 'false',
        'Icp/CorrespondenceRatio': '0.01'  # 里程计对应点比例要求低
    }
    if imu_used:
        icp_odometry_parameters['wait_imu_to_init'] = True  # 等待IMU初始化

    # ========================================
    # RTABMAP参数
    # ========================================
    rtabmap_parameters = {
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_odom_info': True,  # 订阅icp_odometry的info
        'subscribe_scan_cloud': True,  # 订阅3D点云
        'map_frame_id': 'rtabmap_map',  # ✅ 避免与其他SLAM冲突
        'odom_sensor_sync': False,  # LiDAR only不需要
        # RTAB-Map核心参数 (官方推荐)
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/CreateOccupancyGrid': 'false',  # 不创建栅格地图（由Nav2处理）
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Reg/Strategy': '1',  # ICP
        'Icp/CorrespondenceRatio': str(LaunchConfiguration('min_loop_closure_overlap').perform(context))  # 回环检测对应点比例
    }

    # 建图模式 vs 定位模式
    arguments = []
    if localization:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d')  # 删除旧数据库，重新建图

    # ========================================
    # 话题映射
    # ========================================
    remappings = [('odom', 'icp_odom')]
    if imu_used:
        remappings.append(('imu', LaunchConfiguration('imu_topic')))
    else:
        remappings.append(('imu', 'imu_not_used'))

    # ========================================
    # 节点列表
    # ========================================
    nodes = []

    # 日志信息
    log_msg = f"""
    ========================================
    RTABMAP LiDAR SLAM (官方架构重构版)
    ========================================
    核心改进:
      ✅ Lidar Deskewing: {deskewing}
      ✅ frame_id: {frame_id_value}
      ✅ guess_frame_id: {fixed_frame_id if fixed_frame_id else 'None'}
      ✅ MaxCorrespondenceDistance: {max_correspondence_distance}
      ✅ 简化架构 (移除odometry_fusion)

    传感器配置:
      - Livox Mid-360: {lidar_topic_value}
      - IMU: {imu_topic.perform(context) if imu_used else 'Disabled'}
      - 去畸变点云: {lidar_topic_deskewed}

    模式: {'定位' if localization else '建图'}
    ========================================
    """
    nodes.append(LogInfo(msg=log_msg))

    # ========================================
    # 1. MegaRover3底盘驱动
    # ========================================
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

    # ========================================
    # 2. Livox Mid-360驱动
    # ========================================
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
                'frame_id': frame_id_value,  # mid360_lidar
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

    # ========================================
    # 3. ✅ IMU to TF节点 (创建stabilized frame)
    # ========================================
    if fixed_frame_from_imu:
        nodes.append(
            Node(
                package='rtabmap_util',
                executable='imu_to_tf',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,  # mid360_lidar_stabilized
                    'base_frame_id': frame_id,  # mid360_lidar
                    'wait_for_transform_duration': 0.001
                }],
                remappings=[('imu/data', imu_topic)]
            )
        )

    # ========================================
    # 4. ✅ Lidar Deskewing节点 (去畸变)
    # ========================================
    if fixed_frame_id and deskewing:
        nodes.append(
            Node(
                package='rtabmap_util',
                executable='lidar_deskewing',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,
                    'wait_for_transform': 0.2,
                    'slerp': deskewing_slerp
                }],
                remappings=[
                    ('input_cloud', lidar_topic),
                    ('deskewed', lidar_topic_deskewed)  # 关键修复：输出话题映射
                ]
            )
        )

    # ========================================
    # 5. ICP Odometry节点
    # ========================================
    nodes.append(
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[shared_parameters, icp_odometry_parameters],
            remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
            respawn=True,
            respawn_delay=5
        )
    )

    # ========================================
    # 6. RTABMAP核心节点
    # ========================================
    nodes.append(
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[shared_parameters, rtabmap_parameters],
            remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
            arguments=arguments,
            respawn=True,
            respawn_delay=5
        )
    )

    # ========================================
    # 7. RTABMAP可视化 (可选)
    # ========================================
    if rviz:
        nodes.append(
            Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                output='screen',
                parameters=[shared_parameters, rtabmap_parameters],
                remappings=remappings + [('scan_cloud', 'odom_filtered_input_scan')]
            )
        )

    # ========================================
    # 8. 点云转2D激光扫描 (用于Nav2局部避障)
    # ========================================
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
                ('cloud_in', lidar_topic_deskewed),  # 使用去畸变后的点云
                ('scan', '/scan')
            ]
        )
    )

    return nodes


def generate_launch_description():
    """生成Launch描述"""
    return LaunchDescription([
        # ========================================
        # Launch参数声明
        # ========================================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),

        DeclareLaunchArgument(
            'deskewing',
            default_value='true',
            description='启用LiDAR去畸变 (强烈推荐)'
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value='mid360_lidar',  # ✅ 修正：激光雷达frame
            description='激光雷达base frame'
        ),

        DeclareLaunchArgument(
            'fixed_frame_id',
            default_value='',
            description='固定frame用于去畸变。如果为空，将从IMU自动生成'
        ),

        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='定位模式 (true) 或建图模式 (false)'
        ),

        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/livox/lidar',
            description='LiDAR点云话题'
        ),

        DeclareLaunchArgument(
            'imu_topic',
            default_value='/livox/imu',
            description='IMU话题 (如果为空则禁用IMU)'
        ),

        DeclareLaunchArgument(
            'expected_update_rate',
            default_value='15.0',
            description='LiDAR期望帧率 (Hz)。建议设置略高于实际帧率'
        ),

        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.1',
            description='体素大小 (m)。室内: 0.1-0.3, 室外: 0.5+'
        ),

        DeclareLaunchArgument(
            'min_loop_closure_overlap',
            default_value='0.2',
            description='回环检测最小扫描重叠比例'
        ),

        DeclareLaunchArgument(
            'deskewing_slerp',
            default_value='true',
            description='使用快速slerp插值进行去畸变 (推荐)'
        ),

        DeclareLaunchArgument(
            'qos',
            default_value='1',
            description='QoS策略: 0=系统默认, 1=可靠, 2=尽力而为'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='启动RTABMAP可视化'
        ),

        # 执行动态设置
        OpaqueFunction(function=launch_setup)
    ])
