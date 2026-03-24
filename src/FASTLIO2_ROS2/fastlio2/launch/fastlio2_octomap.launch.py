"""
FASTLIO2 + Patchwork++ + OctoMap 集成启动文件

功能：
1. 启动FASTLIO2激光惯性里程计节点
2. 启动Patchwork++地面分割节点 (智能地面过滤，保护低矮障碍物如围栏)
3. 启动OctoMap Server节点，将3D点云转换为3D体素地图
4. OctoMap自动提取并发布2D占用栅格地图
5. 发布必要的静态TF

数据流：
FASTLIO2 → /body_cloud (base_link坐标系) → Patchwork++ → /patchworkpp/nonground → OctoMap → /map
注意：使用 body_cloud 而非 world_cloud，确保 OctoMap 的 sensor origin 正确（用于射线清除）

依赖：
  sudo apt install ros-humble-octomap-server ros-humble-octomap-msgs

Usage:
  ros2 launch fastlio2 fastlio2_octomap.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    fastlio2_dir = get_package_share_directory('fastlio2')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='true')
    
    # Config file paths
    fastlio2_config = os.path.join(fastlio2_dir, 'config', 'lio_megarover.yaml')
    octomap_config = os.path.join(fastlio2_dir, 'config', 'octomap_megarover.yaml')
    
    # RViz config
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),

        # Static TF: base_link -> base_footprint (identity transform)
        # 用于导航系统，base_footprint是底盘中心在地面的投影
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),

        # Static TF: base_link -> livox_frame
        # 位置: x=0, y=0.09 (前方9cm), z=0.56 (高度56cm)
        # 旋转: 水平安装 | 备用 pitch: 0.5236 (30°前倾), 0.4683 (26.83°前倾)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox',
            arguments=['--x', '0', '--y', '0.09', '--z', '0.56',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
            output='screen'
        ),

        # FASTLIO2 LIO Node
        # 发布 /body_cloud (base_link坐标系) 和 /world_cloud (odom坐标系)
        # 发布 /odom (Odometry) 和 TF: odom -> base_link
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

        # Map -> Odom static transform (for SLAM mode)
        # 注意: 如果使用PGO或Localizer，它们会发布map->odom的TF，这个静态TF会被覆盖
        # 在纯SLAM模式下，map和odom是同一个坐标系
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # Patchwork++ 地面分割节点
        # 订阅 /body_cloud (base_link坐标系)，确保 OctoMap 能正确计算 sensor origin
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
                'base_frame': 'base_link',  # 与输入点云的 frame_id 一致
                # Patchwork++ 参数
                'sensor_height': 0.56,      # MID360 安装高度 | Previous: 0.60
                'num_iter': 3,              # PCA迭代次数
                'num_lpr': 20,              # 最低点代表数量
                'num_min_pts': 10,          # 最小点数 (默认值)
                'th_seeds': 0.125,          # 种子点阈值 (默认值)
                'th_dist': 0.125,           # 地面厚度阈值 (默认值)
                'th_seeds_v': 0.25,         # 垂直结构种子阈值
                'th_dist_v': 0.1,           # 垂直结构厚度阈值 (默认值)
                'max_range': 30.0,          # 最大范围 (与FASTLIO2一致)
                'min_range': 0.5,           # 最小范围 (室内需降低)
                'uprightness_thr': 0.707,   # 垂直度阈值 (默认值)
                'verbose': False,
            }],
        ),

        # OctoMap Server2 Node (源码编译版)
        # 功能：
        # 1. 订阅 /patchworkpp/nonground (地面分割后的非地面点云)
        # 2. 通过 TF (odom->base_link) 获取正确的 sensor origin 用于射线清除
        # 3. 构建3D OctoMap（体素占用地图）
        # 4. 发布2D占用栅格地图到 /projected_map
        Node(
            package='octomap_server2',
            executable='octomap_server',
            name='octomap_server',
            output='screen',
            remappings=[
                ('cloud_in', '/patchworkpp/nonground'),
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'odom',
                'base_frame_id': 'base_link',
                'resolution': 0.05,
                'sensor_model/max_range': 30.0,
                'sensor_model/hit': 0.7,
                'sensor_model/miss': 0.4,
                'sensor_model/min': 0.12,
                'sensor_model/max': 0.97,
                'filter_ground': False,
                'compress_map': True,
                'incremental_2D_projection': False,
                'height_map': True,
            }],
        ),

        # RViz for visualization
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])





