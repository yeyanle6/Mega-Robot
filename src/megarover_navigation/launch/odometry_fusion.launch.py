#!/usr/bin/env python3
"""
里程计融合节点启动文件
融合轮式里程计和IMU数据
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述"""

    # 声明启动参数
    wheel_odom_topic_arg = DeclareLaunchArgument(
        'wheel_odom_topic',
        default_value='/rover_odo',
        description='轮式里程计话题'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/livox/imu',
        description='IMU话题'
    )

    output_odom_topic_arg = DeclareLaunchArgument(
        'output_odom_topic',
        default_value='/odom_fused',
        description='融合后里程计输出话题'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='是否发布TF变换'
    )

    wheel_weight_arg = DeclareLaunchArgument(
        'wheel_weight',
        default_value='0.6',
        description='轮式里程计权重 (0.0-1.0)'
    )

    imu_weight_arg = DeclareLaunchArgument(
        'imu_weight',
        default_value='0.4',
        description='IMU权重 (0.0-1.0)'
    )

    # 获取启动配置
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    output_odom_topic = LaunchConfiguration('output_odom_topic')
    publish_tf = LaunchConfiguration('publish_tf')
    wheel_weight = LaunchConfiguration('wheel_weight')
    imu_weight = LaunchConfiguration('imu_weight')

    # 里程计融合节点
    odometry_fusion_node = Node(
        package='megarover_navigation',
        executable='odometry_fusion.py',
        name='odometry_fusion',
        output='screen',
        parameters=[{
            'wheel_odom_topic': wheel_odom_topic,
            'imu_topic': imu_topic,
            'output_odom_topic': output_odom_topic,
            'publish_tf': publish_tf,
            'wheel_weight': wheel_weight,
            'imu_weight': imu_weight,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
        }]
    )

    return LaunchDescription([
        # 参数声明
        wheel_odom_topic_arg,
        imu_topic_arg,
        output_odom_topic_arg,
        publish_tf_arg,
        wheel_weight_arg,
        imu_weight_arg,

        # 节点
        odometry_fusion_node,
    ])
