#!/usr/bin/env python3
"""
传感器检测和管理模块
自动检测可用传感器并配置相应的SLAM模式
"""

import os
import sys
import time
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, Imu, LaserScan
from nav_msgs.msg import Odometry
from typing import Dict, List, Optional, Tuple
import yaml
import threading

class SensorDetector(Node):
    """传感器检测节点"""

    def __init__(self):
        super().__init__('sensor_detector')

        # 传感器状态
        self.sensors = {
            'livox_mid360': {
                'hardware_available': False,
                'topic_active': False,
                'topics': ['/livox/lidar', '/livox/imu'],
                'last_msg_time': None,
                'timeout': 5.0,
                'type': 'lidar'
            },
            'realsense_d455': {
                'hardware_available': False,
                'topic_active': False,
                'topics': [
                    '/camera/color/image_raw',
                    '/camera/depth/image_rect_raw',
                    '/camera/color/camera_info'
                ],
                'last_msg_time': None,
                'timeout': 5.0,
                'type': 'rgbd'
            },
            'wheel_odometry': {
                'hardware_available': True,  # 假设始终可用
                'topic_active': False,
                'topics': ['/odom'],
                'last_msg_time': None,
                'timeout': 2.0,
                'type': 'odometry'
            }
        }

        # 配置模式
        self.slam_modes = {
            'lidar_only': {
                'sensors': ['livox_mid360', 'wheel_odometry'],
                'config': 'rtabmap_lidar_only.yaml'
            },
            'rgbd_only': {
                'sensors': ['realsense_d455', 'wheel_odometry'],
                'config': 'rtabmap_rgbd_only.yaml'
            },
            'fusion': {
                'sensors': ['livox_mid360', 'realsense_d455', 'wheel_odometry'],
                'config': 'rtabmap_fusion.yaml'
            },
            'odometry_only': {
                'sensors': ['wheel_odometry'],
                'config': 'rtabmap_odom_only.yaml'
            }
        }

        # 当前模式
        self.current_mode = None
        self.mode_lock = threading.Lock()

        # 创建订阅者
        self.create_subscriptions()

        # 硬件检测定时器
        self.hardware_check_timer = self.create_timer(2.0, self.check_hardware)

        # 话题活动检测定时器
        self.activity_check_timer = self.create_timer(1.0, self.check_topic_activity)

        # 模式更新定时器
        self.mode_update_timer = self.create_timer(3.0, self.update_slam_mode)

        # 状态发布定时器
        self.status_timer = self.create_timer(5.0, self.publish_status)

        self.get_logger().info('传感器检测器已启动')

    def create_subscriptions(self):
        """创建话题订阅者"""
        # Livox订阅
        self.sub_livox_cloud = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            lambda msg: self.update_sensor_activity('livox_mid360'),
            1
        )

        self.sub_livox_imu = self.create_subscription(
            Imu,
            '/livox/imu',
            lambda msg: self.update_sensor_activity('livox_mid360'),
            1
        )

        # RealSense订阅
        self.sub_camera_rgb = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            lambda msg: self.update_sensor_activity('realsense_d455'),
            1
        )

        self.sub_camera_depth = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            lambda msg: self.update_sensor_activity('realsense_d455'),
            1
        )

        # 里程计订阅
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            lambda msg: self.update_sensor_activity('wheel_odometry'),
            1
        )

    def update_sensor_activity(self, sensor_name: str):
        """更新传感器活动状态"""
        if sensor_name in self.sensors:
            self.sensors[sensor_name]['last_msg_time'] = time.time()
            if not self.sensors[sensor_name]['topic_active']:
                self.sensors[sensor_name]['topic_active'] = True
                self.get_logger().info(f'{sensor_name} 话题已激活')

    def check_hardware(self):
        """检查硬件可用性"""
        # 检查Livox MID360
        try:
            usb_devices = subprocess.check_output(['ls', '/dev/'], text=True)
            self.sensors['livox_mid360']['hardware_available'] = 'ttyUSB' in usb_devices
        except:
            self.sensors['livox_mid360']['hardware_available'] = False

        # 检查RealSense
        try:
            lsusb_output = subprocess.check_output(['lsusb'], text=True)
            self.sensors['realsense_d455']['hardware_available'] = 'Intel' in lsusb_output and 'RealSense' in lsusb_output
        except:
            self.sensors['realsense_d455']['hardware_available'] = False

    def check_topic_activity(self):
        """检查话题活动状态"""
        current_time = time.time()

        for sensor_name, sensor_info in self.sensors.items():
            if sensor_info['last_msg_time']:
                time_since_last = current_time - sensor_info['last_msg_time']

                if time_since_last > sensor_info['timeout']:
                    if sensor_info['topic_active']:
                        sensor_info['topic_active'] = False
                        self.get_logger().warn(f'{sensor_name} 话题超时')

    def determine_slam_mode(self) -> Optional[str]:
        """根据可用传感器确定SLAM模式"""
        active_sensors = []

        for sensor_name, sensor_info in self.sensors.items():
            if sensor_info['topic_active']:
                active_sensors.append(sensor_name)

        # 优先级：融合 > 激光雷达 > RGB-D > 纯里程计
        if 'livox_mid360' in active_sensors and 'realsense_d455' in active_sensors:
            return 'fusion'
        elif 'livox_mid360' in active_sensors:
            return 'lidar_only'
        elif 'realsense_d455' in active_sensors:
            return 'rgbd_only'
        elif 'wheel_odometry' in active_sensors:
            return 'odometry_only'
        else:
            return None

    def update_slam_mode(self):
        """更新SLAM模式"""
        new_mode = self.determine_slam_mode()

        with self.mode_lock:
            if new_mode != self.current_mode:
                self.get_logger().info(f'SLAM模式变更: {self.current_mode} -> {new_mode}')
                self.current_mode = new_mode

                # 写入状态文件供launch文件读取
                self.write_sensor_status()

    def write_sensor_status(self):
        """写入传感器状态文件"""
        status = {
            'timestamp': time.time(),
            'mode': self.current_mode,
            'sensors': {},
            'config_file': None
        }

        for sensor_name, sensor_info in self.sensors.items():
            status['sensors'][sensor_name] = {
                'hardware': sensor_info['hardware_available'],
                'active': sensor_info['topic_active']
            }

        if self.current_mode and self.current_mode in self.slam_modes:
            status['config_file'] = self.slam_modes[self.current_mode]['config']

        # 写入状态文件
        status_file = '/tmp/rtabmap_sensor_status.yaml'
        with open(status_file, 'w') as f:
            yaml.dump(status, f)

    def publish_status(self):
        """发布传感器状态"""
        self.get_logger().info('========== 传感器状态 ==========')

        for sensor_name, sensor_info in self.sensors.items():
            hw_status = '✓' if sensor_info['hardware_available'] else '✗'
            topic_status = '✓' if sensor_info['topic_active'] else '✗'

            self.get_logger().info(
                f'{sensor_name:20} | 硬件: {hw_status} | 话题: {topic_status}'
            )

        self.get_logger().info(f'当前SLAM模式: {self.current_mode}')
        self.get_logger().info('================================')

def main(args=None):
    rclpy.init(args=args)

    detector = SensorDetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()