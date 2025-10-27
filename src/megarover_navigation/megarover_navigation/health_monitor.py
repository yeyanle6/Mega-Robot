#!/usr/bin/env python3
"""
系统健康监控节点
监控传感器状态并在故障时自动切换模式
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import subprocess
import psutil
import time
from typing import Dict, Optional
import yaml

class HealthMonitor(Node):
    """系统健康监控和故障恢复"""

    def __init__(self):
        super().__init__('health_monitor')

        # 参数
        self.declare_parameter('mode', 'fusion')
        self.declare_parameter('check_interval', 5.0)
        self.declare_parameter('restart_on_failure', True)
        self.declare_parameter('cpu_threshold', 80.0)
        self.declare_parameter('memory_threshold', 80.0)
        self.declare_parameter('topic_timeout', 10.0)

        self.mode = self.get_parameter('mode').value
        self.check_interval = self.get_parameter('check_interval').value
        self.restart_on_failure = self.get_parameter('restart_on_failure').value
        self.cpu_threshold = self.get_parameter('cpu_threshold').value
        self.memory_threshold = self.get_parameter('memory_threshold').value
        self.topic_timeout = self.get_parameter('topic_timeout').value

        # 系统状态
        self.system_status = {
            'cpu_usage': 0.0,
            'memory_usage': 0.0,
            'rtabmap_alive': False,
            'sensors_ok': True,
            'tf_ok': False,
            'last_error': None,
            'restart_count': 0
        }

        # 关键节点列表
        self.critical_nodes = {
            'fusion': ['/rtabmap', '/livox_ros_driver2_node', '/camera/realsense2_camera'],
            'lidar_only': ['/rtabmap', '/livox_ros_driver2_node'],
            'rgbd_only': ['/rtabmap', '/camera/realsense2_camera'],
            'odometry_only': ['/rtabmap']
        }

        # 关键话题列表
        self.critical_topics = {
            'fusion': ['/livox/lidar', '/camera/color/image_raw', '/odom', '/tf'],
            'lidar_only': ['/livox/lidar', '/odom', '/tf'],
            'rgbd_only': ['/camera/color/image_raw', '/camera/depth/image_rect_raw', '/odom', '/tf'],
            'odometry_only': ['/odom', '/tf']
        }

        # 话题最后接收时间
        self.topic_last_time = {}

        # 发布者
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/rtabmap_health_status',
            10
        )

        # 定时器
        self.health_check_timer = self.create_timer(
            self.check_interval,
            self.perform_health_check
        )

        self.diagnostic_timer = self.create_timer(
            1.0,
            self.publish_diagnostics
        )

        # 订阅关键话题监控其活动
        self.create_topic_monitors()

        self.get_logger().info(f'健康监控已启动，模式: {self.mode}')

    def create_topic_monitors(self):
        """创建话题监控器"""

        # 根据模式订阅不同话题
        if self.mode in ['fusion', 'lidar_only']:
            self.create_subscription(
                PointCloud2,
                '/livox/lidar',
                lambda msg: self.update_topic_time('/livox/lidar'),
                1
            )

        if self.mode in ['fusion', 'rgbd_only']:
            self.create_subscription(
                Image,
                '/camera/color/image_raw',
                lambda msg: self.update_topic_time('/camera/color/image_raw'),
                1
            )
            self.create_subscription(
                Image,
                '/camera/depth/image_rect_raw',
                lambda msg: self.update_topic_time('/camera/depth/image_rect_raw'),
                1
            )

        self.create_subscription(
            Odometry,
            '/odom',
            lambda msg: self.update_topic_time('/odom'),
            1
        )

        self.create_subscription(
            TFMessage,
            '/tf',
            lambda msg: self.update_topic_time('/tf'),
            1
        )

    def update_topic_time(self, topic_name: str):
        """更新话题接收时间"""
        self.topic_last_time[topic_name] = time.time()

    def check_nodes_alive(self) -> Dict[str, bool]:
        """检查节点是否存活"""
        node_status = {}

        try:
            node_list = subprocess.check_output(
                ['ros2', 'node', 'list'],
                text=True,
                timeout=2
            ).split('\n')

            for node in self.critical_nodes.get(self.mode, []):
                node_status[node] = node in node_list

        except subprocess.TimeoutExpired:
            self.get_logger().error('节点列表获取超时')
        except Exception as e:
            self.get_logger().error(f'节点检查失败: {e}')

        return node_status

    def check_topics_active(self) -> Dict[str, bool]:
        """检查话题是否活跃"""
        topic_status = {}
        current_time = time.time()

        for topic in self.critical_topics.get(self.mode, []):
            if topic in self.topic_last_time:
                time_diff = current_time - self.topic_last_time[topic]
                topic_status[topic] = time_diff < self.topic_timeout
            else:
                topic_status[topic] = False

        return topic_status

    def check_system_resources(self):
        """检查系统资源"""
        try:
            # CPU使用率
            self.system_status['cpu_usage'] = psutil.cpu_percent(interval=1)

            # 内存使用率
            memory = psutil.virtual_memory()
            self.system_status['memory_usage'] = memory.percent

        except Exception as e:
            self.get_logger().error(f'系统资源检查失败: {e}')

    def perform_health_check(self):
        """执行健康检查"""
        self.get_logger().debug('执行健康检查...')

        # 检查系统资源
        self.check_system_resources()

        # 检查节点
        node_status = self.check_nodes_alive()
        all_nodes_ok = all(node_status.values()) if node_status else False

        # 检查话题
        topic_status = self.check_topics_active()
        all_topics_ok = all(topic_status.values()) if topic_status else False

        # 更新系统状态
        self.system_status['rtabmap_alive'] = node_status.get('/rtabmap', False)
        self.system_status['sensors_ok'] = all_nodes_ok and all_topics_ok
        self.system_status['tf_ok'] = topic_status.get('/tf', False)

        # 记录问题
        issues = []

        if self.system_status['cpu_usage'] > self.cpu_threshold:
            issues.append(f'CPU使用率过高: {self.system_status["cpu_usage"]:.1f}%')

        if self.system_status['memory_usage'] > self.memory_threshold:
            issues.append(f'内存使用率过高: {self.system_status["memory_usage"]:.1f}%')

        for node, alive in node_status.items():
            if not alive:
                issues.append(f'节点离线: {node}')

        for topic, active in topic_status.items():
            if not active:
                issues.append(f'话题超时: {topic}')

        # 处理问题
        if issues:
            self.system_status['last_error'] = ', '.join(issues)
            self.get_logger().warn(f'检测到问题: {self.system_status["last_error"]}')

            # 尝试恢复
            if self.restart_on_failure and not self.system_status['rtabmap_alive']:
                self.attempt_recovery()
        else:
            self.system_status['last_error'] = None
            self.get_logger().debug('系统运行正常')

    def attempt_recovery(self):
        """尝试恢复系统"""
        self.system_status['restart_count'] += 1
        self.get_logger().info(f'尝试恢复系统 (第{self.system_status["restart_count"]}次)')

        # 检查是否需要降级模式
        if self.system_status['restart_count'] > 3:
            self.get_logger().error('多次恢复失败，建议降级到更简单的模式')
            self.suggest_mode_downgrade()
            return

        # 尝试重启RTABMAP
        try:
            # 杀死旧进程
            subprocess.run(['pkill', '-f', 'rtabmap'], timeout=5)
            time.sleep(2)

            # 重新启动（这里应该通过launch系统重启）
            self.get_logger().info('请通过launch系统重启RTABMAP节点')

        except Exception as e:
            self.get_logger().error(f'恢复失败: {e}')

    def suggest_mode_downgrade(self):
        """建议降级模式"""
        downgrade_map = {
            'fusion': 'lidar_only',
            'lidar_only': 'odometry_only',
            'rgbd_only': 'odometry_only',
            'odometry_only': None
        }

        new_mode = downgrade_map.get(self.mode)
        if new_mode:
            self.get_logger().warn(f'建议从 {self.mode} 降级到 {new_mode} 模式')

            # 写入建议文件
            suggestion = {
                'timestamp': time.time(),
                'current_mode': self.mode,
                'suggested_mode': new_mode,
                'reason': self.system_status['last_error']
            }

            with open('/tmp/rtabmap_mode_suggestion.yaml', 'w') as f:
                yaml.dump(suggestion, f)

    def publish_diagnostics(self):
        """发布诊断信息"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # RTABMAP状态
        rtabmap_status = DiagnosticStatus()
        rtabmap_status.name = 'RTABMAP SLAM'
        rtabmap_status.hardware_id = 'rtabmap_node'

        if self.system_status['rtabmap_alive']:
            rtabmap_status.level = DiagnosticStatus.OK
            rtabmap_status.message = 'RTABMAP运行正常'
        else:
            rtabmap_status.level = DiagnosticStatus.ERROR
            rtabmap_status.message = 'RTABMAP未运行'

        rtabmap_status.values = [
            KeyValue(key='mode', value=self.mode),
            KeyValue(key='restart_count', value=str(self.system_status['restart_count']))
        ]

        diag_array.status.append(rtabmap_status)

        # 系统资源状态
        resource_status = DiagnosticStatus()
        resource_status.name = '系统资源'
        resource_status.hardware_id = 'system'

        if self.system_status['cpu_usage'] > self.cpu_threshold:
            resource_status.level = DiagnosticStatus.WARN
            resource_status.message = f'CPU使用率高: {self.system_status["cpu_usage"]:.1f}%'
        elif self.system_status['memory_usage'] > self.memory_threshold:
            resource_status.level = DiagnosticStatus.WARN
            resource_status.message = f'内存使用率高: {self.system_status["memory_usage"]:.1f}%'
        else:
            resource_status.level = DiagnosticStatus.OK
            resource_status.message = '资源使用正常'

        resource_status.values = [
            KeyValue(key='cpu_usage', value=f'{self.system_status["cpu_usage"]:.1f}%'),
            KeyValue(key='memory_usage', value=f'{self.system_status["memory_usage"]:.1f}%')
        ]

        diag_array.status.append(resource_status)

        # 传感器状态
        sensor_status = DiagnosticStatus()
        sensor_status.name = '传感器系统'
        sensor_status.hardware_id = 'sensors'

        if self.system_status['sensors_ok']:
            sensor_status.level = DiagnosticStatus.OK
            sensor_status.message = '所有传感器正常'
        else:
            sensor_status.level = DiagnosticStatus.ERROR
            sensor_status.message = '传感器故障'

        diag_array.status.append(sensor_status)

        self.diagnostics_pub.publish(diag_array)

        # 发布状态摘要
        status_msg = String()
        status_msg.data = yaml.dump(self.system_status)
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    monitor = HealthMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()