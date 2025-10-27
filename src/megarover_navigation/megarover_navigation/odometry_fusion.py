#!/usr/bin/env python3
"""
简单的里程计融合节点
融合轮式里程计和IMU数据，提高定位精度
保持代码简洁易懂，不使用复杂的外部库
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class OdometryFusion(Node):
    """里程计融合节点 - 简洁版"""

    def __init__(self):
        super().__init__('odometry_fusion')

        # 参数配置
        self.declare_parameter('wheel_odom_topic', '/rover_odo')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('wheel_weight', 0.7)  # 轮式里程计权重
        self.declare_parameter('imu_weight', 0.3)    # IMU权重
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')

        # 获取参数
        wheel_topic = self.get_parameter('wheel_odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.wheel_weight = self.get_parameter('wheel_weight').value
        self.imu_weight = self.get_parameter('imu_weight').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        # 订阅器
        self.wheel_sub = self.create_subscription(
            Odometry, wheel_topic, self.wheel_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, 10)

        # 发布器
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # 数据缓存
        self.latest_wheel_odom = None
        self.latest_imu = None
        self.fused_odom = Odometry()
        self.fused_odom.header.frame_id = self.odom_frame
        self.fused_odom.child_frame_id = self.base_frame

        # 融合定时器 (50Hz)
        self.fusion_timer = self.create_timer(0.02, self.fusion_callback)

        self.get_logger().info(f'里程计融合节点已启动')
        self.get_logger().info(f'  轮式里程计: {wheel_topic} (权重: {self.wheel_weight})')
        self.get_logger().info(f'  IMU: {imu_topic} (权重: {self.imu_weight})')
        self.get_logger().info(f'  输出: {output_topic}')

    def wheel_callback(self, msg):
        """轮式里程计回调"""
        self.latest_wheel_odom = msg

    def imu_callback(self, msg):
        """IMU回调"""
        self.latest_imu = msg

    def quaternion_to_euler(self, q):
        """四元数转欧拉角 (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def fusion_callback(self):
        """融合处理 - 简单加权平均"""
        # 如果没有轮式里程计数据，跳过
        if self.latest_wheel_odom is None:
            return

        now = self.get_clock().now().to_msg()

        # 基础数据来自轮式里程计
        self.fused_odom.header.stamp = now
        self.fused_odom.pose.pose.position = self.latest_wheel_odom.pose.pose.position
        self.fused_odom.twist.twist = self.latest_wheel_odom.twist.twist

        # 如果有IMU数据，融合姿态角
        if self.latest_imu is not None:
            try:
                # 从轮式里程计获取欧拉角
                wheel_roll, wheel_pitch, wheel_yaw = self.quaternion_to_euler(
                    self.latest_wheel_odom.pose.pose.orientation
                )

                # 从IMU获取欧拉角
                imu_roll, imu_pitch, imu_yaw = self.quaternion_to_euler(
                    self.latest_imu.orientation
                )

                # 加权平均融合欧拉角
                fused_roll = wheel_roll * self.wheel_weight + imu_roll * self.imu_weight
                fused_pitch = wheel_pitch * self.wheel_weight + imu_pitch * self.imu_weight
                fused_yaw = wheel_yaw * self.wheel_weight + imu_yaw * self.imu_weight

                # 转回四元数
                self.fused_odom.pose.pose.orientation = self.euler_to_quaternion(
                    fused_roll, fused_pitch, fused_yaw
                )

            except Exception as e:
                # 如果融合失败，使用轮式里程计的姿态
                self.get_logger().warn(f'姿态融合失败: {e}，使用轮式里程计姿态')
                self.fused_odom.pose.pose.orientation = \
                    self.latest_wheel_odom.pose.pose.orientation
        else:
            # 没有IMU数据，直接使用轮式里程计姿态
            self.fused_odom.pose.pose.orientation = \
                self.latest_wheel_odom.pose.pose.orientation

        # 发布融合后的里程计
        self.odom_pub.publish(self.fused_odom)

        # 发布TF
        if self.publish_tf:
            self.publish_transform()

    def publish_transform(self):
        """发布TF变换"""
        t = TransformStamped()
        t.header.stamp = self.fused_odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # 位置
        t.transform.translation.x = self.fused_odom.pose.pose.position.x
        t.transform.translation.y = self.fused_odom.pose.pose.position.y
        t.transform.translation.z = self.fused_odom.pose.pose.position.z

        # 旋转
        t.transform.rotation = self.fused_odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryFusion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
