#!/usr/bin/env python3
"""
IMU Relay Node - Converts Livox IMU from best_effort to reliable QoS.
Subscribes to /livox/imu with best_effort QoS and republishes with reliable QoS.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')

        # Parameters allow the relay to be retargeted without code changes
        self.input_topic = self.declare_parameter(
            'input_topic', '/livox/imu'
        ).get_parameter_value().string_value
        self.output_topic = self.declare_parameter(
            'output_topic', '/mid360/imu'
        ).get_parameter_value().string_value
        self.output_frame = self.declare_parameter(
            'output_frame_id', 'mid360_imu'
        ).get_parameter_value().string_value
        self.publish_legacy = self.declare_parameter(
            'publish_legacy_topic', True
        ).get_parameter_value().bool_value
        self.legacy_topic = self.declare_parameter(
            'legacy_topic', '/imu/data'
        ).get_parameter_value().string_value

        # QoS for Livox IMU (best_effort)
        livox_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for relayed IMU (best_effort to match robot_localization expectations)
        # robot_localization uses BEST_EFFORT by default for sensor inputs
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to Livox IMU
        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            livox_qos
        )

        # Primary output publisher (/mid360/imu)
        self.publisher = self.create_publisher(
            Imu,
            self.output_topic,
            output_qos
        )

        # Optional legacy publisher (/imu/data) for backwards compatibility
        self.legacy_publisher = None
        if self.publish_legacy:
            self.legacy_publisher = self.create_publisher(
                Imu,
                self.legacy_topic,
                output_qos
            )

        self.get_logger().info(
            f'IMU Relay started: {self.input_topic} -> {self.output_topic} (best_effort)'
        )
        if self.publish_legacy:
            self.get_logger().info(
                f'  Legacy mirror enabled: {self.legacy_topic}'
            )

    def imu_callback(self, msg):
        if self.output_frame:
            msg.header.frame_id = self.output_frame
        self.publisher.publish(msg)
        if self.legacy_publisher is not None:
            self.legacy_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
