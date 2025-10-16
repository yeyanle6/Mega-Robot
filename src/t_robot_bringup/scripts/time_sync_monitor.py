#!/usr/bin/env python3

"""
Time synchronization monitoring script for T-Robot SLAM system.
Monitors time differences between system clock and MID360 sensor timestamps.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu


class TimeSyncMonitor(Node):
    def __init__(self):
        super().__init__("time_sync_monitor")

        lidar_topic = self.declare_parameter(
            'lidar_topic', '/mid360/lidar'
        ).get_parameter_value().string_value
        imu_topic = self.declare_parameter(
            'imu_topic', '/mid360/imu'
        ).get_parameter_value().string_value

        # Create subscriptions
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            lidar_topic,
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(
            f"Time sync monitor started (LiDAR: {lidar_topic}, IMU: {imu_topic})"
        )

    def lidar_callback(self, msg):
        current_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        time_diff = (current_time - msg_time).nanoseconds / 1e9

        if abs(time_diff) > 0.1:  # Alert if more than 100ms difference
            self.get_logger().warn(
                f"LiDAR time sync issue: {time_diff:.3f}s difference"
            )

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        time_diff = (current_time - msg_time).nanoseconds / 1e9

        if abs(time_diff) > 0.1:  # Alert if more than 100ms difference
            self.get_logger().warn(f"IMU time sync issue: {time_diff:.3f}s difference")


def main(args=None):
    rclpy.init(args=args)

    monitor = TimeSyncMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
