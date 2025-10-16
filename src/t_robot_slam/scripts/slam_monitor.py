#!/usr/bin/env python3

"""
SLAM performance monitoring script for T-Robot SLAM system.
Monitors SLAM performance metrics and system health.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import psutil
import time


class SLAMMonitor(Node):
    def __init__(self):
        super().__init__("slam_monitor")

        # Performance tracking
        self.last_odom_time = None
        self.last_pointcloud_time = None
        self.odom_count = 0
        self.pointcloud_count = 0

        # Create subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/mid360/points_filtered", self.pointcloud_callback, 10
        )

        # Create timer for periodic monitoring
        self.timer = self.create_timer(5.0, self.monitor_performance)

        self.get_logger().info("SLAM monitor started")

    def odom_callback(self, msg):
        current_time = time.time()
        if self.last_odom_time is not None:
            freq = 1.0 / (current_time - self.last_odom_time)
            if freq < 20:  # Expected ~50Hz, warn if below 20Hz
                self.get_logger().warn(f"Low odometry frequency: {freq:.1f} Hz")

        self.last_odom_time = current_time
        self.odom_count += 1

    def pointcloud_callback(self, msg):
        current_time = time.time()
        if self.last_pointcloud_time is not None:
            freq = 1.0 / (current_time - self.last_pointcloud_time)
            if freq < 5:  # Expected ~10Hz, warn if below 5Hz
                self.get_logger().warn(f"Low pointcloud frequency: {freq:.1f} Hz")

        self.last_pointcloud_time = current_time
        self.pointcloud_count += 1

    def monitor_performance(self):
        # Monitor system resources
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent

        # Log performance metrics
        self.get_logger().info(
            f"Performance - CPU: {cpu_percent:.1f}%, Memory: {memory_percent:.1f}%, "
            f"Odom msgs: {self.odom_count}, PC msgs: {self.pointcloud_count}"
        )

        # Reset counters
        self.odom_count = 0
        self.pointcloud_count = 0

        # Warn on high resource usage
        if cpu_percent > 80:
            self.get_logger().warn(f"High CPU usage: {cpu_percent:.1f}%")
        if memory_percent > 80:
            self.get_logger().warn(f"High memory usage: {memory_percent:.1f}%")


def main(args=None):
    rclpy.init(args=args)

    monitor = SLAMMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

