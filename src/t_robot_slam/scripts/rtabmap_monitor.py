#!/usr/bin/env python3

"""
RTAB-Map performance monitor for loop closure and odometry drift.
Monitors mapping quality, loop closures, and database statistics.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info
import time
from collections import deque


class RTABMapMonitor(Node):
    def __init__(self):
        super().__init__("rtabmap_monitor")

        # Statistics tracking
        self.loop_closures = 0
        self.last_loop_closure_time = None
        self.total_nodes = 0
        self.position_history = deque(maxlen=100)  # Track last 100 positions
        self.start_time = time.time()

        # Create subscriptions
        self.info_sub = self.create_subscription(
            Info, "/rtabmap/info", self.info_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )

        # Create timer for periodic reporting
        self.timer = self.create_timer(10.0, self.report_status)

        self.get_logger().info("RTAB-Map monitor started")
        self.get_logger().info("Monitoring loop closures and mapping performance...")

    def info_callback(self, msg):
        """Process RTAB-Map info messages"""
        # Update total nodes
        self.total_nodes = msg.loop_closure_id

        # Check for loop closure
        if msg.loop_closure_id > 0 and msg.loop_closure_transform:
            self.loop_closures += 1
            self.last_loop_closure_time = time.time()

            self.get_logger().info(
                f"🔄 LOOP CLOSURE DETECTED! "
                f"ID: {msg.loop_closure_id}, "
                f"Total loops: {self.loop_closures}"
            )

        # Log mapping statistics
        if hasattr(msg, 'stats_map_ids'):
            map_size = len(msg.stats_map_ids) if msg.stats_map_ids else 0
            self.get_logger().debug(
                f"Map nodes: {map_size}, "
                f"Working memory: {msg.wmState if hasattr(msg, 'wmState') else 'N/A'}"
            )

    def odom_callback(self, msg):
        """Track odometry for drift analysis"""
        pos = msg.pose.pose.position
        self.position_history.append((pos.x, pos.y, pos.z))

    def calculate_drift(self):
        """Calculate odometry drift based on position history"""
        if len(self.position_history) < 2:
            return 0.0

        positions = list(self.position_history)
        total_distance = 0.0

        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            dz = positions[i][2] - positions[i-1][2]
            total_distance += (dx**2 + dy**2 + dz**2) ** 0.5

        # Estimate drift as distance from start to current
        if len(positions) > 0:
            start = positions[0]
            end = positions[-1]
            direct_distance = (
                (end[0] - start[0])**2 +
                (end[1] - start[1])**2 +
                (end[2] - start[2])**2
            ) ** 0.5

            # Drift percentage
            if total_distance > 0:
                drift = (total_distance - direct_distance) / total_distance * 100
                return drift

        return 0.0

    def report_status(self):
        """Periodic status report"""
        runtime = time.time() - self.start_time
        drift = self.calculate_drift()

        status_msg = (
            f"\n{'='*50}\n"
            f"RTAB-Map Status (Runtime: {runtime:.1f}s)\n"
            f"{'='*50}\n"
            f"  Map Nodes: {self.total_nodes}\n"
            f"  Loop Closures: {self.loop_closures}\n"
            f"  Estimated Drift: {drift:.2f}%\n"
        )

        if self.last_loop_closure_time:
            time_since_loop = time.time() - self.last_loop_closure_time
            status_msg += f"  Last Loop Closure: {time_since_loop:.1f}s ago\n"
        else:
            status_msg += f"  Last Loop Closure: Never\n"

        # Position info
        if len(self.position_history) > 0:
            pos = self.position_history[-1]
            status_msg += f"  Current Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})\n"

        status_msg += f"{'='*50}"

        self.get_logger().info(status_msg)

        # Warnings
        if self.loop_closures == 0 and runtime > 60:
            self.get_logger().warn(
                "⚠️  No loop closures detected after 60s. "
                "Ensure you revisit previously mapped areas."
            )

        if drift > 10.0:
            self.get_logger().warn(
                f"⚠️  High odometry drift detected: {drift:.2f}%. "
                "Loop closures recommended."
            )


def main(args=None):
    rclpy.init(args=args)

    monitor = RTABMapMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
