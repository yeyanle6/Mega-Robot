#!/usr/bin/env python3

"""
Automated mapping data recorder for T-Robot SLAM system.
Records rosbag data during mapping sessions with metadata.
"""

import rclpy
from rclpy.node import Node
import subprocess
import argparse
import os
import datetime


class MappingRecorder(Node):
    def __init__(self, duration=300, output_dir="/tmp/slam_recordings"):
        super().__init__("mapping_recorder")

        self.duration = duration
        self.output_dir = output_dir

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # Generate filename with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bag_name = f"slam_mapping_{timestamp}"
        self.bag_path = os.path.join(output_dir, self.bag_name)

        self.get_logger().info(f"Mapping recorder initialized - Duration: {duration}s")
        self.get_logger().info(f"Output path: {self.bag_path}")

        # Start recording
        self.start_recording()

    def start_recording(self):
        # Topics to record
        topics = [
            "/mid360/lidar",
            "/mid360/imu",
            "/mid360/points_filtered",
            "/odometry/filtered",
            "/odom",
            "/tf",
            "/tf_static",
            "/cmd_vel",
        ]

        # Build rosbag2 command
        cmd = ["ros2", "bag", "record"] + topics + ["-o", self.bag_path]

        try:
            self.get_logger().info(f"Starting recording for {self.duration} seconds...")
            self.process = subprocess.Popen(cmd)

            # Create timer to stop recording
            self.timer = self.create_timer(self.duration, self.stop_recording)

        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")

    def stop_recording(self):
        if hasattr(self, "process"):
            self.process.terminate()
            self.get_logger().info("Recording stopped")
            self.get_logger().info(f"Bag saved to: {self.bag_path}")

        # Clean shutdown
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description="Record SLAM mapping session")
    parser.add_argument(
        "--duration",
        type=int,
        default=300,
        help="Recording duration in seconds (default: 300)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="/tmp/slam_recordings",
        help="Output directory for recordings",
    )

    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    recorder = MappingRecorder(
        duration=parsed_args.duration, output_dir=parsed_args.output_dir
    )

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass

    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

