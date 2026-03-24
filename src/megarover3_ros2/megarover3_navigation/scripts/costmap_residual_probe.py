#!/usr/bin/env python3
"""
Measure how long local obstacle remnants persist after a transient obstacle leaves.

The probe watches both:
  - /local_costmap/voxel_grid (PointCloud2 width * height)
  - /local_costmap/costmap (occupied cell count > 0)
  - /rosout recovery-related events

Workflow:
  1. Start the probe while navigation stack is running.
  2. Wait for the probe to report "baseline locked".
  3. Introduce a transient obstacle once (e.g. a person walks across the robot front).
  4. Remove the obstacle completely.
  5. The probe detects the spike and reports how many seconds it took to decay
     back near baseline for both voxel and costmap occupancy.
"""

import argparse
import os
from collections import Counter, deque
from datetime import datetime
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class ResidualProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('costmap_residual_probe')
        self._args = args
        self._voxel_window = deque(maxlen=args.baseline_samples)
        self._costmap_window = deque(maxlen=args.baseline_samples)
        self._baseline_locked = False
        self._baseline_voxel = 0.0
        self._baseline_costmap = 0.0

        self._spike_active = False
        self._spike_started_at: Optional[float] = None
        self._voxel_spike_started_at: Optional[float] = None
        self._costmap_spike_started_at: Optional[float] = None
        self._voxel_spike_detected = False
        self._costmap_spike_detected = False
        self._voxel_recovered_at: Optional[float] = None
        self._costmap_recovered_at: Optional[float] = None
        self._stable_since_voxel: Optional[float] = None
        self._stable_since_costmap: Optional[float] = None
        self._max_voxel = 0
        self._max_costmap = 0
        self._events = Counter()

        log_dir = os.path.expanduser('~/nav_regression_logs')
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = os.path.join(log_dir, f'costmap_residual_{stamp}.log')
        self._fp = open(self._log_path, 'w', encoding='utf-8')
        self._closed = False

        qos_rosout = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=200,
        )
        self.create_subscription(PointCloud2, '/local_costmap/voxel_grid', self._voxel_cb, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, qos_rosout)
        self.create_timer(0.5, self._tick)

        self._log(
            f'probe log -> {self._log_path}\n'
            f'baseline_samples={args.baseline_samples}, spike_multiplier={args.spike_multiplier}, '
            f'recovery_ratio={args.recovery_ratio}, stable_hold={args.stable_hold}s'
        )

    def _log(self, message: str) -> None:
        if self._closed:
            return
        line = f'[{datetime.now().strftime("%H:%M:%S")}] {message}'
        self.get_logger().info(message)
        self._fp.write(line + '\n')
        self._fp.flush()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _rosout_cb(self, msg: Log) -> None:
        text = msg.msg
        patterns = {
            'failed_to_make_progress': 'Failed to make progress',
            'no_valid_trajectories': 'No valid trajectories',
            'clear_local_costmap': 'clear entirely the local_costmap',
            'controller_patience_exceeded': 'Controller patience exceeded',
        }
        for key, pattern in patterns.items():
            if pattern in text:
                self._events[key] += 1

    def _voxel_cb(self, msg: PointCloud2) -> None:
        count = int(msg.width) * int(msg.height)
        self._max_voxel = max(self._max_voxel, count)
        if not self._baseline_locked:
            self._voxel_window.append(count)
        self._evaluate(count, None)

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        occupied = sum(1 for v in msg.data if v > 0)
        self._max_costmap = max(self._max_costmap, occupied)
        if not self._baseline_locked:
            self._costmap_window.append(occupied)
        self._evaluate(None, occupied)

    def _evaluate(self, voxel_count: Optional[int], occupied_count: Optional[int]) -> None:
        if not self._baseline_locked:
            return

        now = self._now()
        if voxel_count is None:
            voxel_count = self._voxel_window[-1] if self._voxel_window else 0
        if occupied_count is None:
            occupied_count = self._costmap_window[-1] if self._costmap_window else 0

        voxel_spike = voxel_count > max(5.0, self._baseline_voxel * self._args.spike_multiplier)
        costmap_spike = occupied_count > max(5.0, self._baseline_costmap * self._args.spike_multiplier)

        if not self._spike_active and (voxel_spike or costmap_spike):
            self._spike_active = True
            self._spike_started_at = now
            self._voxel_spike_started_at = now if voxel_spike else None
            self._costmap_spike_started_at = now if costmap_spike else None
            self._voxel_spike_detected = voxel_spike
            self._costmap_spike_detected = costmap_spike
            self._voxel_recovered_at = None
            self._costmap_recovered_at = None
            self._stable_since_voxel = None
            self._stable_since_costmap = None
            self._events.clear()
            self._log(
                f'spike detected: voxel={voxel_count} baseline={self._baseline_voxel:.1f}, '
                f'costmap={occupied_count} baseline={self._baseline_costmap:.1f}'
            )
            return

        if not self._spike_active:
            return

        voxel_threshold = max(5.0, self._baseline_voxel * self._args.recovery_ratio)
        costmap_threshold = max(5.0, self._baseline_costmap * self._args.recovery_ratio)

        if voxel_spike and not self._voxel_spike_detected:
            self._voxel_spike_detected = True
            self._voxel_spike_started_at = now
            self._log(f'voxel spike armed late: voxel={voxel_count} baseline={self._baseline_voxel:.1f}')

        if costmap_spike and not self._costmap_spike_detected:
            self._costmap_spike_detected = True
            self._costmap_spike_started_at = now
            self._log(f'costmap spike armed late: costmap={occupied_count} baseline={self._baseline_costmap:.1f}')

        if self._voxel_spike_detected and voxel_count <= voxel_threshold:
            if self._stable_since_voxel is None:
                self._stable_since_voxel = now
            elif now - self._stable_since_voxel >= self._args.stable_hold and self._voxel_recovered_at is None:
                self._voxel_recovered_at = now
                voxel_elapsed = self._voxel_recovered_at - self._voxel_spike_started_at
                self._log(
                    f'voxel recovered in {voxel_elapsed:.2f}s '
                    f'(threshold={voxel_threshold:.1f}, current={voxel_count})'
                )
        else:
            self._stable_since_voxel = None

        if self._costmap_spike_detected and occupied_count <= costmap_threshold:
            if self._stable_since_costmap is None:
                self._stable_since_costmap = now
            elif now - self._stable_since_costmap >= self._args.stable_hold and self._costmap_recovered_at is None:
                self._costmap_recovered_at = now
                costmap_elapsed = self._costmap_recovered_at - self._costmap_spike_started_at
                self._log(
                    f'costmap recovered in {costmap_elapsed:.2f}s '
                    f'(threshold={costmap_threshold:.1f}, current={occupied_count})'
                )
        else:
            self._stable_since_costmap = None

        voxel_done = (not self._voxel_spike_detected) or (self._voxel_recovered_at is not None)
        costmap_done = (not self._costmap_spike_detected) or (self._costmap_recovered_at is not None)
        if voxel_done and costmap_done:
            voxel_elapsed = (
                self._voxel_recovered_at - self._voxel_spike_started_at
                if self._voxel_spike_detected and self._voxel_recovered_at is not None
                else None
            )
            costmap_elapsed = (
                self._costmap_recovered_at - self._costmap_spike_started_at
                if self._costmap_spike_detected and self._costmap_recovered_at is not None
                else None
            )
            self._log(
                f'probe complete: voxel_recovery={voxel_elapsed}, '
                f'costmap_recovery={costmap_elapsed}, '
                f'max_voxel={self._max_voxel}, max_costmap={self._max_costmap}, events={dict(self._events)}'
            )
            self._spike_active = False
            self._voxel_spike_started_at = None
            self._costmap_spike_started_at = None
            self._voxel_spike_detected = False
            self._costmap_spike_detected = False
            self._max_voxel = 0
            self._max_costmap = 0

    def _tick(self) -> None:
        if self._baseline_locked:
            return
        if len(self._voxel_window) < self._args.baseline_samples:
            return
        if len(self._costmap_window) < self._args.baseline_samples:
            return
        self._baseline_voxel = sum(self._voxel_window) / len(self._voxel_window)
        self._baseline_costmap = sum(self._costmap_window) / len(self._costmap_window)
        self._baseline_locked = True
        self._log(
            f'baseline locked: voxel={self._baseline_voxel:.1f}, '
            f'costmap={self._baseline_costmap:.1f}. introduce and remove one transient obstacle now.'
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._fp.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline-samples', type=int, default=20)
    parser.add_argument('--spike-multiplier', type=float, default=1.5)
    parser.add_argument('--recovery-ratio', type=float, default=1.1)
    parser.add_argument('--stable-hold', type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ResidualProbe(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
