#!/usr/bin/env python3
"""
Two-point navigation auto test for MegaRover3.

Features:
- Alternate between two goals indefinitely or for a fixed number of cycles
- Optional RViz goal capture from /goal_pose (capture two poses, then start)
- Structured trace logging under ~/nav_regression_logs
- Automatic stall judgement when commands remain active but pose stops changing

Typical usage:
  ros2 run megarover3_navigation two_point_nav_test.py

  ros2 run megarover3_navigation two_point_nav_test.py --capture-from-rviz
"""

import argparse
import json
import math
import os
from collections import Counter, deque
from dataclasses import dataclass
from datetime import datetime
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from dwb_msgs.msg import LocalPlanEvaluation
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int16MultiArray, String


DEFAULT_GOAL_A = (-0.92, -6.47, 1.307)
DEFAULT_GOAL_B = (0.35, -1.77, 1.497)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def path_length(path_msg: Path) -> float:
    total = 0.0
    poses = path_msg.poses
    for idx in range(1, len(poses)):
        dx = poses[idx].pose.position.x - poses[idx - 1].pose.position.x
        dy = poses[idx].pose.position.y - poses[idx - 1].pose.position.y
        total += math.hypot(dx, dy)
    return total


@dataclass
class GoalSpec:
    name: str
    x: float
    y: float
    yaw: float


class GoalMetrics:
    def __init__(self) -> None:
        self.dispatched_at: Optional[float] = None
        self.first_nonzero_cmd_at: Optional[float] = None
        self.cmd_samples = 0
        self.nonzero_cmd_samples = 0
        self.reverse_cmd_samples = 0
        self.reverse_rover_twist_samples = 0
        self.reverse_rover_odo_samples = 0
        self.max_cmd_linear = 0.0
        self.max_cmd_angular = 0.0
        self.min_cmd_linear = 0.0
        self.max_rover_twist_linear = 0.0
        self.min_rover_twist_linear = 0.0
        self.max_rover_odo_linear = 0.0
        self.min_rover_odo_linear = 0.0
        self.latest_global_plan_length: Optional[float] = None
        self.latest_local_plan_length: Optional[float] = None
        self.max_global_plan_length = 0.0
        self.min_global_plan_length: Optional[float] = None
        self.max_local_plan_length = 0.0
        self.pose_samples = 0

    def on_cmd(self, linear_x: float, angular_z: float, now_sec: float) -> None:
        self.cmd_samples += 1
        self.max_cmd_linear = max(self.max_cmd_linear, abs(linear_x))
        self.min_cmd_linear = min(self.min_cmd_linear, linear_x)
        self.max_cmd_angular = max(self.max_cmd_angular, abs(angular_z))
        if linear_x < -1e-3:
            self.reverse_cmd_samples += 1
        if abs(linear_x) > 1e-3 or abs(angular_z) > 1e-3:
            self.nonzero_cmd_samples += 1
            if self.first_nonzero_cmd_at is None:
                self.first_nonzero_cmd_at = now_sec

    def on_rover_twist(self, linear_x: float) -> None:
        self.max_rover_twist_linear = max(self.max_rover_twist_linear, abs(linear_x))
        self.min_rover_twist_linear = min(self.min_rover_twist_linear, linear_x)
        if linear_x < -1e-3:
            self.reverse_rover_twist_samples += 1

    def on_rover_odo(self, linear_x: float) -> None:
        self.max_rover_odo_linear = max(self.max_rover_odo_linear, abs(linear_x))
        self.min_rover_odo_linear = min(self.min_rover_odo_linear, linear_x)
        if linear_x < -1e-3:
            self.reverse_rover_odo_samples += 1

    def on_plan(self, length: float) -> None:
        self.latest_global_plan_length = length
        self.max_global_plan_length = max(self.max_global_plan_length, length)
        if self.min_global_plan_length is None:
            self.min_global_plan_length = length
        else:
            self.min_global_plan_length = min(self.min_global_plan_length, length)

    def on_local_plan(self, length: float) -> None:
        self.latest_local_plan_length = length
        self.max_local_plan_length = max(self.max_local_plan_length, length)

    def snapshot(self, end_time: float) -> Dict[str, object]:
        return {
            'cmd_samples': self.cmd_samples,
            'nonzero_cmd_samples': self.nonzero_cmd_samples,
            'reverse_cmd_samples': self.reverse_cmd_samples,
            'reverse_rover_twist_samples': self.reverse_rover_twist_samples,
            'reverse_rover_odo_samples': self.reverse_rover_odo_samples,
            'max_cmd_linear': self.max_cmd_linear,
            'min_cmd_linear': self.min_cmd_linear,
            'max_cmd_angular': self.max_cmd_angular,
            'max_rover_twist_linear': self.max_rover_twist_linear,
            'min_rover_twist_linear': self.min_rover_twist_linear,
            'max_rover_odo_linear': self.max_rover_odo_linear,
            'min_rover_odo_linear': self.min_rover_odo_linear,
            'latest_global_plan_length': self.latest_global_plan_length,
            'latest_local_plan_length': self.latest_local_plan_length,
            'max_global_plan_length': self.max_global_plan_length,
            'min_global_plan_length': self.min_global_plan_length,
            'max_local_plan_length': self.max_local_plan_length,
            'pose_samples': self.pose_samples,
            'time_to_first_nonzero_cmd_s': (
                round(self.first_nonzero_cmd_at - self.dispatched_at, 2)
                if self.first_nonzero_cmd_at is not None and self.dispatched_at is not None
                else None
            ),
            'end_time_s': round(end_time, 2),
        }


class TwoPointNavTest(Node):
    BUMPER_BLOCKING_STATES = {
        'TRIGGERED',
        'RETREATING',
        'WAITING_CLEAR',
        'COOLDOWN',
    }

    def __init__(self, args: argparse.Namespace):
        super().__init__('two_point_nav_test')
        self._args = args
        self._goal_queue: List[GoalSpec] = []
        self._capture_done = not args.capture_from_rviz
        self._nav_status: Optional[str] = None
        self._nav_status_first_seen_at: Optional[float] = None
        self._startup_started_at = self._now_sec()
        self._last_wait_log_at = 0.0
        if self._capture_done:
            self._goal_queue = [
                GoalSpec('goal_a', args.goal_a[0], args.goal_a[1], args.goal_a[2]),
                GoalSpec('goal_b', args.goal_b[0], args.goal_b[1], args.goal_b[2]),
            ]

        self._goal_index = 0
        self._run_index = 0
        self._run_active = False
        self._run_done = False
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._current_goal_name = ''
        self._current_goal: Optional[GoalSpec] = None
        self._current_metrics: Optional[GoalMetrics] = None
        self._current_events = Counter()
        self._current_goal_start = None
        self._next_dispatch_at = 0.0
        self._latest_cmd = {'linear_x': 0.0, 'angular_z': 0.0}
        self._latest_rover_twist = {'linear_x': 0.0}
        self._latest_rover_odo = {'linear_x': 0.0}
        self._latest_pose = {'x': None, 'y': None, 'yaw': None}
        self._latest_rover_sensor_raw = 0
        self._latest_bumper_status = 'UNKNOWN'
        self._latest_bumper_detail = ''
        self._last_bumper_block_log_at = 0.0
        self._pose_window: Deque[Tuple[float, float, float, float]] = deque()
        self._last_trace_at = 0.0
        self._trace_interval_sec = 0.2
        self._goal_stats: List[Dict[str, object]] = []
        self._goal_retry_counts: Dict[str, int] = {}
        self._recent_rosout: List[str] = []
        self._log_patterns = {
            'failed_to_make_progress': 'Failed to make progress',
            'no_valid_trajectories': 'No valid trajectories',
            'trajectory_hits_obstacle': 'Trajectory Hits Obstacle',
            'clear_local_costmap': 'clear entirely the local_costmap',
            'goal_succeeded_log': 'Goal succeeded',
        }

        log_dir = os.path.expanduser('~/nav_regression_logs')
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        label = 'rviz_capture' if args.capture_from_rviz else 'default_p1_p2'
        base_name = f'two_point_nav_{stamp}_{label}'
        self._log_path = os.path.join(log_dir, base_name + '.log')
        self._trace_path = os.path.join(log_dir, base_name + '.jsonl')
        self._log_fp = open(self._log_path, 'w', encoding='utf-8')
        self._trace_fp = open(self._trace_path, 'w', encoding='utf-8')
        self._closed = False

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        qos_rosout = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=200,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=50,
        )

        self.create_subscription(Log, '/rosout', self._rosout_cb, qos_rosout)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self._local_plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Twist, '/rover_twist', self._rover_twist_cb, qos_sensor)
        self.create_subscription(Twist, '/rover_odo', self._rover_odo_cb, qos_sensor)
        self.create_subscription(Odometry, '/lio_odom', self._lio_odom_cb, qos_sensor)
        self.create_subscription(Int16MultiArray, '/rover_sensor', self._rover_sensor_cb, qos_sensor)
        self.create_subscription(String, '/bumper_safety/status', self._bumper_status_cb, 10)
        self.create_subscription(String, '/bumper_safety/detail', self._bumper_detail_cb, 10)
        self.create_subscription(LocalPlanEvaluation, '/evaluation', self._evaluation_cb, qos_sensor)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_cb, qos_sensor)
        self.create_subscription(String, '/nav_initializer/status', self._nav_status_cb, 10)

        self._timer = self.create_timer(0.2, self._tick)
        self._write_trace({
            'type': 'meta',
            'log_file': self._log_path,
            'trace_file': self._trace_path,
            'capture_from_rviz': args.capture_from_rviz,
            'cycles': args.cycles,
            'timeout_s': args.timeout,
            'settle_s': args.settle,
            'require_localized': args.require_localized,
            'startup_wait_s': args.startup_wait,
            'stall_window_s': args.stall_window,
            'stall_min_cmd_linear': args.stall_min_cmd_linear,
            'stall_min_pose_delta': args.stall_min_pose_delta,
            'bumper_resume_delay_s': args.bumper_resume_delay,
            'max_bumper_retries': args.max_bumper_retries,
        })

        if self._capture_done:
            self._write(
                'two-point auto test ready with default goals: '
                f'A=({self._goal_queue[0].x:.3f}, {self._goal_queue[0].y:.3f}, yaw={self._goal_queue[0].yaw:.3f}), '
                f'B=({self._goal_queue[1].x:.3f}, {self._goal_queue[1].y:.3f}, yaw={self._goal_queue[1].yaw:.3f})'
            )
        else:
            self._write('waiting for 2 RViz goals on /goal_pose to start alternating test')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _write(self, message: str) -> None:
        if self._closed:
            return
        line = f'[{datetime.now().strftime("%H:%M:%S")}] {message}'
        self.get_logger().info(message)
        self._log_fp.write(line + '\n')
        self._log_fp.flush()

    def _write_trace(self, payload: Dict[str, object]) -> None:
        if self._closed:
            return
        payload = dict(payload)
        payload.setdefault('wall_time', datetime.now().isoformat(timespec='milliseconds'))
        payload.setdefault('ros_time_s', round(self._now_sec(), 3))
        self._trace_fp.write(json.dumps(payload, ensure_ascii=True) + '\n')
        self._trace_fp.flush()

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        if self._capture_done:
            return
        yaw = quat_to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)
        goal = GoalSpec(
            name=f'goal_{len(self._goal_queue) + 1}',
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=yaw,
        )
        self._goal_queue.append(goal)
        self._write(
            f'captured RViz goal {len(self._goal_queue)}/2 -> '
            f'({goal.x:.3f}, {goal.y:.3f}, yaw={goal.yaw:.3f})'
        )
        self._write_trace({
            'type': 'captured_goal',
            'goal': {'name': goal.name, 'x': goal.x, 'y': goal.y, 'yaw': goal.yaw},
        })
        if len(self._goal_queue) >= 2:
            self._capture_done = True
            self._goal_queue[0].name = 'goal_a'
            self._goal_queue[1].name = 'goal_b'
            self._write('captured 2 RViz goals, auto test will start')

    def _nav_status_cb(self, msg: String) -> None:
        self._nav_status = msg.data.strip().upper()
        if self._nav_status_first_seen_at is None:
            self._nav_status_first_seen_at = self._now_sec()
        self._write_trace({
            'type': 'nav_initializer_status',
            'status': self._nav_status,
        })
        if self._nav_status in ('FAILED', 'LOCALIZED'):
            self._write(f'nav_initializer/status={self._nav_status}')

    def _rosout_cb(self, msg: Log) -> None:
        text = msg.msg
        self._recent_rosout.append(f'{msg.name}: {text}')
        if len(self._recent_rosout) > 200:
            self._recent_rosout.pop(0)

        if not self._run_active:
            return

        for key, pattern in self._log_patterns.items():
            if pattern in text:
                self._current_events[key] += 1
                self._write_trace({
                    'type': 'event',
                    'run': self._current_goal_name,
                    'event': key,
                    'node': msg.name,
                    'message': text,
                })

    def _tick(self) -> None:
        if self._run_done or not self._capture_done:
            return

        startup_ready, startup_reason = self._check_startup_readiness()
        if startup_reason is not None:
            self._finish_run(startup_reason)
            return
        if not startup_ready:
            return

        if not self._client.server_is_ready():
            if not self._client.wait_for_server(timeout_sec=0.1):
                self._write('waiting for navigate_to_pose action server...')
                return

        now_sec = self._now_sec()
        if not self._run_active:
            if now_sec < self._next_dispatch_at:
                return
            if self._latest_bumper_status in self.BUMPER_BLOCKING_STATES:
                if now_sec - self._last_bumper_block_log_at >= 5.0:
                    self._write(
                        f'waiting for bumper safety to clear before dispatch: '
                        f'status={self._latest_bumper_status} detail={self._latest_bumper_detail or "n/a"}'
                    )
                    self._last_bumper_block_log_at = now_sec
                return
            if self._args.cycles > 0 and self._run_index >= self._args.cycles:
                self._finish_run()
                return
            self._dispatch_next_goal()
            return

        elapsed = (self.get_clock().now() - self._current_goal_start).nanoseconds / 1e9
        if elapsed > self._args.timeout:
            self._write(f'{self._current_goal_name} timeout after {elapsed:.1f}s, canceling goal')
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._finalize_goal('timeout')
            return

        stall_reason = self._detect_stall(now_sec)
        if stall_reason is not None:
            self._write(f'{self._current_goal_name} detected {stall_reason}, canceling goal')
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._finalize_goal(stall_reason)

    def _check_startup_readiness(self) -> Tuple[bool, Optional[str]]:
        if not self._args.require_localized:
            return True, None
        elapsed = self._now_sec() - self._startup_started_at
        if self._nav_status == 'LOCALIZED':
            return True, None
        if self._nav_status == 'FAILED':
            return False, 'startup_localization_failed'
        if elapsed < self._args.startup_wait:
            if elapsed - self._last_wait_log_at >= 5.0:
                status = self._nav_status if self._nav_status is not None else 'WAITING_STATUS'
                self._write(
                    f'waiting for nav initializer readiness: status={status}, '
                    f'elapsed={elapsed:.1f}s/{self._args.startup_wait:.1f}s'
                )
                self._last_wait_log_at = elapsed
            return False, None
        if self._nav_status is None:
            return False, 'startup_nav_status_missing'
        return False, 'startup_not_localized'

    def _dispatch_next_goal(self) -> None:
        goal = self._goal_queue[self._goal_index]
        self._goal_index = (self._goal_index + 1) % 2
        self._run_index += 1
        self._current_goal = goal
        self._current_goal_name = f'run_{self._run_index:02d}_{goal.name}'
        self._current_goal_start = self.get_clock().now()
        self._current_metrics = GoalMetrics()
        self._current_metrics.dispatched_at = self._current_goal_start.nanoseconds / 1e9
        self._current_events = Counter()
        self._run_active = True
        self._pose_window.clear()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal.x
        goal_msg.pose.pose.position.y = goal.y
        qx, qy, qz, qw = yaw_to_quat(goal.yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._write(
            f'{self._current_goal_name} dispatch -> '
            f'({goal.x:.3f}, {goal.y:.3f}, yaw={goal.yaw:.3f})'
        )
        self._write_trace({
            'type': 'goal_dispatch',
            'run': self._current_goal_name,
            'goal': {'name': goal.name, 'x': goal.x, 'y': goal.y, 'yaw': goal.yaw},
        })
        self._goal_future = self._client.send_goal_async(goal_msg)
        self._goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self._write(f'{self._current_goal_name} goal request failed: {exc}')
            self._finalize_goal('request_failed')
            return

        if not self._goal_handle.accepted:
            self._write(f'{self._current_goal_name} goal rejected')
            self._finalize_goal('rejected')
            return

        self._write(f'{self._current_goal_name} accepted')
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        if not self._run_active:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._write(f'{self._current_goal_name} result error: {exc}')
            self._finalize_goal('result_error')
            return

        self._finalize_goal(self._status_name(result.status))

    def _plan_cb(self, msg: Path) -> None:
        if not self._run_active or self._current_metrics is None or len(msg.poses) < 2:
            return
        self._current_metrics.on_plan(path_length(msg))

    def _local_plan_cb(self, msg: Path) -> None:
        if not self._run_active or self._current_metrics is None or len(msg.poses) < 2:
            return
        self._current_metrics.on_local_plan(path_length(msg))

    def _cmd_cb(self, msg: Twist) -> None:
        if not self._run_active or self._current_metrics is None:
            return
        now_sec = self._now_sec()
        self._latest_cmd = {'linear_x': msg.linear.x, 'angular_z': msg.angular.z}
        self._current_metrics.on_cmd(msg.linear.x, msg.angular.z, now_sec)

    def _rover_twist_cb(self, msg: Twist) -> None:
        if not self._run_active or self._current_metrics is None:
            return
        self._latest_rover_twist = {'linear_x': msg.linear.x}
        self._current_metrics.on_rover_twist(msg.linear.x)

    def _rover_odo_cb(self, msg: Twist) -> None:
        if not self._run_active or self._current_metrics is None:
            return
        self._latest_rover_odo = {'linear_x': msg.linear.x}
        self._current_metrics.on_rover_odo(msg.linear.x)

    def _lio_odom_cb(self, msg: Odometry) -> None:
        if not self._run_active or self._current_metrics is None:
            return
        pose = msg.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z),
        )
        now_sec = self._now_sec()
        self._latest_pose = {'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw}
        self._current_metrics.pose_samples += 1
        self._pose_window.append((now_sec, pose.position.x, pose.position.y, yaw))
        self._trim_pose_window(now_sec)

        if now_sec - self._last_trace_at < self._trace_interval_sec:
            return
        self._last_trace_at = now_sec
        self._write_trace({
            'type': 'sample',
            'run': self._current_goal_name,
            'goal': (
                {
                    'name': self._current_goal.name,
                    'x': self._current_goal.x,
                    'y': self._current_goal.y,
                    'yaw': self._current_goal.yaw,
                } if self._current_goal is not None else None
            ),
            'pose': self._latest_pose,
            'cmd': self._latest_cmd,
            'rover_twist': self._latest_rover_twist,
            'rover_odo': self._latest_rover_odo,
            'rover_sensor_raw': self._latest_rover_sensor_raw,
            'bumper_status': self._latest_bumper_status,
            'global_plan_length': self._current_metrics.latest_global_plan_length,
            'local_plan_length': self._current_metrics.latest_local_plan_length,
        })

    def _rover_sensor_cb(self, msg: Int16MultiArray) -> None:
        if len(msg.data) < 1:
            return
        raw = int(msg.data[0])
        self._latest_rover_sensor_raw = raw
        if not self._run_active:
            return
        if raw != 0:
            self._current_events['bumper_trigger'] += 1
            self._write_trace({
                'type': 'event',
                'run': self._current_goal_name,
                'event': 'bumper_trigger',
                'raw': raw,
            })

    def _bumper_status_cb(self, msg: String) -> None:
        status = msg.data.strip().upper()
        self._latest_bumper_status = status
        if not self._run_active:
            return
        if status and status != 'NORMAL':
            self._current_events['bumper_safety_active'] += 1
            self._write_trace({
                'type': 'event',
                'run': self._current_goal_name,
                'event': 'bumper_safety_active',
                'status': status,
            })

    def _bumper_detail_cb(self, msg: String) -> None:
        self._latest_bumper_detail = msg.data.strip()

    def _evaluation_cb(self, msg: LocalPlanEvaluation) -> None:
        if not self._run_active:
            return
        self._current_events['evaluation_samples'] += 1
        if len(msg.twists) == 0:
            self._current_events['zero_trajectory_eval'] += 1
            return
        finite_scores = 0
        forward_candidates = 0
        for twist in msg.twists:
            if math.isfinite(float(twist.total)):
                finite_scores += 1
            if twist.traj.velocity.x > 0.01:
                forward_candidates += 1
        if finite_scores == 0:
            self._current_events['no_finite_trajectories'] += 1
        if forward_candidates == 0:
            self._current_events['no_forward_candidates'] += 1

    def _trim_pose_window(self, now_sec: float) -> None:
        min_time = now_sec - self._args.stall_window
        while self._pose_window and self._pose_window[0][0] < min_time:
            self._pose_window.popleft()

    def _detect_stall(self, now_sec: float) -> Optional[str]:
        if len(self._pose_window) < 2:
            return None
        self._trim_pose_window(now_sec)
        oldest = self._pose_window[0]
        newest = self._pose_window[-1]
        observed_window = newest[0] - oldest[0]
        if observed_window < self._args.stall_window:
            return None
        pose_delta = math.hypot(newest[1] - oldest[1], newest[2] - oldest[2])
        cmd_linear = abs(self._latest_cmd['linear_x'])
        rover_twist_linear = abs(self._latest_rover_twist['linear_x'])
        rover_odo_linear = abs(self._latest_rover_odo['linear_x'])
        if cmd_linear < self._args.stall_min_cmd_linear:
            return None
        if pose_delta >= self._args.stall_min_pose_delta:
            return None

        if rover_twist_linear < self._args.stall_min_feedback_linear:
            return 'stall_cmd_not_reaching_base'
        if rover_odo_linear < self._args.stall_min_feedback_linear:
            return 'stall_base_not_moving'
        return 'stall_lio_pose_frozen'

    def _build_diagnosis(self) -> Dict[str, object]:
        outcome_counts = Counter(item['outcome'] for item in self._goal_stats)
        event_totals = Counter()
        bridge_ok_runs = 0
        bridge_gap_runs = 0
        base_feedback_gap_runs = 0
        rotation_only_runs = 0
        no_local_plan_runs = 0
        moving_cmd_runs = 0
        near_goal_timeout_runs = 0
        for item in self._goal_stats:
            event_totals.update(item['events'])
            metrics = item.get('metrics', {})
            max_cmd_linear = float(metrics.get('max_cmd_linear') or 0.0)
            max_cmd_angular = float(metrics.get('max_cmd_angular') or 0.0)
            max_rover_twist_linear = float(metrics.get('max_rover_twist_linear') or 0.0)
            max_rover_odo_linear = float(metrics.get('max_rover_odo_linear') or 0.0)
            max_local_plan = float(metrics.get('max_local_plan_length') or 0.0)
            min_global_plan = metrics.get('min_global_plan_length')
            min_global_plan = float(min_global_plan) if min_global_plan is not None else None
            if max_cmd_linear >= self._args.stall_min_cmd_linear:
                moving_cmd_runs += 1
                if max_rover_twist_linear >= 0.8 * max_cmd_linear:
                    bridge_ok_runs += 1
                elif max_rover_twist_linear < self._args.stall_min_feedback_linear:
                    bridge_gap_runs += 1
                if max_rover_twist_linear >= self._args.stall_min_feedback_linear and \
                        max_rover_odo_linear < self._args.stall_min_feedback_linear:
                    base_feedback_gap_runs += 1
            if max_cmd_linear <= 0.02 and max_cmd_angular >= 0.3:
                rotation_only_runs += 1
            if max_local_plan <= 0.05:
                no_local_plan_runs += 1
            if item.get('outcome') == 'timeout' and min_global_plan is not None and \
                    min_global_plan <= 0.35 and max_rover_odo_linear >= 0.10:
                near_goal_timeout_runs += 1

        diagnosis = {
            'primary_issue': 'insufficient_data',
            'confidence': 'low',
            'evidence': {
                'outcomes': dict(outcome_counts),
                'events': dict(event_totals),
                'bridge_ok_runs': bridge_ok_runs,
                'bridge_gap_runs': bridge_gap_runs,
                'base_feedback_gap_runs': base_feedback_gap_runs,
                'moving_cmd_runs': moving_cmd_runs,
                'rotation_only_runs': rotation_only_runs,
                'no_local_plan_runs': no_local_plan_runs,
                'near_goal_timeout_runs': near_goal_timeout_runs,
            },
            'recommendation': 'collect more runs',
        }

        total_runs = len(self._goal_stats)
        if total_runs == 0:
            return diagnosis

        if event_totals.get('bumper_trigger', 0) > 0 or event_totals.get('bumper_safety_active', 0) > 0:
            diagnosis.update({
                'primary_issue': 'bumper_safety_intervention',
                'confidence': 'high',
                'recommendation': 'inspect physical bumper contacts and /bumper_safety state before changing Nav2 parameters',
            })
            return diagnosis

        if moving_cmd_runs > 0 and bridge_gap_runs >= max(1, math.ceil(moving_cmd_runs / 2)):
            diagnosis.update({
                'primary_issue': 'cmd_vel_not_reaching_base',
                'confidence': 'high',
                'recommendation': 'inspect /cmd_vel -> /rover_twist bridge, safety interlocks, and velocity_smoother path',
            })
            return diagnosis

        if moving_cmd_runs > 0 and base_feedback_gap_runs >= max(1, math.ceil(moving_cmd_runs / 2)):
            diagnosis.update({
                'primary_issue': 'base_not_executing_motion',
                'confidence': 'high',
                'recommendation': 'inspect base controller, micro-ROS link, motor enable, and /rover_odo feedback',
            })
            return diagnosis

        if outcome_counts.get('stall_cmd_not_reaching_base', 0) >= max(1, math.ceil(total_runs / 2)):
            diagnosis.update({
                'primary_issue': 'cmd_vel_not_reaching_base',
                'confidence': 'high',
                'recommendation': 'inspect /cmd_vel -> /rover_twist bridge and velocity_smoother path',
            })
            return diagnosis

        if outcome_counts.get('stall_base_not_moving', 0) >= max(1, math.ceil(total_runs / 2)):
            diagnosis.update({
                'primary_issue': 'base_not_executing_motion',
                'confidence': 'high',
                'recommendation': 'inspect base controller, micro-ROS link, motor enable, and /rover_odo feedback',
            })
            return diagnosis

        if outcome_counts.get('stall_lio_pose_frozen', 0) >= max(1, math.ceil(total_runs / 2)):
            diagnosis.update({
                'primary_issue': 'lio_pose_freeze',
                'confidence': 'high',
                'recommendation': 'inspect FAST-LIO2/localizer state and compare /rover_odo against /lio_odom',
            })
            return diagnosis

        if near_goal_timeout_runs >= max(1, math.ceil(total_runs / 2)):
            diagnosis.update({
                'primary_issue': 'near_goal_not_converging',
                'confidence': 'high',
                'recommendation': 'relax goal tolerances and reduce RotateToGoal aggressiveness near the target',
            })
            return diagnosis

        controller_failure_runs = outcome_counts.get('aborted', 0) + outcome_counts.get('timeout', 0)
        if controller_failure_runs >= max(1, math.ceil(total_runs / 2)):
            if rotation_only_runs >= max(1, math.ceil(total_runs / 3)):
                diagnosis.update({
                    'primary_issue': 'near_goal_rotation_or_controller_abort',
                    'confidence': 'medium',
                    'recommendation': 'inspect RotateToGoal, goal tolerances, and controller/planner abort causes',
                })
                return diagnosis

            if no_local_plan_runs >= max(1, math.ceil(total_runs / 3)) or \
                    event_totals.get('zero_trajectory_eval', 0) > 0 or \
                    event_totals.get('no_finite_trajectories', 0) > 0:
                diagnosis.update({
                    'primary_issue': 'local_planner_not_generating_paths',
                    'confidence': 'medium',
                    'recommendation': 'inspect local costmap, obstacle layers, and controller critic configuration',
                })
                return diagnosis

        return diagnosis

    def _finalize_goal(self, outcome: str) -> None:
        if not self._run_active:
            return
        elapsed = (self.get_clock().now() - self._current_goal_start).nanoseconds / 1e9
        metrics = self._current_metrics.snapshot(self._now_sec()) if self._current_metrics else {}
        row = {
            'name': self._current_goal_name,
            'goal': (
                {
                    'name': self._current_goal.name,
                    'x': self._current_goal.x,
                    'y': self._current_goal.y,
                    'yaw': self._current_goal.yaw,
                } if self._current_goal is not None else None
            ),
            'outcome': outcome,
            'elapsed_s': round(elapsed, 2),
            'events': dict(self._current_events),
            'metrics': metrics,
        }
        self._goal_stats.append(row)
        self._write(
            f'{self._current_goal_name} summary -> outcome={outcome}, '
            f'elapsed={elapsed:.1f}s, events={dict(self._current_events)}, metrics={metrics}'
        )
        self._write_trace({
            'type': 'goal_summary',
            'run': self._current_goal_name,
            'outcome': outcome,
            'elapsed_s': round(elapsed, 2),
            'events': dict(self._current_events),
            'metrics': metrics,
        })

        retry_goal = None
        if self._current_goal is not None and outcome == 'canceled':
            if self._current_events.get('bumper_trigger', 0) > 0 or \
                    self._current_events.get('bumper_safety_active', 0) > 0:
                goal_name = self._current_goal.name
                retry_count = self._goal_retry_counts.get(goal_name, 0)
                if retry_count < self._args.max_bumper_retries:
                    self._goal_retry_counts[goal_name] = retry_count + 1
                    retry_goal = self._current_goal
                    self._goal_index = (self._goal_index - 1) % len(self._goal_queue)
                    self._write(
                        f'bumper cancellation detected, will retry {goal_name} after '
                        f'{self._args.bumper_resume_delay:.1f}s '
                        f'(retry {retry_count + 1}/{self._args.max_bumper_retries})'
                    )
                else:
                    self._write(
                        f'bumper cancellation detected for {goal_name}, retry budget exhausted '
                        f'({self._args.max_bumper_retries})'
                    )
        if retry_goal is None and self._current_goal is not None:
            self._goal_retry_counts[self._current_goal.name] = 0

        self._run_active = False
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._current_goal_name = ''
        self._current_goal = None
        self._current_metrics = None
        self._current_events = Counter()
        self._pose_window.clear()

        delay = self._args.settle
        if retry_goal is not None:
            delay = self._args.bumper_resume_delay
        if delay > 0.0:
            self._next_dispatch_at = self._now_sec() + delay

    def _finish_run(self, startup_failure: Optional[str] = None) -> None:
        if self._run_done:
            return
        self._run_done = True
        outcome_counts = Counter(item['outcome'] for item in self._goal_stats)
        event_totals = Counter()
        for item in self._goal_stats:
            event_totals.update(item['events'])

        self._write('=== two-point auto test summary ===')
        if startup_failure is not None:
            self._write(f'startup_failure={startup_failure}')
        self._write(f'total_runs={len(self._goal_stats)} outcomes={dict(outcome_counts)}')
        self._write(f'event_totals={dict(event_totals)}')
        diagnosis = self._build_diagnosis()
        self._write(
            'diagnosis='
            f"{diagnosis['primary_issue']} confidence={diagnosis['confidence']} "
            f"recommendation={diagnosis['recommendation']}"
        )
        self._write(f'log_file={self._log_path}')
        self._write(f'trace_file={self._trace_path}')
        self._write_trace({
            'type': 'run_summary',
            'startup_failure': startup_failure,
            'outcomes': dict(outcome_counts),
            'event_totals': dict(event_totals),
            'diagnosis': diagnosis,
            'goal_stats': self._goal_stats,
            'log_file': self._log_path,
            'trace_file': self._trace_path,
        })
        self.destroy_timer(self._timer)

    @staticmethod
    def _status_name(status: int) -> str:
        mapping = {
            GoalStatus.STATUS_UNKNOWN: 'unknown',
            GoalStatus.STATUS_ACCEPTED: 'accepted',
            GoalStatus.STATUS_EXECUTING: 'executing',
            GoalStatus.STATUS_CANCELING: 'canceling',
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }
        return mapping.get(status, f'status_{status}')

    @property
    def run_done(self) -> bool:
        return self._run_done

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._log_fp.close()
        self._trace_fp.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture-from-rviz', action='store_true',
                        help='capture goal A and goal B from two RViz /goal_pose clicks')
    parser.add_argument('--goal-a', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        default=DEFAULT_GOAL_A,
                        help='goal A in map frame')
    parser.add_argument('--goal-b', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        default=DEFAULT_GOAL_B,
                        help='goal B in map frame')
    parser.add_argument('--cycles', type=int, default=0,
                        help='total goal dispatch count; 0 means run until Ctrl-C')
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='per-goal timeout seconds')
    parser.add_argument('--settle', type=float, default=3.0,
                        help='wait after each goal before sending the next one')
    parser.add_argument('--require-localized', dest='require_localized', action='store_true',
                        default=True,
                        help='wait for /nav_initializer/status=LOCALIZED before dispatching goals')
    parser.add_argument('--no-require-localized', dest='require_localized', action='store_false',
                        help='skip startup localization gating when pose has already been set manually')
    parser.add_argument('--startup-wait', type=float, default=20.0,
                        help='seconds to wait for nav initializer readiness before failing early')
    parser.add_argument('--stall-window', type=float, default=6.0,
                        help='window used to detect pose freeze under active commands')
    parser.add_argument('--stall-min-cmd-linear', type=float, default=0.20,
                        help='minimum |cmd_vel.linear.x| considered active forward command')
    parser.add_argument('--stall-min-pose-delta', type=float, default=0.05,
                        help='minimum pose displacement required over the stall window')
    parser.add_argument('--stall-min-feedback-linear', type=float, default=0.02,
                        help='minimum |rover_twist| or |rover_odo| considered actual movement')
    parser.add_argument('--bumper-resume-delay', type=float, default=5.0,
                        help='delay before retrying the same goal after bumper-triggered cancellation')
    parser.add_argument('--max-bumper-retries', type=int, default=2,
                        help='maximum retries for the same goal after bumper-triggered cancellation')
    args = parser.parse_args()
    if args.cycles < 0:
        parser.error('--cycles must be >= 0')
    if args.timeout <= 0.0:
        parser.error('--timeout must be > 0')
    if args.stall_window <= 0.0:
        parser.error('--stall-window must be > 0')
    if args.max_bumper_retries < 0:
        parser.error('--max-bumper-retries must be >= 0')
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TwoPointNavTest(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.run_done:
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info('interrupted by user')
    finally:
        executor.shutdown()
        node.destroy_node()
        node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
