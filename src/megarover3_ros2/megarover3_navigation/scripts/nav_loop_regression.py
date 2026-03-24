#!/usr/bin/env python3
"""
按固定目标点序列循环运行，并统计每轮导航结果。

用途:
  - 回归验证 Nav2 在重复目标切换下是否稳定
  - 统计每轮耗时、成功率、恢复类日志次数
  - 识别 "No valid trajectories" / "Failed to make progress" 等问题

示例:
  ros2 run megarover3_navigation nav_loop_regression.py \
    --goal-a 2.82 -0.14 0.0 \
    --goal-b -2.41 0.79 3.14 \
    --cycles 6 \
    --timeout 180

  ros2 run megarover3_navigation nav_loop_regression.py \
    --goal p1 1.0 0.0 0.0 \
    --goal p2 2.0 0.0 1.57 \
    --goal p3 2.0 1.0 3.14 \
    --goal p4 1.0 1.0 -1.57 \
    --loops 3 \
    --timeout 180
"""

import argparse
import json
import math
import os
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def sanitize_name(value: str) -> str:
    return ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in value)


@dataclass
class GoalSpec:
    name: str
    x: float
    y: float
    yaw: float


def path_length(path_msg: Path) -> float:
    total = 0.0
    poses = path_msg.poses
    for i in range(1, len(poses)):
        dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
        dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
        total += math.hypot(dx, dy)
    return total


class GoalRunMetrics:
    def __init__(self) -> None:
        self.dispatched_at: Optional[float] = None
        self.first_cmd_at: Optional[float] = None
        self.first_nonzero_cmd_at: Optional[float] = None
        self.first_progress_at: Optional[float] = None
        self.cmd_samples = 0
        self.nonzero_cmd_samples = 0
        self.zero_cmd_streak = 0
        self.max_zero_cmd_streak = 0
        self.max_cmd_linear = 0.0
        self.max_cmd_angular = 0.0
        self.odom_samples = 0
        self.max_odom_linear = 0.0
        self.max_odom_angular = 0.0
        self.plan_updates = 0
        self.local_plan_updates = 0
        self.initial_global_plan_length = None
        self.latest_global_plan_length = None
        self.max_global_plan_length = 0.0
        self.min_global_plan_length = None
        self.local_plan_lengths: List[float] = []
        self.latest_local_plan_length = None

    def on_plan(self, length: float) -> None:
        self.plan_updates += 1
        if self.initial_global_plan_length is None:
            self.initial_global_plan_length = length
        self.latest_global_plan_length = length
        self.max_global_plan_length = max(self.max_global_plan_length, length)
        if self.min_global_plan_length is None:
            self.min_global_plan_length = length
        else:
            self.min_global_plan_length = min(self.min_global_plan_length, length)

    def on_local_plan(self, length: float) -> None:
        self.local_plan_updates += 1
        self.local_plan_lengths.append(length)
        self.latest_local_plan_length = length

    def on_cmd(self, linear: float, angular: float, now_sec: float) -> None:
        self.cmd_samples += 1
        self.max_cmd_linear = max(self.max_cmd_linear, abs(linear))
        self.max_cmd_angular = max(self.max_cmd_angular, abs(angular))
        if self.first_cmd_at is None:
            self.first_cmd_at = now_sec
        if abs(linear) > 1e-3 or abs(angular) > 1e-3:
            self.nonzero_cmd_samples += 1
            self.max_zero_cmd_streak = max(self.max_zero_cmd_streak, self.zero_cmd_streak)
            self.zero_cmd_streak = 0
            if self.first_nonzero_cmd_at is None:
                self.first_nonzero_cmd_at = now_sec
        else:
            self.zero_cmd_streak += 1
            self.max_zero_cmd_streak = max(self.max_zero_cmd_streak, self.zero_cmd_streak)

    def on_odom(self, linear: float, angular: float, now_sec: float) -> None:
        self.odom_samples += 1
        self.max_odom_linear = max(self.max_odom_linear, abs(linear))
        self.max_odom_angular = max(self.max_odom_angular, abs(angular))
        if self.first_progress_at is None and (abs(linear) > 0.02 or abs(angular) > 0.05):
            self.first_progress_at = now_sec

    def snapshot(self, end_time: float) -> Dict[str, object]:
        mean_local_plan = (
            sum(self.local_plan_lengths) / len(self.local_plan_lengths)
            if self.local_plan_lengths else None
        )
        return {
            'cmd_samples': self.cmd_samples,
            'nonzero_cmd_samples': self.nonzero_cmd_samples,
            'plan_updates': self.plan_updates,
            'local_plan_updates': self.local_plan_updates,
            'initial_global_plan_length': self.initial_global_plan_length,
            'latest_global_plan_length': self.latest_global_plan_length,
            'max_global_plan_length': self.max_global_plan_length,
            'min_global_plan_length': self.min_global_plan_length,
            'mean_local_plan_length': mean_local_plan,
            'latest_local_plan_length': self.latest_local_plan_length,
            'max_cmd_linear': self.max_cmd_linear,
            'max_cmd_angular': self.max_cmd_angular,
            'max_odom_linear': self.max_odom_linear,
            'max_odom_angular': self.max_odom_angular,
            'time_to_first_cmd_s': (
                round(self.first_cmd_at - self.dispatched_at, 2)
                if self.first_cmd_at is not None and self.dispatched_at is not None
                else None
            ),
            'time_to_first_nonzero_cmd_s': (
                round(self.first_nonzero_cmd_at - self.dispatched_at, 2)
                if self.first_nonzero_cmd_at is not None and self.dispatched_at is not None
                else None
            ),
            'time_to_first_progress_s': (
                round(self.first_progress_at - self.dispatched_at, 2)
                if self.first_progress_at is not None and self.dispatched_at is not None
                else None
            ),
            'max_zero_cmd_streak_samples': self.max_zero_cmd_streak,
            'end_time_s': round(end_time, 2),
        }


class NavLoopRegression(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__('nav_loop_regression')
        self._args = args
        self._goals: List[GoalSpec] = build_goals(args)
        self._goal_index = 0
        self._total_runs = args.loops * len(self._goals) if args.loops is not None else args.cycles
        self._goal_handle = None
        self._result_future = None
        self._goal_future = None
        self._run_active = False
        self._run_done = False
        self._result_status = None
        self._current_result = None
        self._current_seq = 0
        self._current_goal_start = None
        self._current_goal_name = ''
        self._next_dispatch_at = 0.0
        self._goal_stats: List[Dict[str, object]] = []
        self._current_events = Counter()
        self._current_metrics: Optional[GoalRunMetrics] = None
        self._current_goal_spec: Optional[GoalSpec] = None
        self._recent_rosout: List[str] = []
        self._latest_cmd = {'linear_x': 0.0, 'angular_z': 0.0}
        self._latest_pose = {'x': None, 'y': None, 'yaw': None}
        self._last_trace_at = 0.0
        self._trace_interval_sec = 0.2
        self._log_patterns = {
            'failed_to_make_progress': 'Failed to make progress',
            'no_valid_trajectories': 'No valid trajectories',
            'trajectory_hits_obstacle': 'Trajectory Hits Obstacle',
            'controller_patience_exceeded': 'Controller patience exceeded',
            'clear_local_costmap': 'clear entirely the local_costmap',
            'goal_succeeded_log': 'Goal succeeded',
        }

        log_dir = os.path.expanduser('~/nav_regression_logs')
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_name = (
            f'nav_loop_{stamp}_'
            f'{sanitize_name(args.map_name)}_'
            f'cycles{args.cycles}.log'
        )
        self._log_path = os.path.join(log_dir, file_name)
        self._log_fp = open(self._log_path, 'w', encoding='utf-8')
        self._trace_path = self._log_path[:-4] + '.jsonl'
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
        self.create_subscription(Twist, '/rover_odo', self._odom_cb, qos_sensor)
        self.create_subscription(Odometry, '/lio_odom', self._lio_odom_cb, qos_sensor)

        self._timer = self.create_timer(0.5, self._tick)
        self._write_trace({
            'type': 'meta',
            'log_file': self._log_path,
            'trace_file': self._trace_path,
            'map_name': args.map_name,
            'timeout_s': args.timeout,
            'settle_s': args.settle,
            'total_runs': self._total_runs,
            'goals': [
                {'name': g.name, 'x': g.x, 'y': g.y, 'yaw': g.yaw}
                for g in self._goals
            ],
        })
        self._write(
            f'loop regression logger -> {self._log_path}\n'
            + '\n'.join(
                f'{goal.name}=({goal.x:.3f}, {goal.y:.3f}, yaw={goal.yaw:.3f})'
                for goal in self._goals
            ) + '\n'
            + f'total_runs={self._total_runs}, timeout={args.timeout}s, settle={args.settle}s'
        )

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
        if self._run_done:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        if not self._client.server_is_ready():
            if not self._client.wait_for_server(timeout_sec=0.1):
                self._write('waiting for navigate_to_pose action server...')
                return

        if not self._run_active:
            if now_sec < self._next_dispatch_at:
                return
            self._dispatch_next_goal()
            return

        elapsed = (
            self.get_clock().now() - self._current_goal_start
        ).nanoseconds / 1e9
        if elapsed > self._args.timeout:
            self._write(
                f'{self._current_goal_name} timeout after {elapsed:.1f}s, canceling goal')
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._finalize_goal('timeout')

    def _dispatch_next_goal(self) -> None:
        if self._current_seq >= self._total_runs:
            self._finish_run()
            return

        goal = self._goals[self._goal_index]
        self._goal_index = (self._goal_index + 1) % len(self._goals)
        self._current_seq += 1
        self._current_goal_name = f'run_{self._current_seq:02d}_{goal.name}'
        self._current_goal_spec = goal
        self._current_events = Counter()
        self._current_metrics = GoalRunMetrics()
        self._run_active = True
        self._current_goal_start = self.get_clock().now()
        self._current_metrics.dispatched_at = self._current_goal_start.nanoseconds / 1e9
        self._last_trace_at = 0.0

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
        self._goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        self._goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        try:
            self._goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self._write(f'{self._current_goal_name} goal request failed: {exc}')
            self._finalize_goal('request_failed')
            return

        if not self._goal_handle.accepted:
            self._write(f'{self._current_goal_name} goal rejected')
            self._finalize_goal('rejected')
            return

        self._write(f'{self._current_goal_name} accepted')
        self._write_trace({
            'type': 'goal_accepted',
            'run': self._current_goal_name,
        })
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if feedback.distance_remaining > 0.0:
            self.get_logger().debug(
                f'{self._current_goal_name} '
                f'distance_remaining={feedback.distance_remaining:.3f} '
                f'navigation_time={feedback.navigation_time.sec:.1f}s'
            )

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
        self._latest_cmd = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z,
        }
        self._current_metrics.on_cmd(msg.linear.x, msg.angular.z, now_sec)

    def _odom_cb(self, msg: Twist) -> None:
        if not self._run_active or self._current_metrics is None:
            return
        now_sec = self._now_sec()
        self._current_metrics.on_odom(
            msg.linear.x,
            msg.angular.z,
            now_sec,
        )

    def _lio_odom_cb(self, msg: Odometry) -> None:
        if not self._run_active or self._current_metrics is None:
            return
        pose = msg.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z),
        )
        self._latest_pose = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw,
        }
        now_sec = self._now_sec()
        if now_sec - self._last_trace_at < self._trace_interval_sec:
            return
        self._last_trace_at = now_sec
        self._write_trace({
            'type': 'sample',
            'run': self._current_goal_name,
            'goal': (
                {
                    'name': self._current_goal_spec.name,
                    'x': self._current_goal_spec.x,
                    'y': self._current_goal_spec.y,
                    'yaw': self._current_goal_spec.yaw,
                } if self._current_goal_spec is not None else None
            ),
            'pose': self._latest_pose,
            'cmd': self._latest_cmd,
            'odom': {
                'linear_x': msg.twist.twist.linear.x,
                'angular_z': msg.twist.twist.angular.z,
            },
            'global_plan_length': self._current_metrics.latest_global_plan_length,
            'local_plan_length': self._current_metrics.latest_local_plan_length,
        })

    def _result_cb(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self._write(f'{self._current_goal_name} result error: {exc}')
            self._finalize_goal('result_error')
            return

        self._result_status = result.status
        self._current_result = result.result
        status_name = self._status_name(result.status)
        self._write(f'{self._current_goal_name} finished with status={status_name}')
        self._finalize_goal(status_name)

    def _finalize_goal(self, outcome: str) -> None:
        if not self._run_active:
            return

        elapsed = (
            self.get_clock().now() - self._current_goal_start
        ).nanoseconds / 1e9
        row = {
            'name': self._current_goal_name,
            'outcome': outcome,
            'elapsed_s': round(elapsed, 2),
            'events': dict(self._current_events),
            'metrics': (
                self._current_metrics.snapshot(
                    self.get_clock().now().nanoseconds / 1e9
                )
                if self._current_metrics is not None else {}
            ),
        }
        self._goal_stats.append(row)
        self._write_trace({
            'type': 'goal_summary',
            'run': self._current_goal_name,
            'goal': (
                {
                    'name': self._current_goal_spec.name,
                    'x': self._current_goal_spec.x,
                    'y': self._current_goal_spec.y,
                    'yaw': self._current_goal_spec.yaw,
                } if self._current_goal_spec is not None else None
            ),
            'outcome': outcome,
            'elapsed_s': round(elapsed, 2),
            'events': dict(self._current_events),
            'metrics': row['metrics'],
        })
        self._write(
            f'{self._current_goal_name} summary -> '
            f'outcome={outcome}, elapsed={elapsed:.1f}s, events={dict(self._current_events)}, '
            f'metrics={row["metrics"]}'
        )

        self._run_active = False
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._result_status = None
        self._current_result = None
        self._current_goal_name = ''
        self._current_goal_spec = None
        self._current_events = Counter()
        self._current_metrics = None

        if self._current_seq >= self._total_runs:
            self._finish_run()
        elif self._args.settle > 0.0:
            self._next_dispatch_at = (
                self.get_clock().now().nanoseconds / 1e9
            ) + self._args.settle
            self._write(f'settling for {self._args.settle:.1f}s before next goal')

    def _finish_run(self) -> None:
        if self._run_done:
            return
        self._run_done = True

        counts = defaultdict(int)
        event_totals = Counter()
        for item in self._goal_stats:
            counts[item['outcome']] += 1
            event_totals.update(item['events'])

        self._write('=== regression summary ===')
        self._write(f'total_runs={len(self._goal_stats)} outcomes={dict(counts)}')
        self._write(f'event_totals={dict(event_totals)}')
        for item in self._goal_stats:
            self._write(
                f"{item['name']}: outcome={item['outcome']} "
                f"elapsed={item['elapsed_s']}s events={item['events']} "
                f"metrics={item['metrics']}"
            )
        self._write(f'log_file={self._log_path}')
        self._write(f'trace_file={self._trace_path}')
        self._write_trace({
            'type': 'run_summary',
            'outcomes': dict(counts),
            'event_totals': dict(event_totals),
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
    parser.add_argument(
        '--goal', nargs=4, action='append', metavar=('NAME', 'X', 'Y', 'YAW'),
        help='ordered goal list item in map frame; repeat for 3+ point loops'
    )
    parser.add_argument(
        '--goal-a', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
        help='goal A in map frame'
    )
    parser.add_argument(
        '--goal-b', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
        help='goal B in map frame'
    )
    parser.add_argument(
        '--cycles', type=int, default=6,
        help='total goal dispatch count, alternating A/B'
    )
    parser.add_argument(
        '--loops', type=int,
        help='full sequence repetitions; total runs = loops * goal_count'
    )
    parser.add_argument(
        '--timeout', type=float, default=180.0,
        help='per-goal timeout seconds'
    )
    parser.add_argument(
        '--settle', type=float, default=3.0,
        help='wait after each goal before sending the next one'
    )
    parser.add_argument(
        '--map-name', default='unknown_map',
        help='used only in log filename'
    )
    args = parser.parse_args()

    if args.goal:
        if len(args.goal) < 2:
            parser.error('--goal requires at least 2 entries')
        if args.goal_a or args.goal_b:
            parser.error('use either repeated --goal or --goal-a/--goal-b, not both')
    else:
        if args.goal_a is None or args.goal_b is None:
            parser.error('either repeated --goal or both --goal-a and --goal-b are required')

    if args.loops is not None and args.loops <= 0:
        parser.error('--loops must be > 0')
    if args.cycles <= 0:
        parser.error('--cycles must be > 0')

    return args


def build_goals(args: argparse.Namespace) -> List[GoalSpec]:
    if args.goal:
        goals: List[GoalSpec] = []
        for raw_goal in args.goal:
            name, x, y, yaw = raw_goal
            goals.append(GoalSpec(str(name), float(x), float(y), float(yaw)))
        return goals

    return [
        GoalSpec('goal_a', args.goal_a[0], args.goal_a[1], args.goal_a[2]),
        GoalSpec('goal_b', args.goal_b[0], args.goal_b[1], args.goal_b[2]),
    ]


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = NavLoopRegression(args)
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
