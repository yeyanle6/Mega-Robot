"""
Headless ROS2 navigation test engine.

Runs in a background QThread.  Sends NavigateToPose goals, records
trajectories in the **map** frame, and emits Qt signals for the UI.

Key design choices:
  - All poses are transformed to the map frame via TF (odom→map).
  - A single MultiThreadedExecutor.spin() handles all callbacks.
    The test loop polls future.done() instead of calling
    spin_until_future_complete (avoids double-spin on one node).
  - Trajectory is sampled once per cycle in the polling loop (no
    duplicate recording from feedback callback).
"""

import math
import time
import threading
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from action_msgs.msg import GoalStatus
import tf2_ros
from rcl_interfaces.msg import Log
from std_msgs.msg import String


# ======================================================================
# Data types
# ======================================================================

@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0
    stamp_sec: float = 0.0


@dataclass
class RobotSnapshot:
    """Full robot state at one instant."""
    t: float              # unix timestamp
    x: float              # map-frame position (m)
    y: float              # map-frame position (m)
    yaw: float            # map-frame heading (rad)
    cmd_vx: float = 0.0   # /cmd_vel linear.x (m/s)
    cmd_wz: float = 0.0   # /cmd_vel angular.z (rad/s)
    actual_vx: float = 0.0  # actual forward speed from pose diff (m/s)
    actual_wz: float = 0.0  # actual angular speed from yaw diff (rad/s)
    goal_dist: float = 0.0   # distance to current goal (m)
    goal_yaw_err: float = 0.0  # heading error to goal (rad)


@dataclass
class RunRecord:
    """One waypoint-to-waypoint navigation run."""
    goal: Pose2D = None
    start_pose: Pose2D = None
    status: str = 'pending'
    start_time: float = 0.0
    end_time: float = 0.0

    theoretical_path: List[Pose2D] = field(default_factory=list)
    # Each entry: (timestamp, [(x, y), ...]) — captures every re-plan
    global_plan_history: List[Tuple[float, list]] = field(default_factory=list)
    # Each entry: (timestamp, [(x, y), ...]) — controller output every cycle
    local_plan_history: List[Tuple[float, list]] = field(default_factory=list)
    snapshots: List[RobotSnapshot] = field(default_factory=list)

    events: List[Tuple[float, str]] = field(default_factory=list)

    @property
    def actual_trajectory(self) -> List[Pose2D]:
        """Backward-compatible pose trajectory derived from snapshots."""
        return [Pose2D(s.x, s.y, s.yaw, s.t) for s in self.snapshots]

    @property
    def global_plan(self) -> List[Pose2D]:
        """Latest global plan (backward compat for deviation calc & replay)."""
        if not self.global_plan_history:
            return []
        _, pts = self.global_plan_history[-1]
        return [Pose2D(x, y) for x, y in pts]

    @property
    def local_plan(self) -> List[Pose2D]:
        """Latest local plan (backward compat for replay)."""
        if not self.local_plan_history:
            return []
        _, pts = self.local_plan_history[-1]
        return [Pose2D(x, y) for x, y in pts]

    @property
    def duration(self):
        if self.end_time > 0 and self.start_time > 0:
            return self.end_time - self.start_time
        return 0.0

    @property
    def theoretical_length(self):
        return _path_length(self.theoretical_path)

    @property
    def actual_length(self):
        return _path_length(self.actual_trajectory)

    @property
    def mean_lateral_deviation(self):
        return _mean_deviation(self.actual_trajectory, self.theoretical_path)

    @property
    def max_lateral_deviation(self):
        return _max_deviation(self.actual_trajectory, self.theoretical_path)

    @property
    def deviation_from_global(self):
        """Mean lateral deviation of actual trajectory from global plan."""
        return _mean_deviation(self.actual_trajectory, self.global_plan)

    @property
    def recovery_count(self):
        """Costmap clears and recovery behavior invocations."""
        return sum(1 for _, msg in self.events if '[COSTMAP_CLEAR]' in msg)

    @property
    def abort_count(self):
        """Navigation aborts (distinct from recoveries)."""
        return sum(1 for _, msg in self.events if '[NAV_ABORT]' in msg)

    @property
    def stall_count(self):
        return sum(1 for _, msg in self.events if '[NAV_STALL]' in msg)

    def to_dict(self):
        """Serialize to a JSON-compatible dict."""
        def _pose(p):
            if p is None:
                return None
            return {'x': p.x, 'y': p.y, 'yaw': p.yaw}
        return {
            'goal': _pose(self.goal),
            'start_pose': _pose(self.start_pose),
            'status': self.status,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'duration': self.duration,
            'theoretical_length': self.theoretical_length,
            'actual_length': self.actual_length,
            'mean_lateral_deviation': self.mean_lateral_deviation,
            'max_lateral_deviation': self.max_lateral_deviation,
            'deviation_from_global': self.deviation_from_global,
            'recovery_count': self.recovery_count,
            'abort_count': self.abort_count,
            'stall_count': self.stall_count,
            'theoretical_path': [(p.x, p.y) for p in self.theoretical_path],
            'global_plan_history': [
                {'t': t, 'path': pts}
                for t, pts in self.global_plan_history],
            'local_plan_history': [
                {'t': t, 'path': pts}
                for t, pts in self.local_plan_history],
            'snapshots': [
                {
                    't': s.t, 'x': s.x, 'y': s.y, 'yaw': s.yaw,
                    'cmd_vx': s.cmd_vx, 'cmd_wz': s.cmd_wz,
                    'actual_vx': s.actual_vx, 'actual_wz': s.actual_wz,
                    'goal_dist': s.goal_dist, 'goal_yaw_err': s.goal_yaw_err,
                }
                for s in self.snapshots],
            'events': [{'time': t, 'msg': m} for t, m in self.events],
        }


# ======================================================================
# Path geometry helpers
# ======================================================================

def _path_length(pts: List[Pose2D]) -> float:
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(pts)):
        dx = pts[i].x - pts[i - 1].x
        dy = pts[i].y - pts[i - 1].y
        total += math.sqrt(dx * dx + dy * dy)
    return total


def _closest_point_on_path(px, py, path: List[Pose2D]) -> float:
    if len(path) < 2:
        return math.hypot(px - path[0].x, py - path[0].y) if path else 0.0
    min_d = float('inf')
    for i in range(len(path) - 1):
        ax, ay = path[i].x, path[i].y
        bx, by = path[i + 1].x, path[i + 1].y
        abx, aby = bx - ax, by - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-12:
            d = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
            d = math.hypot(px - (ax + t * abx), py - (ay + t * aby))
        if d < min_d:
            min_d = d
    return min_d


def _mean_deviation(actual: List[Pose2D], ref: List[Pose2D]) -> float:
    if not actual or len(ref) < 2:
        return 0.0
    return float(np.mean([_closest_point_on_path(p.x, p.y, ref) for p in actual]))


def _max_deviation(actual: List[Pose2D], ref: List[Pose2D]) -> float:
    if not actual or len(ref) < 2:
        return 0.0
    return float(np.max([_closest_point_on_path(p.x, p.y, ref) for p in actual]))


# ======================================================================
# Engine
# ======================================================================

class NavTestEngine(QThread):
    """Background thread running the ROS2 navigation test loop."""

    # Signals → UI (all poses are in map frame)
    pose_in_map_updated = pyqtSignal(float, float, float)   # x, y, yaw
    global_plan_updated = pyqtSignal(list)                   # [(x,y), ...]
    local_plan_updated = pyqtSignal(list)                    # [(x,y), ...]
    theoretical_path_received = pyqtSignal(list)             # [(x,y), ...]
    run_started = pyqtSignal(int, object)                    # idx, RunRecord
    run_finished = pyqtSignal(int, object)                   # idx, RunRecord
    event_logged = pyqtSignal(float, str)                    # ts, msg
    engine_ready = pyqtSignal()
    engine_stopped = pyqtSignal()
    test_batch_finished = pyqtSignal()
    pose_refresh_result = pyqtSignal(bool, float, float, float)  # ok, x, y, yaw
    # initial pose verification: (ok, pub_x, pub_y, pub_yaw, actual_x, actual_y, actual_yaw, offset_m)
    initial_pose_verified = pyqtSignal(bool, float, float, float, float, float, float, float)

    SAMPLE_INTERVAL = 0.1   # seconds between trajectory samples
    _POSE_EMIT_INTERVAL = 0.1  # throttle pose signal to ~10 Hz

    _LOG_PATTERNS = [
        ('Failed to make progress', 'NAV_STALL'),
        ('No valid trajectories', 'NO_VALID_TRAJ'),
        ('Trajectory Hits Obstacle', 'TRAJ_OBSTACLE'),
        ('clear entirely', 'COSTMAP_CLEAR'),
        ('Aborting', 'NAV_ABORT'),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._stop_flag = False
        self._stop_test_flag = False
        self._test_pending = False
        self._goal_queue: List[Pose2D] = []
        self._num_cycles = 1
        self._testing = False
        self.records: List[RunRecord] = []

        self._node: Optional[Node] = None
        self._nav_client = None
        self._plan_client = None
        self._executor = None

        # Map-frame pose (updated by TF lookup, not raw odom)
        self._map_pose: Optional[Pose2D] = None
        self._tf_buffer = None
        self._last_pose_emit = 0.0
        self._tf_diag_done = False   # one-shot TF diagnostic

        # Velocity cache (updated by /cmd_vel callback)
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0

        # Bumper safety state
        self._bumper_status = 'IDLE'
        self._pose_refresh_requested = False
        self._initialpose_pub = None
        self._pending_initial_pose = None  # (x, y, yaw) or None

    # ------------------------------------------------------------------
    # Public API (called from UI thread)
    # ------------------------------------------------------------------
    def start_test(self, goals: List[Pose2D], num_cycles: int = 1):
        """Begin a test run (engine must already be started)."""
        self._goal_queue = list(goals)
        self._num_cycles = num_cycles
        self._stop_test_flag = False
        self._test_pending = True

    def request_stop_test(self):
        """Stop current test but keep monitoring."""
        self._stop_test_flag = True

    def request_shutdown(self):
        """Stop everything and exit the engine thread."""
        self._stop_test_flag = True
        self._stop_flag = True

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        """Queue a /initialpose publish (thread-safe, called from UI thread)."""
        self._pending_initial_pose = (x, y, yaw)

    def request_pose_refresh(self) -> bool:
        """Request a pose refresh to be handled inside the engine thread."""
        self._pose_refresh_requested = True
        return True

    def is_testing(self):
        """True while a test batch is executing."""
        return self._testing

    def _should_stop_test(self):
        return self._stop_flag or self._stop_test_flag

    # ------------------------------------------------------------------
    # Thread entry
    # ------------------------------------------------------------------
    def run(self):
        if not rclpy.ok():
            rclpy.init()

        self._node = Node('nav_test_workbench_engine')
        cb_group = ReentrantCallbackGroup()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # TF for odom → map
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)

        # Action clients
        self._nav_client = ActionClient(
            self._node, NavigateToPose, 'navigate_to_pose',
            callback_group=cb_group)
        self._plan_client = ActionClient(
            self._node, ComputePathToPose, 'compute_path_to_pose',
            callback_group=cb_group)

        # Subscribers — odom kept for compatibility/diagnostics
        self._node.create_subscription(
            Odometry, '/lio_odom', self._odom_cb, sensor_qos,
            callback_group=cb_group)
        self._node.create_subscription(
            Path, '/plan', self._plan_cb, 10,
            callback_group=cb_group)
        self._node.create_subscription(
            Path, '/local_plan', self._local_plan_cb, sensor_qos,
            callback_group=cb_group)

        # Event monitoring: /rosout for nav2 failure patterns
        rosout_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=100)
        self._node.create_subscription(
            Log, '/rosout', self._rosout_cb, rosout_qos,
            callback_group=cb_group)

        # Event monitoring: bumper safety
        self._node.create_subscription(
            String, '/bumper_safety/status', self._bumper_status_cb, 10,
            callback_group=cb_group)
        self._node.create_subscription(
            String, '/bumper_safety/detail', self._bumper_detail_cb, 10,
            callback_group=cb_group)

        # /cmd_vel subscriber for velocity command recording
        self._node.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10,
            callback_group=cb_group)

        # /initialpose publisher for 2D Pose Estimate
        self._initialpose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Poll TF directly so UI pose tracking does not depend on odom callback timing.
        self._node.create_timer(
            0.1, self._poll_map_pose, callback_group=cb_group)

        # Single executor — spin in background daemon thread.
        # The test loop NEVER calls spin_until_future_complete;
        # it polls future.done() instead.
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)

        spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        spin_thread.start()

        self.engine_ready.emit()
        self._stop_flag = False

        try:
            # Monitoring loop — stays alive, dispatches test runs on demand
            while not self._stop_flag:
                if self._pending_initial_pose is not None:
                    x, y, yaw = self._pending_initial_pose
                    self._pending_initial_pose = None
                    self._do_publish_initial_pose(x, y, yaw)
                if self._pose_refresh_requested:
                    self._pose_refresh_requested = False
                    ok = self._wait_for_map_pose(1.0, force_refresh=True)
                    if ok and self._map_pose is not None:
                        p = self._map_pose
                        self.pose_refresh_result.emit(True, p.x, p.y, p.yaw)
                    else:
                        self.pose_refresh_result.emit(False, 0.0, 0.0, 0.0)
                if self._test_pending:
                    self._test_pending = False
                    self._testing = True
                    self._run_test_loop()
                    self._testing = False
                    self.test_batch_finished.emit()
                time.sleep(0.1)
        finally:
            try:
                self._executor.shutdown()
                self._node.destroy_node()
            except Exception:
                pass
            self.engine_stopped.emit()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    # Fallback TF frame chains, tried in priority order.
    # Each entry: (target_frame, source_frame, label)
    _TF_CHAINS = [
        ('map', 'base_footprint', 'map→base_footprint'),
        ('map', 'lio_base',       'map→lio_base'),
        ('odom', 'base_footprint', 'odom→base_footprint'),
        ('odom', 'lio_base',       'odom→lio_base'),
    ]

    def _update_map_pose_from_tf(self, stamp_sec: float = 0.0,
                                timeout_sec: float = 0.0) -> bool:
        """Refresh the robot pose from TF, trying multiple frame chains.

        Priority: map→base_footprint > map→lio_base > odom→base_footprint
                  > odom→lio_base.  Uses the first chain that succeeds.

        Args:
            timeout_sec: TF lookup timeout. Use 0 from executor callbacks
                         (non-blocking to avoid starving executor threads).
                         Use >0 from the monitoring thread where blocking is safe.
        """
        dur = rclpy.duration.Duration(seconds=timeout_sec)
        for target, source, label in self._TF_CHAINS:
            try:
                tf = self._tf_buffer.lookup_transform(
                    target, source, rclpy.time.Time(), timeout=dur)
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                stamp = stamp_sec if stamp_sec > 0.0 else time.time()
                self._map_pose = Pose2D(t.x, t.y, yaw, stamp)

                # Log which chain succeeded (once)
                if not self._tf_diag_done:
                    self._tf_diag_done = True
                    self._log(f'[TF] Using frame chain: {label}')

                # Throttle UI signal emission to ~10 Hz
                now = time.time()
                if now - self._last_pose_emit >= self._POSE_EMIT_INTERVAL:
                    self._last_pose_emit = now
                    self.pose_in_map_updated.emit(t.x, t.y, yaw)
                return True
            except Exception:
                continue

        # All chains failed — log diagnostic (once per 30 seconds)
        now = time.time()
        last_diag = getattr(self, '_last_tf_diag_time', 0.0)
        if now - last_diag >= 30.0:
            self._last_tf_diag_time = now
            frames = ''
            try:
                frames = self._tf_buffer.all_frames_as_string()
            except Exception:
                frames = '<unavailable>'
            self._log(
                f'[TF_DIAG] All frame chains failed.\n'
                f'  Available frames:\n{frames}')
        return False

    def _odom_cb(self, msg: Odometry):
        """Use odom arrival as an extra trigger, but not the only pose source."""
        self._odom_count = getattr(self, '_odom_count', 0) + 1
        if self._odom_count == 1:
            self._log('[DIAG] First /lio_odom received')
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._update_map_pose_from_tf(stamp)

    def _cmd_vel_cb(self, msg: Twist):
        """Cache latest velocity commands."""
        self._cmd_vx = msg.linear.x
        self._cmd_wz = msg.angular.z

    def _poll_map_pose(self):
        """Periodic TF polling keeps the UI alive even if odom callback timing is poor."""
        self._tf_poll_n = getattr(self, '_tf_poll_n', 0) + 1
        ok = self._update_map_pose_from_tf()
        if ok:
            self._tf_poll_ok = getattr(self, '_tf_poll_ok', 0) + 1
        # Diagnostic every ~10 seconds (100 ticks at 10 Hz)
        if self._tf_poll_n % 100 == 0:
            p = self._map_pose
            pos = f'({p.x:.3f},{p.y:.3f})' if p else 'None'
            self._log(
                f'[TF_POLL] ticks={self._tf_poll_n} ok={getattr(self, "_tf_poll_ok", 0)} '
                f'pose={pos} last_emit={self._last_pose_emit:.1f}')

    def _get_tf_to_map(self, frame_id: str):
        """Get 2D transform (tx, ty, yaw) from frame_id → map. None on failure."""
        if frame_id == 'map':
            return (0.0, 0.0, 0.0)
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0))
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (t.x, t.y, yaw)
        except Exception:
            return None

    def _points_to_map(self, pts, frame_id: str):
        """Transform list of (x, y) from frame_id to map frame."""
        tf_info = self._get_tf_to_map(frame_id)
        if tf_info is None or tf_info == (0.0, 0.0, 0.0):
            return pts
        tx, ty, yaw = tf_info
        c, s = math.cos(yaw), math.sin(yaw)
        return [(tx + c * x - s * y, ty + s * x + c * y) for x, y in pts]

    def _plan_cb(self, msg: Path):
        frame = msg.header.frame_id or 'map'
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        pts = self._points_to_map(pts, frame)
        self.global_plan_updated.emit(pts)
        if self.records and self.records[-1].status == 'running':
            self.records[-1].global_plan_history.append(
                (time.time(), list(pts)))

    def _local_plan_cb(self, msg: Path):
        frame = msg.header.frame_id or 'odom'
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        pts = self._points_to_map(pts, frame)
        self.local_plan_updated.emit(pts)
        if self.records and self.records[-1].status == 'running':
            self.records[-1].local_plan_history.append(
                (time.time(), list(pts)))

    def _rosout_cb(self, msg: Log):
        """Monitor /rosout for navigation-related events."""
        try:
            level = int(msg.level)
            if level < int(Log.WARN):
                return
            text = msg.msg
            name = msg.name
            # Check known navigation failure/recovery patterns
            for pattern, tag in self._LOG_PATTERNS:
                if pattern.lower() in text.lower():
                    self._log(f'[{tag}] {name}: {text}')
                    if self.records and self.records[-1].status == 'running':
                        self.records[-1].events.append(
                            (time.time(), f'[{tag}] {text}'))
                    return
            # Log any ERROR from nav2-related nodes
            if level >= int(Log.ERROR):
                nav_names = ('nav2', 'controller', 'planner', 'bt_navigator',
                             'smoother', 'behavior_server', 'waypoint_follower')
                if any(n in name.lower() for n in nav_names):
                    self._log(f'[NAV_ERROR] {name}: {text}')
                    if self.records and self.records[-1].status == 'running':
                        self.records[-1].events.append(
                            (time.time(), f'[NAV_ERROR] {text}'))
        except Exception:
            pass  # never let a single rosout message kill the executor

    def _bumper_status_cb(self, msg: String):
        """Monitor bumper safety status changes."""
        old = self._bumper_status
        self._bumper_status = msg.data
        if msg.data != old and msg.data != 'IDLE':
            self._log(f'[BUMPER] {msg.data}')
            if self.records and self.records[-1].status == 'running':
                self.records[-1].events.append(
                    (time.time(), f'[BUMPER] {msg.data}'))

    def _bumper_detail_cb(self, msg: String):
        """Log bumper detail messages."""
        if msg.data:
            self._log(f'[BUMPER] {msg.data}')
            if self.records and self.records[-1].status == 'running':
                self.records[-1].events.append(
                    (time.time(), f'[BUMPER_DETAIL] {msg.data}'))

    # ------------------------------------------------------------------
    # Test loop
    # ------------------------------------------------------------------
    def _run_test_loop(self):
        self._log('Waiting for NavigateToPose server...')
        if not self._wait_for_server(self._nav_client, 15.0):
            self._log('NavigateToPose server not available')
            return
        self._log('NavigateToPose ready')

        self._log('Waiting for ComputePathToPose server...')
        plan_available = self._wait_for_server(self._plan_client, 10.0)
        if not plan_available:
            self._log('ComputePathToPose not available — will skip theoretical path')

        if not self._goal_queue:
            self._log('No goals configured')
            return

        run_index = 0
        for cycle in range(self._num_cycles):
            for goal_pose in self._goal_queue:
                if self._should_stop_test():
                    return
                if not self._wait_for_map_pose(5.0):
                    record = RunRecord(goal=goal_pose)
                    record.status = 'failed'
                    record.start_time = time.time()
                    record.end_time = record.start_time
                    record.events.append((record.end_time, 'No valid map pose available before run start'))
                    self.records.append(record)
                    self.run_started.emit(run_index, record)
                    self.run_finished.emit(run_index, record)
                    self._log('Skipping run: no valid map pose available')
                    run_index += 1
                    continue
                self._execute_one_run(run_index, goal_pose)
                run_index += 1

    def _execute_one_run(self, run_index: int, goal: Pose2D):
        record = RunRecord(goal=goal)
        record.start_pose = self._map_pose   # already in map frame
        record.start_time = time.time()
        record.status = 'running'
        self.records.append(record)
        self.run_started.emit(run_index, record)

        # 1. Request theoretical path
        self._request_theoretical_path(record)

        # 2. Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose2d_to_stamped(goal)

        send_future = self._nav_client.send_goal_async(goal_msg)

        # Poll for acceptance (no spin_until_future_complete)
        if not self._wait_future(send_future, 10.0):
            record.status = 'failed'
            record.end_time = time.time()
            record.events.append((time.time(), 'Goal send timeout'))
            self.run_finished.emit(run_index, record)
            return

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            record.status = 'failed'
            record.end_time = time.time()
            record.events.append((time.time(), 'Goal rejected'))
            self.run_finished.emit(run_index, record)
            return

        # 3. Poll for result, sampling full robot state at fixed interval
        result_future = goal_handle.get_result_async()
        while not result_future.done() and not self._should_stop_test():
            if self._map_pose:
                p = self._map_pose
                now = time.time()
                g = record.goal
                dx = g.x - p.x
                dy = g.y - p.y
                goal_dist = math.sqrt(dx * dx + dy * dy)
                goal_yaw_err = math.atan2(dy, dx) - p.yaw
                # Normalize to [-pi, pi]
                goal_yaw_err = math.atan2(
                    math.sin(goal_yaw_err), math.cos(goal_yaw_err))

                # Compute actual velocity from pose differences
                actual_vx = 0.0
                actual_wz = 0.0
                if record.snapshots:
                    prev = record.snapshots[-1]
                    dt = now - prev.t
                    if dt > 0.01:
                        ddx = p.x - prev.x
                        ddy = p.y - prev.y
                        # Project displacement onto heading → forward speed
                        avg_yaw = (p.yaw + prev.yaw) / 2.0
                        actual_vx = (ddx * math.cos(avg_yaw)
                                     + ddy * math.sin(avg_yaw)) / dt
                        # Angular velocity from yaw difference
                        dyaw = math.atan2(math.sin(p.yaw - prev.yaw),
                                          math.cos(p.yaw - prev.yaw))
                        actual_wz = dyaw / dt

                record.snapshots.append(RobotSnapshot(
                    t=now,
                    x=p.x, y=p.y, yaw=p.yaw,
                    cmd_vx=self._cmd_vx, cmd_wz=self._cmd_wz,
                    actual_vx=actual_vx, actual_wz=actual_wz,
                    goal_dist=goal_dist, goal_yaw_err=goal_yaw_err,
                ))
            time.sleep(self.SAMPLE_INTERVAL)

        if self._should_stop_test():
            goal_handle.cancel_goal_async()
            record.status = 'canceled'
            record.end_time = time.time()
            self.run_finished.emit(run_index, record)
            return

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            record.status = 'succeeded'
        else:
            record.status = 'failed'
            record.events.append((time.time(), f'Goal status: {result.status}'))

        record.end_time = time.time()
        self.run_finished.emit(run_index, record)

    def _request_theoretical_path(self, record: RunRecord):
        """Call ComputePathToPose to get the pre-navigation planned path."""
        if not self._plan_client.server_is_ready():
            return
        try:
            goal_msg = ComputePathToPose.Goal()
            if self._map_pose:
                goal_msg.start = self._pose2d_to_stamped(self._map_pose)
                goal_msg.use_start = True
            else:
                goal_msg.use_start = False
            goal_msg.goal = self._pose2d_to_stamped(record.goal)

            send_future = self._plan_client.send_goal_async(goal_msg)
            if not self._wait_future(send_future, 5.0):
                return
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                return

            result_future = goal_handle.get_result_async()
            if not self._wait_future(result_future, 5.0):
                return
            result = result_future.result()
            if result and result.result:
                path = result.result.path
                pts = [Pose2D(p.pose.position.x, p.pose.position.y)
                       for p in path.poses]
                record.theoretical_path = pts
                self.theoretical_path_received.emit([(p.x, p.y) for p in pts])
        except Exception as e:
            self._log(f'Theoretical path error: {e}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _do_publish_initial_pose(self, x: float, y: float, yaw: float):
        """Publish a PoseWithCovarianceStamped to /initialpose, then verify."""
        if self._initialpose_pub is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Covariance: same defaults as rviz2
        cov = [0.0] * 36
        cov[0] = 0.25   # x variance
        cov[7] = 0.25   # y variance
        cov[35] = 0.06853891945200942  # yaw variance
        msg.pose.covariance = cov
        self._initialpose_pub.publish(msg)
        self._log(f'[INITIAL_POSE] Published: ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)')

        # Verify: wait for localization to process, then compare TF pose
        time.sleep(1.0)
        ok = self._update_map_pose_from_tf(timeout_sec=0.5)
        if ok and self._map_pose is not None:
            p = self._map_pose
            offset = math.hypot(p.x - x, p.y - y)
            yaw_diff = abs(math.degrees(p.yaw - yaw))
            if yaw_diff > 180:
                yaw_diff = 360 - yaw_diff
            is_ok = offset < 0.5
            status = 'OK' if is_ok else 'OFFSET'
            self._log(
                f'[INITIAL_POSE] Verify {status}: TF=({p.x:.3f}, {p.y:.3f}, '
                f'yaw={math.degrees(p.yaw):.1f}°) '
                f'offset={offset:.3f}m yaw_diff={yaw_diff:.1f}°')
            self.initial_pose_verified.emit(
                is_ok, x, y, yaw, p.x, p.y, p.yaw, offset)
        else:
            self._log('[INITIAL_POSE] Verify FAILED: no TF pose available')
            self.initial_pose_verified.emit(False, x, y, yaw, 0.0, 0.0, 0.0, -1.0)

    def _pose2d_to_stamped(self, pose: Pose2D) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.orientation.z = math.sin(pose.yaw / 2.0)
        msg.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return msg

    def _wait_future(self, future, timeout_sec: float) -> bool:
        """Poll future.done() without calling spin_until_future_complete."""
        t0 = time.time()
        while not future.done():
            if self._should_stop_test():
                return False
            if time.time() - t0 > timeout_sec:
                return False
            time.sleep(0.05)
        return True

    def _wait_for_map_pose(self, timeout_sec: float, force_refresh: bool = False) -> bool:
        """Wait until a valid map-frame pose is available.

        This runs in the monitoring thread (not an executor callback),
        so blocking TF lookups are safe — the executor's daemon threads
        can still process TF listener callbacks while we block here.
        """
        if self._map_pose is not None and not force_refresh:
            return True
        t0 = time.time()
        while time.time() - t0 <= timeout_sec:
            if self._should_stop_test():
                return False
            remaining = max(0.1, timeout_sec - (time.time() - t0))
            lookup_timeout = min(0.5, remaining)
            if self._update_map_pose_from_tf(timeout_sec=lookup_timeout):
                return True
            time.sleep(0.05)
        return self._map_pose is not None

    def _wait_for_server(self, client: ActionClient, timeout: float) -> bool:
        """Poll server_is_ready() so we don't block the executor."""
        t0 = time.time()
        while not client.server_is_ready():
            if self._should_stop_test() or time.time() - t0 > timeout:
                return False
            time.sleep(0.2)
        return True

    def _log(self, msg: str):
        self.event_logged.emit(time.time(), msg)
        if self._node:
            self._node.get_logger().info(msg)
