#!/usr/bin/env python3
"""
Navigation initializer for MegaRover3.

Calls the /localizer/relocalize service to load the PCD map and set
the initial pose, enabling the localizer to publish the map->odom TF.

Also subscribes to /initialpose (RViz "2D Pose Estimate") so users can
re-relocalize at runtime.

Publishes status to /nav_initializer/status (std_msgs/String):
  WAITING      - waiting for the relocalize service to become available
  INITIALIZING - calling the relocalize service
  VERIFYING    - relocalize request accepted, waiting for localizer validity
  LOCALIZED    - localizer reports a valid alignment
  FAILED       - relocalize failed after all retries or validation timeout
"""

import math
import os

import yaml

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from interface.srv import IsValid
from interface.srv import Relocalize


def euler_from_quaternion(q):
    """Extract yaw from a quaternion (x, y, z, w)."""
    # roll (x-axis)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert (roll, pitch, yaw) to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(a, b):
    """Hamilton product of quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotation_matrix_from_quaternion(q):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def rotate_vector(rotation_matrix, vector):
    """Multiply a 3x3 rotation matrix by a 3D vector."""
    return (
        rotation_matrix[0][0] * vector[0] +
        rotation_matrix[0][1] * vector[1] +
        rotation_matrix[0][2] * vector[2],
        rotation_matrix[1][0] * vector[0] +
        rotation_matrix[1][1] * vector[1] +
        rotation_matrix[1][2] * vector[2],
        rotation_matrix[2][0] * vector[0] +
        rotation_matrix[2][1] * vector[1] +
        rotation_matrix[2][2] * vector[2],
    )


class NavInitializer(Node):
    def __init__(self):
        super().__init__('nav_initializer')

        # Declare parameters
        self.declare_parameter('pcd_map_path', '')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('service_timeout_sec', 30.0)
        self.declare_parameter('validation_timeout_sec', 30.0)
        self.declare_parameter('validation_poll_sec', 0.5)
        self.declare_parameter('max_retries', 10)
        self.declare_parameter('last_pose_file',
                               '~/.ros/megarover3_last_pose.yaml')
        self.declare_parameter('use_last_pose', False)
        self.declare_parameter('initial_pose_frame', 'base_footprint')
        self.declare_parameter('localizer_pose_frame', 'lio_base')

        # Read parameters
        self._pcd_path = self.get_parameter(
            'pcd_map_path').get_parameter_value().string_value
        self._init_x = self.get_parameter(
            'initial_x').get_parameter_value().double_value
        self._init_y = self.get_parameter(
            'initial_y').get_parameter_value().double_value
        self._init_z = self.get_parameter(
            'initial_z').get_parameter_value().double_value
        self._init_yaw = self.get_parameter(
            'initial_yaw').get_parameter_value().double_value
        self._timeout = self.get_parameter(
            'service_timeout_sec').get_parameter_value().double_value
        self._validation_timeout = self.get_parameter(
            'validation_timeout_sec').get_parameter_value().double_value
        self._validation_poll = self.get_parameter(
            'validation_poll_sec').get_parameter_value().double_value
        self._max_retries = self.get_parameter(
            'max_retries').get_parameter_value().integer_value
        self._last_pose_file = os.path.expanduser(
            self.get_parameter(
                'last_pose_file').get_parameter_value().string_value)
        self._use_last_pose = self.get_parameter(
            'use_last_pose').get_parameter_value().bool_value
        self._initial_pose_frame = self.get_parameter(
            'initial_pose_frame').get_parameter_value().string_value
        self._localizer_pose_frame = self.get_parameter(
            'localizer_pose_frame').get_parameter_value().string_value

        if not self._pcd_path:
            self.get_logger().error('pcd_map_path is required but not set')
            raise SystemExit(1)

        # Status publisher
        self._status_pub = self.create_publisher(
            String, '/nav_initializer/status', 10)

        # Service client  (localizer_node is in namespace "localizer",
        # service name is "relocalize" → /localizer/relocalize)
        self._client = self.create_client(
            Relocalize, '/localizer/relocalize')
        self._check_client = self.create_client(
            IsValid, '/localizer/relocalize_check')

        self._validate_timer = None
        self._validate_start_time = None
        self._pending_pose = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribe to /initialpose for re-relocalization from RViz
        self._initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self._initialpose_cb, 10)

        # Kick off the initialization sequence on a timer so we don't block
        # the constructor.
        self._init_timer = self.create_timer(0.1, self._start_init)

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------
    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')

    # ------------------------------------------------------------------
    # Startup sequence
    # ------------------------------------------------------------------
    def _start_init(self):
        """One-shot timer callback that triggers the first relocalize."""
        self._init_timer.cancel()

        # Optionally load last pose
        if self._use_last_pose:
            self._load_last_pose()

        self._wait_and_call(
            self._pcd_path,
            self._init_x, self._init_y, self._init_z,
            self._init_yaw, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Service interaction
    # ------------------------------------------------------------------
    def _wait_and_call(self, pcd_path, x, y, z, yaw, pitch, roll):
        """Wait for the service, then call relocalize with retries."""
        self._publish_status('WAITING')

        # First wait with the configured timeout
        if not self._client.wait_for_service(timeout_sec=self._timeout):
            self.get_logger().warn(
                f'Service not available after {self._timeout}s, '
                'will retry...')
            # Retry loop
            for i in range(self._max_retries):
                self.get_logger().info(
                    f'Retry {i + 1}/{self._max_retries} '
                    '— waiting 5s for service...')
                if self._client.wait_for_service(timeout_sec=5.0):
                    break
            else:
                self.get_logger().error(
                    'Relocalize service never became available')
                self._publish_status('FAILED')
                return

        if not self._check_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'Relocalize check service never became available')
            self._publish_status('FAILED')
            return

        self._call_relocalize(pcd_path, x, y, z, yaw, pitch, roll)

    def _call_relocalize(self, pcd_path, x, y, z, yaw, pitch, roll):
        """Send the relocalize request and handle the response."""
        self._publish_status('INITIALIZING')

        req = Relocalize.Request()
        req.pcd_path = pcd_path
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.pitch = float(pitch)
        req.roll = float(roll)

        self.get_logger().info(
            f'Calling relocalize: pcd={pcd_path}, '
            f'pose=({x:.3f}, {y:.3f}, {z:.3f}), yaw={yaw:.3f}')

        self._pending_pose = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'yaw': float(yaw),
        }
        future = self._client.call_async(req)
        future.add_done_callback(self._relocalize_done)

    def _convert_pose_to_localizer_frame(self, x, y, z, yaw, pitch, roll):
        """Convert RViz initial pose from robot display frame to localizer frame."""
        if self._initial_pose_frame == self._localizer_pose_frame:
            return x, y, z, yaw, pitch, roll

        try:
            tf = self._tf_buffer.lookup_transform(
                self._initial_pose_frame,
                self._localizer_pose_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'Failed to convert initial pose from {self._initial_pose_frame} '
                f'to {self._localizer_pose_frame}: {e}'
            )
            return x, y, z, yaw, pitch, roll

        q_map_initial = quaternion_from_euler(roll, pitch, yaw)
        q_initial_localizer = (
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )
        q_map_localizer = quaternion_multiply(q_map_initial, q_initial_localizer)

        rotation = rotation_matrix_from_quaternion(q_map_initial)
        offset = (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        )
        rotated_offset = rotate_vector(rotation, offset)
        localizer_roll, localizer_pitch, localizer_yaw = euler_from_quaternion(
            type('Quat', (), {
                'x': q_map_localizer[0],
                'y': q_map_localizer[1],
                'z': q_map_localizer[2],
                'w': q_map_localizer[3],
            })()
        )

        converted = (
            x + rotated_offset[0],
            y + rotated_offset[1],
            z + rotated_offset[2],
            localizer_yaw,
            localizer_pitch,
            localizer_roll,
        )
        self.get_logger().info(
            f'Converted initial pose {self._initial_pose_frame} -> '
            f'{self._localizer_pose_frame}: ({converted[0]:.3f}, '
            f'{converted[1]:.3f}, {converted[2]:.3f}), yaw={converted[3]:.3f}'
        )
        return converted

    def _relocalize_done(self, future):
        """Callback when the relocalize service responds."""
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self._publish_status('FAILED')
            return

        if result.success:
            self.get_logger().info(
                f'Relocalize request accepted: {result.message}')
            self._publish_status('VERIFYING')
            self._start_validation()
        else:
            self.get_logger().error(
                f'Relocalize failed: {result.message}')
            self._publish_status('FAILED')
            self._pending_pose = None

    def _start_validation(self):
        """Poll localizer validity before declaring localization success."""
        if self._validate_timer is not None:
            self._validate_timer.cancel()
            self.destroy_timer(self._validate_timer)
            self._validate_timer = None

        self._validate_start_time = self.get_clock().now()
        self._validate_timer = self.create_timer(
            self._validation_poll, self._check_localization)

    def _check_localization(self):
        """Call /localizer/relocalize_check until localization becomes valid."""
        elapsed = (
            self.get_clock().now() - self._validate_start_time
        ).nanoseconds / 1e9
        if elapsed > self._validation_timeout:
            self.get_logger().error(
                f'Localization validation timed out after {elapsed:.1f}s')
            self._stop_validation()
            self._publish_status('FAILED')
            self._pending_pose = None
            return

        req = IsValid.Request()
        req.code = 0
        future = self._check_client.call_async(req)
        future.add_done_callback(self._validation_done)

    def _validation_done(self, future):
        """Handle localizer validity response."""
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'Localization validity check failed: {e}')
            return

        if not result.valid:
            return

        self._stop_validation()
        if self._pending_pose is not None:
            self._init_x = self._pending_pose['x']
            self._init_y = self._pending_pose['y']
            self._init_z = self._pending_pose['z']
            self._init_yaw = self._pending_pose['yaw']
        self.get_logger().info('Localization validity confirmed by localizer')
        self._publish_status('LOCALIZED')
        self._save_last_pose()
        self._pending_pose = None

    def _stop_validation(self):
        """Stop the active localization validation timer, if any."""
        if self._validate_timer is None:
            return
        self._validate_timer.cancel()
        self.destroy_timer(self._validate_timer)
        self._validate_timer = None

    # ------------------------------------------------------------------
    # /initialpose callback (RViz "2D Pose Estimate")
    # ------------------------------------------------------------------
    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        """Re-relocalize when the user sets a pose in RViz."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(ori)

        self.get_logger().info(
            f'Received /initialpose: ({pos.x:.3f}, {pos.y:.3f}, '
            f'{pos.z:.3f}), yaw={yaw:.3f}')

        # Update cached pose
        self._init_x = pos.x
        self._init_y = pos.y
        self._init_z = pos.z
        self._init_yaw = yaw

        localizer_x, localizer_y, localizer_z, localizer_yaw, localizer_pitch, localizer_roll = (
            self._convert_pose_to_localizer_frame(
                pos.x, pos.y, pos.z, yaw, pitch, roll
            )
        )
        self._call_relocalize(
            self._pcd_path,
            localizer_x, localizer_y, localizer_z,
            localizer_yaw, localizer_pitch, localizer_roll)

    # ------------------------------------------------------------------
    # Pose persistence
    # ------------------------------------------------------------------
    def _save_last_pose(self):
        """Save the current pose to a YAML file."""
        data = {
            'x': float(self._init_x),
            'y': float(self._init_y),
            'z': float(self._init_z),
            'yaw': float(self._init_yaw),
            'pitch': 0.0,
            'roll': 0.0,
            'pcd_path': self._pcd_path,
        }
        try:
            os.makedirs(os.path.dirname(self._last_pose_file), exist_ok=True)
            with open(self._last_pose_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info(
                f'Saved pose to {self._last_pose_file}')
        except OSError as e:
            self.get_logger().warn(f'Failed to save pose: {e}')

    def _load_last_pose(self):
        """Load the last saved pose from a YAML file if it exists."""
        if not os.path.isfile(self._last_pose_file):
            self.get_logger().info(
                f'No saved pose file at {self._last_pose_file}')
            return
        try:
            with open(self._last_pose_file, 'r') as f:
                data = yaml.safe_load(f)
            if not isinstance(data, dict):
                return
            self._init_x = float(data.get('x', self._init_x))
            self._init_y = float(data.get('y', self._init_y))
            self._init_z = float(data.get('z', self._init_z))
            self._init_yaw = float(data.get('yaw', self._init_yaw))
            self.get_logger().info(
                f'Loaded saved pose: ({self._init_x:.3f}, '
                f'{self._init_y:.3f}, {self._init_z:.3f}), '
                f'yaw={self._init_yaw:.3f}')
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().warn(f'Failed to load saved pose: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = NavInitializer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node._stop_validation()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
