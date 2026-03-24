#!/usr/bin/env python3
"""
Latch-stop navigation when the physical bumper input is triggered.

/rover_sensor.data[0] = MU16_IM_DI (digital input)
/rover_sensor.data[1] = battery voltage [mV]
"""

import copy

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import Trigger


class BumperSafetyMonitor(Node):
    BUMPER_BITS = {
        0: 'front_left',
        1: 'front_center',
        2: 'front_right',
        4: 'rear_right',
        5: 'rear_center',
        6: 'rear_left',
    }
    FRONT_MASK = (1 << 0) | (1 << 1) | (1 << 2)
    REAR_MASK = (1 << 4) | (1 << 5) | (1 << 6)

    def __init__(self) -> None:
        super().__init__('bumper_safety_monitor')

        self.declare_parameter('enabled', True)
        self.declare_parameter('trigger_mask', 0xFFFF)
        self.declare_parameter('zero_cmd_rate_hz', 20.0)
        self.declare_parameter('stop_hold_s', 0.3)
        self.declare_parameter('retreat_distance_m', 0.05)
        self.declare_parameter('retreat_speed_mps', 0.05)
        self.declare_parameter('auto_reset_after_s', 5.0)
        self.declare_parameter('auto_resume_goal', True)
        self.declare_parameter('trigger_debounce_s', 0.05)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._trigger_mask = int(self.get_parameter('trigger_mask').value)
        zero_cmd_rate_hz = float(self.get_parameter('zero_cmd_rate_hz').value)
        self._stop_hold_s = max(float(self.get_parameter('stop_hold_s').value), 0.0)
        self._retreat_distance_m = float(self.get_parameter('retreat_distance_m').value)
        self._retreat_speed_mps = max(float(self.get_parameter('retreat_speed_mps').value), 0.01)
        self._auto_reset_after_s = max(float(self.get_parameter('auto_reset_after_s').value), 0.0)
        self._auto_resume_goal = bool(self.get_parameter('auto_resume_goal').value)
        self._trigger_debounce_ns = int(
            max(float(self.get_parameter('trigger_debounce_s').value), 0.0) * 1e9
        )

        self._latched = False
        self._last_raw_value = 0
        self._last_trigger_value = 0
        self._last_trigger_labels = []
        self._pending_trigger_value = 0
        self._pending_trigger_started_ns = 0
        self._motion_phase = 'idle'
        self._stop_hold_until_ns = 0
        self._retreat_until_ns = 0
        self._resume_at_ns = 0
        self._retreat_twist = Twist()
        self._last_goal_pose = None

        self._cmd_pub = self.create_publisher(Twist, '/rover_twist', 10)
        self._status_pub = self.create_publisher(String, '/bumper_safety/status', 10)
        self._detail_pub = self.create_publisher(String, '/bumper_safety/detail', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.create_service(Trigger, '/bumper_safety/reset', self._handle_reset)

        sensor_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Int16MultiArray, '/rover_sensor', self._sensor_cb, sensor_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_cb, sensor_qos)
        self.create_timer(1.0 / max(zero_cmd_rate_hz, 1.0), self._hold_zero_velocity)

        self._publish_status('IDLE')
        self.get_logger().info(
            'bumper safety enabled=%s trigger_mask=0x%04X retreat_distance=%.3fm '
            'retreat_speed=%.3fm/s auto_reset_after=%.1fs auto_resume_goal=%s '
            'trigger_debounce=%.3fs'
            % (
                self._enabled,
                self._trigger_mask,
                self._retreat_distance_m,
                self._retreat_speed_mps,
                self._auto_reset_after_s,
                self._auto_resume_goal,
                self._trigger_debounce_ns / 1e9,
            )
        )

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        self._last_goal_pose = copy.deepcopy(msg)

    def _sensor_cb(self, msg: Int16MultiArray) -> None:
        if not self._enabled or len(msg.data) < 1:
            return

        raw_value = int(msg.data[0]) & 0xFFFF
        self._last_raw_value = raw_value
        triggered_bits = raw_value & self._trigger_mask

        if triggered_bits == 0:
            self._pending_trigger_value = 0
            self._pending_trigger_started_ns = 0
            if self._latched and self._motion_phase == 'waiting_clear':
                self._motion_phase = 'cooldown'
                self._resume_at_ns = (
                    self.get_clock().now().nanoseconds + int(self._auto_reset_after_s * 1e9)
                )
                self._publish_status('COOLDOWN')
                self._publish_detail(
                    f'bumper cleared, auto reset in {self._auto_reset_after_s:.1f}s'
                )
            return

        if self._latched:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._trigger_debounce_ns > 0:
            if triggered_bits != self._pending_trigger_value:
                self._pending_trigger_value = triggered_bits
                self._pending_trigger_started_ns = now_ns
                return
            if now_ns - self._pending_trigger_started_ns < self._trigger_debounce_ns:
                return
        self._pending_trigger_value = 0
        self._pending_trigger_started_ns = 0

        self._latched = True
        self._last_trigger_value = triggered_bits
        self._last_trigger_labels = self._decode_bits(triggered_bits)
        self._arm_retreat(triggered_bits)
        self._publish_status('TRIGGERED')
        self._publish_detail(
            f'trigger raw=0x{raw_value:04X} masked=0x{triggered_bits:04X} '
            f'labels={",".join(self._last_trigger_labels) if self._last_trigger_labels else "unknown"} '
            f'phase={self._motion_phase}'
        )
        self.get_logger().error(
            f'physical bumper triggered: raw=0x{raw_value:04X} masked=0x{triggered_bits:04X} '
            f'labels={self._last_trigger_labels}')

        self._motion_phase = 'stopping'
        self._stop_hold_until_ns = (
            self.get_clock().now().nanoseconds + int(self._stop_hold_s * 1e9)
        )
        self._cmd_pub.publish(Twist())
        self._cancel_navigation()

    def _cancel_navigation(self) -> None:
        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('navigate_to_pose cancel service unavailable')
            return

        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        future = self._cancel_client.call_async(request)
        future.add_done_callback(self._cancel_done_cb)

    def _cancel_done_cb(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'cancel navigation failed: {exc}')
            return
        self.get_logger().warn(f'cancel navigation response code={response.return_code}')

    def _hold_zero_velocity(self) -> None:
        if not self._latched:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._motion_phase == 'stopping':
            self._cmd_pub.publish(Twist())
            if now_ns < self._stop_hold_until_ns:
                return
            if self._retreat_until_ns > 0:
                self._motion_phase = 'retreating'
                self._publish_status('RETREATING')
                self._publish_detail(
                    f'retreat begin vx={self._retreat_twist.linear.x:+.3f} '
                    f'labels={",".join(self._last_trigger_labels) if self._last_trigger_labels else "unknown"}'
                )
            else:
                self._motion_phase = 'latched_stop'

        if self._motion_phase == 'retreating' and now_ns < self._retreat_until_ns:
            self._cmd_pub.publish(self._retreat_twist)
            return

        if self._motion_phase == 'retreating':
            self._motion_phase = 'waiting_clear'
            self._publish_status('WAITING_CLEAR')
            self._publish_detail(
                f'waiting_clear last_trigger=0x{self._last_trigger_value:04X} '
                f'labels={",".join(self._last_trigger_labels) if self._last_trigger_labels else "unknown"}'
            )
            self._cmd_pub.publish(Twist())
            return

        if self._motion_phase == 'cooldown':
            self._cmd_pub.publish(Twist())
            if now_ns < self._resume_at_ns:
                return
            self._auto_reset_and_resume()
            return

        self._cmd_pub.publish(Twist())

    def _handle_reset(self, request, response):
        del request
        self._clear_latch_state()
        self._publish_status('RESET')
        response.success = True
        response.message = (
            f'bumper safety reset, last_raw=0x{self._last_raw_value:04X}, '
            f'last_trigger=0x{self._last_trigger_value:04X}, '
            f'labels={",".join(self._last_trigger_labels) if self._last_trigger_labels else "none"}'
        )
        self.get_logger().info(response.message)
        return response

    def _publish_status(self, label: str) -> None:
        msg = String()
        msg.data = label
        self._status_pub.publish(msg)

    def _publish_detail(self, label: str) -> None:
        msg = String()
        msg.data = label
        self._detail_pub.publish(msg)

    def _decode_bits(self, raw_value: int):
        labels = []
        for bit, name in sorted(self.BUMPER_BITS.items()):
            if raw_value & (1 << bit):
                labels.append(name)
        return labels

    def _arm_retreat(self, triggered_bits: int) -> None:
        front_hit = bool(triggered_bits & self.FRONT_MASK)
        rear_hit = bool(triggered_bits & self.REAR_MASK)

        self._retreat_twist = Twist()
        if front_hit and not rear_hit:
            self._retreat_twist.linear.x = -self._retreat_speed_mps
            self._motion_phase = 'retreating'
        elif rear_hit and not front_hit:
            self._retreat_twist.linear.x = self._retreat_speed_mps
            self._motion_phase = 'retreating'
        else:
            self._motion_phase = 'latched_stop'

        if self._motion_phase == 'retreating':
            duration_sec = self._retreat_distance_m / self._retreat_speed_mps
            self._retreat_until_ns = (
                self.get_clock().now().nanoseconds
                + int((self._stop_hold_s + duration_sec) * 1e9)
            )
            self.get_logger().warn(
                f'bumper retreat armed: stop_hold={self._stop_hold_s:.2f}s '
                f'vx={self._retreat_twist.linear.x:+.3f} m/s for {duration_sec:.2f}s'
            )
        else:
            self._retreat_until_ns = 0

    def _clear_latch_state(self) -> None:
        self._latched = False
        self._pending_trigger_value = 0
        self._pending_trigger_started_ns = 0
        self._motion_phase = 'idle'
        self._stop_hold_until_ns = 0
        self._retreat_until_ns = 0
        self._resume_at_ns = 0
        self._retreat_twist = Twist()

    def _auto_reset_and_resume(self) -> None:
        self._clear_latch_state()
        if self._auto_resume_goal and self._last_goal_pose is not None:
            goal = copy.deepcopy(self._last_goal_pose)
            goal.header.stamp = self.get_clock().now().to_msg()
            self._publish_status('AUTO_RESUME')
            self._publish_detail('auto reset complete, republishing last goal_pose')
            self._goal_pub.publish(goal)
            self.get_logger().warn('bumper auto reset complete, republished last goal_pose')
            return

        self._publish_status('AUTO_RESET')
        self._publish_detail('auto reset complete, no saved goal to resume')
        self.get_logger().warn('bumper auto reset complete')


def main() -> None:
    rclpy.init()
    node = BumperSafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
