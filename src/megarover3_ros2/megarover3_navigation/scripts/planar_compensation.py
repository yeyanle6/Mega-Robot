#!/usr/bin/env python3
"""
Planar motion compensation for FASTLIO2.

FASTLIO2 estimates 6DOF pose without planar constraints, causing
Z/Roll/Pitch drift and jitter for ground robots. This node publishes
a map->odom TF that compensates for this, constraining the robot
to move on a horizontal plane (Z=const, Roll=0, Pitch=0).

Subscribes: /lio_odom (nav_msgs/Odometry) from FASTLIO2
Publishes:  TF map -> odom (compensating Z/Roll/Pitch)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class PlanarCompensationNode(Node):
    def __init__(self):
        super().__init__('planar_compensation')

        self.declare_parameter('odom_topic', '/lio_odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        self.z_ref = None  # Captured from first odometry message
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10)

        self.get_logger().info(
            f'Planar compensation node started, subscribing to {odom_topic}')

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        if self.z_ref is None:
            self.z_ref = pos.z
            self.get_logger().info(f'Reference Z set to {self.z_ref:.4f}m')

        q_ob = (ori.x, ori.y, ori.z, ori.w)
        roll, pitch, yaw = _quat_to_euler(q_ob)

        # Desired map->base_link: keep X/Y/Yaw, lock Z=z_ref, Roll=0, Pitch=0
        q_desired = _euler_to_quat(0.0, 0.0, yaw)

        # map->odom rotation = desired * inverse(odom->base)
        q_comp = _quat_mult(q_desired, _quat_inv(q_ob))

        # map->odom translation: keep XY unchanged, force Z = z_ref
        # t_map_base = R_comp * t_odom_base + t_comp
        # We want t_map_base = (tx, ty, z_ref)
        R = _quat_to_mat(q_comp)
        tx, ty, tz = pos.x, pos.y, pos.z
        rx = R[0][0] * tx + R[0][1] * ty + R[0][2] * tz
        ry = R[1][0] * tx + R[1][1] * ty + R[1][2] * tz
        rz = R[2][0] * tx + R[2][1] * ty + R[2][2] * tz

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = tx - rx
        t.transform.translation.y = ty - ry
        t.transform.translation.z = self.z_ref - rz
        t.transform.rotation.x = q_comp[0]
        t.transform.rotation.y = q_comp[1]
        t.transform.rotation.z = q_comp[2]
        t.transform.rotation.w = q_comp[3]

        self.tf_broadcaster.sendTransform(t)


# ---------- Quaternion utilities (x, y, z, w convention) ----------

def _quat_to_euler(q):
    """Quaternion (x,y,z,w) -> (roll, pitch, yaw)."""
    x, y, z, w = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _euler_to_quat(roll, pitch, yaw):
    """(roll, pitch, yaw) -> quaternion (x,y,z,w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_inv(q):
    """Inverse of unit quaternion (x,y,z,w)."""
    return (-q[0], -q[1], -q[2], q[3])


def _quat_mult(a, b):
    """Hamilton product of quaternions (x,y,z,w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_to_mat(q):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix (list of lists)."""
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def main(args=None):
    rclpy.init(args=args)
    node = PlanarCompensationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
