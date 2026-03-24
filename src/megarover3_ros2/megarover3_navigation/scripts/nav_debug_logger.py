#!/usr/bin/env python3
"""
Nav2 导航调试日志记录器

记录导航过程中的关键数据：速度指令、机器人位姿、路径规划、行为树状态等。
输出到终端 + 日志文件，方便事后分析。

使用方法:
  ros2 run megarover3_navigation nav_debug_logger.py
  # 或直接运行:
  python3 nav_debug_logger.py

日志文件保存到: ~/nav_debug_logs/nav_YYYYMMDD_HHMMSS.log
"""

import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from tf2_ros import Buffer, TransformListener
from rcl_interfaces.msg import Log


def quat_to_yaw(q):
    """四元数 → yaw (度)"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def path_length(path_msg):
    """计算路径总长度 (米)"""
    pts = path_msg.poses
    total = 0.0
    for i in range(1, len(pts)):
        dx = pts[i].pose.position.x - pts[i - 1].pose.position.x
        dy = pts[i].pose.position.y - pts[i - 1].pose.position.y
        total += math.hypot(dx, dy)
    return total


def fmt_pose(pose):
    """格式化 Pose 为字符串"""
    p = pose.position
    yaw = quat_to_yaw(pose.orientation)
    return f"({p.x:+.3f}, {p.y:+.3f}) yaw={yaw:+.1f}°"


class NavDebugLogger(Node):
    def __init__(self):
        super().__init__('nav_debug_logger')

        # 创建日志目录和文件
        log_dir = os.path.expanduser('~/nav_debug_logs')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = os.path.join(log_dir, f'nav_{ts}.log')
        self._log_file = open(self._log_path, 'w')
        self._log(f'=== Nav Debug Logger 启动 === 日志: {self._log_path}')

        # TF
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # 状态跟踪
        self._last_cmd_time = 0.0
        self._last_pose_log_time = 0.0
        self._nav_active = False
        self._goal_pose = None
        self._last_cmd = None
        self._phase = 'IDLE'  # IDLE / ROTATING / MOVING / GOAL_REACHED
        self._phase_start_time = 0.0
        self._cmd_count = 0
        self._zero_cmd_count = 0

        # --- 订阅 ---

        # 速度指令 (发给底盘的)
        self.create_subscription(Twist, '/rover_twist', self._cmd_cb, 10)

        # 导航目标
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)

        # 全局路径
        self.create_subscription(Path, '/plan', self._plan_cb, 10)

        # 平滑后的全局路径
        self.create_subscription(Path, '/plan_smoothed', self._plan_smoothed_cb, 10)

        # 局部路径
        self.create_subscription(Path, '/local_plan', self._local_plan_cb, 10)

        # 控制器变换后的全局路径
        self.create_subscription(Path, '/transformed_global_plan', self._transformed_plan_cb, 10)

        # 里程计
        self.create_subscription(Odometry, '/lio_odom', self._odom_cb, 10)

        # RViz 2D Pose Estimate
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initialpose_cb, 10)

        # /rosout (捕获 Nav2 节点的日志)
        qos_rosout = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=50)
        self.create_subscription(Log, '/rosout', self._rosout_cb, qos_rosout)

        # 定时输出状态摘要
        self.create_timer(2.0, self._summary_timer_cb)

        self._log('等待导航目标...')

    # ------------------------------------------------------------------
    # 日志输出
    # ------------------------------------------------------------------
    def _log(self, msg, level='INFO'):
        now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        line = f'[{now}] [{level}] {msg}'
        self.get_logger().info(msg)
        self._log_file.write(line + '\n')
        self._log_file.flush()

    # ------------------------------------------------------------------
    # 导航目标
    # ------------------------------------------------------------------
    def _goal_cb(self, msg: PoseStamped):
        self._goal_pose = msg
        self._nav_active = True
        self._cmd_count = 0
        self._zero_cmd_count = 0
        self._phase = 'RECEIVED'
        self._phase_start_time = time.time()
        yaw = quat_to_yaw(msg.pose.orientation)
        self._log(
            f'★ 收到导航目标: '
            f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}) '
            f'yaw={yaw:.1f}° frame={msg.header.frame_id}')

        # 计算目标相对于机器人的方向
        self._log_goal_analysis()

    def _log_goal_analysis(self):
        """分析目标相对机器人的方向和距离"""
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            r_yaw = quat_to_yaw(tf.transform.rotation)

            gx = self._goal_pose.pose.position.x
            gy = self._goal_pose.pose.position.y
            g_yaw = quat_to_yaw(self._goal_pose.pose.orientation)

            dx = gx - rx
            dy = gy - ry
            dist = math.hypot(dx, dy)
            bearing = math.degrees(math.atan2(dy, dx))
            turn_needed = bearing - r_yaw
            # 归一化到 [-180, 180]
            turn_needed = (turn_needed + 180) % 360 - 180

            self._log(
                f'  机器人位置: ({rx:.3f}, {ry:.3f}) yaw={r_yaw:.1f}°')
            self._log(
                f'  目标距离: {dist:.2f}m  方位角: {bearing:.1f}°  '
                f'需旋转: {turn_needed:.1f}°')
            self._log(
                f'  目标朝向: {g_yaw:.1f}°  到达后需旋转到: {g_yaw:.1f}°')
        except Exception:
            self._log('  (无法获取 map→base_footprint TF)', 'WARN')

    # ------------------------------------------------------------------
    # 速度指令
    # ------------------------------------------------------------------
    def _cmd_cb(self, msg: Twist):
        now = time.time()
        self._last_cmd = msg
        self._cmd_count += 1

        vx = msg.linear.x
        wz = msg.angular.z
        is_zero = abs(vx) < 0.001 and abs(wz) < 0.001

        if is_zero:
            self._zero_cmd_count += 1
        else:
            self._zero_cmd_count = 0

        # 判断运动阶段
        new_phase = self._phase
        if not is_zero:
            if abs(vx) < 0.01 and abs(wz) > 0.05:
                new_phase = 'ROTATING'
            elif abs(vx) > 0.01:
                new_phase = 'MOVING'

        if new_phase != self._phase and new_phase in ('ROTATING', 'MOVING'):
            duration = now - self._phase_start_time if self._phase_start_time > 0 else 0
            self._log(
                f'▶ 阶段切换: {self._phase} → {new_phase} '
                f'(上一阶段持续 {duration:.1f}s)')
            self._phase = new_phase
            self._phase_start_time = now

        # 每 0.5 秒记录一次非零速度
        if not is_zero and (now - self._last_cmd_time) > 0.5:
            self._last_cmd_time = now
            self._log(
                f'  cmd_vel: vx={vx:+.3f} m/s  wz={wz:+.3f} rad/s  '
                f'[{self._phase}]')

        # 连续零速度检测 (可能卡住)
        if self._zero_cmd_count == 40 and self._nav_active:  # ~2s at 20Hz
            self._log('⚠ 连续 2s 零速度指令 (可能卡住或到达目标)', 'WARN')

    # ------------------------------------------------------------------
    # 全局路径
    # ------------------------------------------------------------------
    def _plan_cb(self, msg: Path):
        n = len(msg.poses)
        if n < 2:
            self._log(f'全局路径: {n} 个点 (太短)')
            return
        length = path_length(msg)
        start = fmt_pose(msg.poses[0].pose)
        end = fmt_pose(msg.poses[-1].pose)
        self._log(
            f'📍 全局路径: {n} 个点, 长度 {length:.2f}m')
        self._log(
            f'  起点: {start}')
        self._log(
            f'  终点: {end}')

    def _plan_smoothed_cb(self, msg: Path):
        n = len(msg.poses)
        if n < 2:
            return
        length = path_length(msg)
        self._log(f'  平滑路径: {n} 个点, 长度 {length:.2f}m')

    # ------------------------------------------------------------------
    # 局部路径
    # ------------------------------------------------------------------
    def _local_plan_cb(self, msg: Path):
        now = time.time()
        # 局部路径更新频率高，每 2 秒记录一次
        if not hasattr(self, '_last_local_plan_time'):
            self._last_local_plan_time = 0.0
        if now - self._last_local_plan_time < 2.0:
            return
        self._last_local_plan_time = now

        n = len(msg.poses)
        if n < 2:
            return
        length = path_length(msg)
        start = fmt_pose(msg.poses[0].pose)
        end = fmt_pose(msg.poses[-1].pose)
        self._log(
            f'  局部路径: {n} 点, 长{length:.2f}m  '
            f'从{start} → {end}')

    def _transformed_plan_cb(self, msg: Path):
        now = time.time()
        if not hasattr(self, '_last_tfplan_time'):
            self._last_tfplan_time = 0.0
        if now - self._last_tfplan_time < 3.0:
            return
        self._last_tfplan_time = now

        n = len(msg.poses)
        if n < 2:
            return
        # 检查路径方向：第一个点到第二个点的方向
        dx = msg.poses[1].pose.position.x - msg.poses[0].pose.position.x
        dy = msg.poses[1].pose.position.y - msg.poses[0].pose.position.y
        heading = math.degrees(math.atan2(dy, dx))
        self._log(
            f'  变换后路径: {n} 点, 起始方向={heading:.1f}° '
            f'(base_footprint 帧)')

    # ------------------------------------------------------------------
    # 里程计 + TF
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        now = time.time()
        if now - self._last_pose_log_time < 3.0:
            return
        if not self._nav_active:
            return
        self._last_pose_log_time = now

        # 从 TF 获取 map 帧下的位姿
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = quat_to_yaw(tf.transform.rotation)

            info = f'  位姿(map): ({x:+.3f}, {y:+.3f}) yaw={yaw:+.1f}°'

            # 如果有目标，显示到目标的距离
            if self._goal_pose:
                gx = self._goal_pose.pose.position.x
                gy = self._goal_pose.pose.position.y
                dist = math.hypot(gx - x, gy - y)
                g_yaw = quat_to_yaw(self._goal_pose.pose.orientation)
                yaw_err = (g_yaw - yaw + 180) % 360 - 180
                info += f'  距目标: {dist:.2f}m  yaw差: {yaw_err:+.1f}°'

            self._log(info)
        except Exception:
            pass

        # 也记录 odom 帧数据用于对比
        odom_yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self._log(
            f'  里程计(odom): ({msg.pose.pose.position.x:+.3f}, '
            f'{msg.pose.pose.position.y:+.3f}) yaw={odom_yaw:+.1f}°  '
            f'实际速度: vx={vx:+.3f} wz={wz:+.3f}')

    # ------------------------------------------------------------------
    # RViz 初始位姿
    # ------------------------------------------------------------------
    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self._log(
            f'收到 2D Pose Estimate: '
            f'({msg.pose.pose.position.x:.3f}, '
            f'{msg.pose.pose.position.y:.3f}) yaw={yaw:.1f}°')

    # ------------------------------------------------------------------
    # /rosout — 捕获 Nav2 关键日志
    # ------------------------------------------------------------------
    def _rosout_cb(self, msg: Log):
        name = msg.name
        text = msg.msg

        # 只关注 Nav2 相关节点
        nav2_nodes = (
            'controller_server', 'planner_server', 'bt_navigator',
            'behavior_server', 'smoother_server', 'velocity_smoother',
            'waypoint_follower', 'global_costmap', 'local_costmap',
        )
        if not any(n in name for n in nav2_nodes):
            return

        # 过滤噪音
        skip_patterns = ('Publishing the map', 'Received a')
        if any(p in text for p in skip_patterns):
            return

        # 关键事件
        important_patterns = (
            'Goal', 'goal', 'abort', 'cancel', 'fail', 'succeed',
            'recovery', 'Activat', 'Deactivat', 'error', 'Error',
            'path', 'Plan', 'plan', 'timeout', 'Timeout',
            'progress', 'stuck', 'oscillat', 'invalid',
        )
        if any(p in text for p in important_patterns):
            level = 'WARN' if msg.level >= 30 else 'INFO'
            self._log(f'  [{name}] {text}', level)

    # ------------------------------------------------------------------
    # 定时摘要
    # ------------------------------------------------------------------
    def _summary_timer_cb(self):
        if not self._nav_active:
            return

        # 检查是否到达目标 (连续零速度 > 3s)
        if self._zero_cmd_count > 60 and self._phase != 'IDLE':
            if self._goal_pose:
                try:
                    tf = self._tf_buf.lookup_transform(
                        'map', 'base_footprint', rclpy.time.Time())
                    gx = self._goal_pose.pose.position.x
                    gy = self._goal_pose.pose.position.y
                    dist = math.hypot(
                        gx - tf.transform.translation.x,
                        gy - tf.transform.translation.y)
                    if dist < 0.3:
                        self._log(f'✓ 到达目标 (距离 {dist:.2f}m)')
                        self._phase = 'IDLE'
                        self._nav_active = False
                    else:
                        self._log(
                            f'⚠ 停止但未到达目标 (距离 {dist:.2f}m)',
                            'WARN')
                except Exception:
                    pass

    def destroy_node(self):
        self._log('=== Nav Debug Logger 停止 ===')
        self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavDebugLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
