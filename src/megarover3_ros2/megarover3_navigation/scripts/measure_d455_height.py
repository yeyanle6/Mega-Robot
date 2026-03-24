#!/usr/bin/env python3
"""
Sensor Height Measurement Tool

Measures both MID360 and D455 mounting heights by analyzing ground plane
position in the base_footprint frame. Ground should appear at z≈0 if
sensor heights are correctly configured.

Principle:
  - base_footprint is defined at ground level.
  - FAST-LIO2 uses t_il z (= mid360_mount_z) to place lio_base relative
    to the LiDAR. If t_il z is correct, ground points from Patchwork++
    will appear at z≈0 in base_footprint.
  - D455 depth points are transformed to base_footprint via the URDF TF
    chain. If d455_front_mount_z is correct, D455 ground points will
    also appear at z≈0.
  - Any deviation from z=0 directly reveals the height configuration error.

Usage:
  1. Place robot on flat ground with clear space in front (1-3m).
  2. Start the SLAM system:
     ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam
  3. Run this script:
     python3 measure_d455_height.py
  4. The script collects 10 seconds of data, then reports corrections.
"""

import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ])


def fit_ground_plane(points, n_iter=300, threshold=0.03):
    """RANSAC ground plane fitting. Returns (normal, d, inlier_mask).
    Ground plane expected to be roughly horizontal (normal.z > 0.8).
    """
    best_inliers = 0
    best_model = None
    n = len(points)
    if n < 10:
        return None, None, None

    for _ in range(n_iter):
        idx = np.random.choice(n, 3, replace=False)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-8:
            continue
        normal /= norm_len
        if normal[2] < 0:
            normal = -normal
        if abs(normal[2]) < 0.8:
            continue
        d = -np.dot(normal, p0)
        dists = np.abs(points @ normal + d)
        inliers = np.sum(dists < threshold)
        if inliers > best_inliers:
            best_inliers = inliers
            best_model = (normal, d, dists < threshold)

    if best_model is None:
        return None, None, None

    # Refit with all inliers
    normal, d, mask = best_model
    inlier_pts = points[mask]
    centroid = inlier_pts.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_pts - centroid)
    normal = vh[2]
    if normal[2] < 0:
        normal = -normal
    d = -np.dot(normal, centroid)
    final_dists = np.abs(points @ normal + d)
    mask = final_dists < threshold
    return normal, d, mask


class SensorHeightMeasurer(Node):
    def __init__(self):
        super().__init__('sensor_height_measurer')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Patchwork++ publishes with RELIABLE + TRANSIENT_LOCAL
        from rclpy.qos import DurabilityPolicy
        patchwork_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Also subscribe to /body_cloud (RELIABLE) as fallback for ground detection
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.ground_frames = []
        self.body_frames = []       # raw /body_cloud for fallback ground detection
        self.d455_frames = []
        self.ground_frame_id = None
        self.body_frame_id = None
        self.d455_frame_id = None

        self.create_subscription(
            PointCloud2, '/patchworkpp/ground', self.ground_cb, patchwork_qos)
        self.create_subscription(
            PointCloud2, '/body_cloud', self.body_cb, reliable_qos)
        self.create_subscription(
            PointCloud2, '/camera/d455_front/depth/color/points', self.d455_cb, sensor_qos)

        self.get_logger().info('Waiting for data...')

    def ground_cb(self, msg):
        self.ground_frame_id = msg.header.frame_id
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) > 0:
            self.ground_frames.append(pts)

    def body_cb(self, msg):
        self.body_frame_id = msg.header.frame_id
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        # Keep points within 5m
        dist = np.linalg.norm(pts, axis=1)
        pts = pts[dist < 5.0]
        if len(pts) > 5000:
            idx = np.random.choice(len(pts), 5000, replace=False)
            pts = pts[idx]
        if len(pts) > 0:
            self.body_frames.append(pts)

    def d455_cb(self, msg):
        self.d455_frame_id = msg.header.frame_id
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        dist = np.linalg.norm(pts, axis=1)
        pts = pts[(dist > 0.2) & (dist < 2.0)]
        if len(pts) > 3000:
            idx = np.random.choice(len(pts), 3000, replace=False)
            pts = pts[idx]
        if len(pts) > 0:
            self.d455_frames.append(pts)

    def get_tf(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            R = quat_to_rot(q.x, q.y, q.z, q.w)
            T = np.array([t.x, t.y, t.z])
            return R, T
        except Exception as e:
            self.get_logger().error(f'TF {source}->{target}: {e}')
            return None, None

    def measure_ground_z(self, points_bf, label, x_range=2.0, y_min=0.3, y_max=3.0,
                          is_raw_cloud=False):
        """Measure ground plane z in base_footprint for a set of points.
        Returns (ground_z_mean, ground_z_std, n_inliers) or None if failed.
        """
        mask = (
            (np.abs(points_bf[:, 0]) < x_range) &
            (points_bf[:, 1] > y_min) & (points_bf[:, 1] < y_max)
        )
        fwd = points_bf[mask]
        print(f'  {label} region (|X|<{x_range}m, Y={y_min}~{y_max}m): {len(fwd)} pts')

        if len(fwd) < 50:
            print(f'  [ERROR] Too few {label} points in region.')
            return None

        # For raw point clouds (body_cloud or D455), extract lowest points
        # as ground candidates. Ground in base_footprint should be near z=0.
        if is_raw_cloud or 'D455' in label:
            # Use bottom 20% of z-values as ground candidates
            z_thresh = np.percentile(fwd[:, 2], 20)
            # But also cap at z < 0.3m (ground shouldn't be above 30cm)
            z_cap = min(z_thresh, 0.3)
            low = fwd[fwd[:, 2] < z_cap]
            if len(low) < 30:
                # Relax: take bottom 30%
                z_thresh = np.percentile(fwd[:, 2], 30)
                low = fwd[fwd[:, 2] < z_thresh]
            print(f'  {label} low candidates (z < {z_cap:.2f}m): {len(low)} pts, '
                  f'z=[{low[:,2].min():.3f}, {low[:,2].max():.3f}]')
            target = low
        else:
            # Patchwork++ ground: all points are already ground-classified
            target = fwd

        if len(target) < 10:
            print(f'  [ERROR] Too few ground candidate points.')
            return None

        normal, d, inlier_mask = fit_ground_plane(target)
        if normal is not None:
            inliers = target[inlier_mask]
            gz = inliers[:, 2].mean()
            gz_std = inliers[:, 2].std()
            n = np.sum(inlier_mask)
            print(f'  {label} RANSAC ground: normal=[{normal[0]:+.3f}, {normal[1]:+.3f}, '
                  f'{normal[2]:+.3f}], inliers={n}')
            print(f'  {label} ground z: mean={gz:+.4f}m, std={gz_std:.4f}m')
            return gz, gz_std, n
        else:
            # Fallback: median of lowest points
            gz = np.median(target[:, 2])
            gz_std = target[:, 2].std()
            print(f'  {label} RANSAC failed, using median of low pts: z={gz:+.4f}m')
            return gz, gz_std, len(target)

    def analyze(self):
        print(f'\n{"="*64}')
        print(f'  Sensor Height Measurement (MID360 + D455)')
        print(f'{"="*64}')

        has_ground = len(self.ground_frames) > 0
        has_body = len(self.body_frames) > 0
        has_d455 = len(self.d455_frames) > 0

        if not has_ground and not has_body:
            print('\n  [ERROR] No LiDAR data received!')
            print('  Ensure SLAM system is running.')
            return

        print(f'\n  Data collected:')
        if has_ground:
            print(f'    Patchwork++ ground: {len(self.ground_frames)} frames '
                  f'(frame: {self.ground_frame_id})')
        if has_body:
            print(f'    body_cloud (raw): {len(self.body_frames)} frames '
                  f'(frame: {self.body_frame_id})')
        if has_d455:
            print(f'    D455 depth: {len(self.d455_frames)} frames '
                  f'(frame: {self.d455_frame_id})')
        else:
            print(f'    D455 depth: 0 frames (will skip D455 measurement)')

        # ============================================================
        # MID360 Ground Measurement
        # ============================================================
        print(f'\n{"─"*64}')
        print(f'  1. MID360 Height Verification')
        print(f'{"─"*64}')

        # Try Patchwork++ ground first, fall back to body_cloud
        use_body_cloud = False
        if has_ground:
            Rg, Tg = self.get_tf('base_footprint', self.ground_frame_id)
            if Rg is not None:
                ground_all = np.vstack(self.ground_frames)
                ground_bf = (Rg @ ground_all.T).T + Tg
                # Check if enough forward points
                fwd_mask = (
                    (np.abs(ground_bf[:, 0]) < 3.0) &
                    (ground_bf[:, 1] > -1.0) & (ground_bf[:, 1] < 5.0)
                )
                if np.sum(fwd_mask) >= 50:
                    print(f'  Using Patchwork++ ground ({len(ground_all)} pts)')
                else:
                    print(f'  Patchwork++ has too few points ({np.sum(fwd_mask)} in region), '
                          f'falling back to body_cloud')
                    use_body_cloud = True
            else:
                use_body_cloud = True
        else:
            use_body_cloud = True

        if use_body_cloud and has_body:
            print(f'  Using body_cloud with RANSAC ground detection')
            frame_id = self.body_frame_id
            Rg, Tg = self.get_tf('base_footprint', frame_id)
            if Rg is None:
                print(f'  [ERROR] Cannot get TF: {frame_id} -> base_footprint')
                return
            body_all = np.vstack(self.body_frames)
            ground_bf = (Rg @ body_all.T).T + Tg
            print(f'  Total body_cloud points: {len(body_all)}')
        elif use_body_cloud:
            print(f'  [ERROR] No usable LiDAR ground data.')
            return

        # Widen the search region for ground detection
        mid_result = self.measure_ground_z(ground_bf, 'MID360', x_range=3.0,
                                           y_min=-1.0, y_max=5.0,
                                           is_raw_cloud=use_body_cloud)
        if mid_result is None:
            return
        mid_gz, mid_std, mid_n = mid_result

        current_mid360_z = 0.56
        corrected_mid360_z = current_mid360_z - mid_gz

        print(f'\n  MID360 Result:')
        print(f'    Ground z in base_footprint = {mid_gz:+.4f}m (should be ~0.0)')
        if abs(mid_gz) < 0.02:
            print(f'    [OK] MID360 height is well-calibrated (error < 2cm)')
        else:
            direction = 'HIGHER' if mid_gz > 0 else 'LOWER'
            print(f'    [NEEDS CORRECTION] Ground is {abs(mid_gz)*100:.1f}cm '
                  f'{"above" if mid_gz > 0 else "below"} expected.')
            print(f'    → MID360 is actually {direction} than configured.')
            print(f'    Current mid360_mount_z = {current_mid360_z}')
            print(f'    Corrected mid360_mount_z = {corrected_mid360_z:.4f}')
            print(f'    Also update FAST-LIO2 lio_megarover.yaml:')
            print(f'      t_il: [0.09, 0.0, {corrected_mid360_z:.4f}]  (was 0.56)')

        if mid_std > 0.05:
            print(f'    [WARNING] High ground variance (std={mid_std:.3f}m). '
                  f'Surface may not be flat.')

        # ============================================================
        # D455 Ground Measurement
        # ============================================================
        if not has_d455:
            self._print_summary(mid_gz, mid_std, None, None)
            return

        print(f'\n{"─"*64}')
        print(f'  2. D455 Height Measurement')
        print(f'{"─"*64}')

        Rd, Td = self.get_tf('base_footprint', self.d455_frame_id)
        if Rd is None:
            print(f'  [ERROR] Cannot get TF: {self.d455_frame_id} -> base_footprint')
            self._print_summary(mid_gz, mid_std, None, None)
            return

        d455_raw = np.vstack(self.d455_frames)
        d455_bf = (Rd @ d455_raw.T).T + Td
        print(f'  Total D455 points: {len(d455_bf)}')
        print(f'  Z range in base_footprint: [{d455_bf[:,2].min():.3f}, '
              f'{d455_bf[:,2].max():.3f}]')

        d455_result = self.measure_ground_z(d455_bf, 'D455', x_range=1.5, y_max=2.5)
        if d455_result is None:
            self._print_summary(mid_gz, mid_std, None, None)
            return
        d455_gz, d455_std, d455_n = d455_result

        # D455-specific error (relative to MID360 reference)
        d455_relative_error = d455_gz - mid_gz
        current_d455_z = 0.16
        corrected_d455_z = current_d455_z - d455_relative_error

        print(f'\n  D455 Result:')
        print(f'    D455 ground z in base_footprint = {d455_gz:+.4f}m')
        print(f'    MID360 ground z (reference)      = {mid_gz:+.4f}m')
        print(f'    D455-specific error               = {d455_relative_error:+.4f}m')

        if abs(d455_relative_error) < 0.02:
            print(f'    [OK] D455 height is consistent with MID360 (error < 2cm)')
        else:
            direction = 'HIGHER' if d455_relative_error < 0 else 'LOWER'
            print(f'    → D455 is actually {direction} than configured '
                  f'(by {abs(d455_relative_error)*100:.1f}cm)')
            print(f'    Current d455_front_mount_z = {current_d455_z}')
            print(f'    Corrected d455_front_mount_z = {corrected_d455_z:.4f}')

        # Camera center height (accounting for inverted mount offset)
        implied_center = corrected_d455_z - 0.0145
        print(f'    Implied camera center height: {implied_center*100:.1f}cm from ground')

        self._print_summary(mid_gz, mid_std, d455_relative_error, corrected_d455_z)

    def _print_summary(self, mid_gz, mid_std, d455_error, corrected_d455_z):
        """Print final summary with all corrections."""
        current_mid360_z = 0.56
        corrected_mid360_z = current_mid360_z - mid_gz

        print(f'\n{"="*64}')
        print(f'  SUMMARY - Required Changes')
        print(f'{"="*64}')

        mid_ok = abs(mid_gz) < 0.02
        d455_ok = d455_error is None or abs(d455_error) < 0.02

        # MID360
        if mid_ok:
            print(f'\n  MID360: OK (ground error = {mid_gz:+.4f}m, < 2cm)')
        else:
            print(f'\n  MID360: NEEDS UPDATE (ground error = {mid_gz:+.4f}m)')
            print(f'    File: calibration_offsets.xacro')
            print(f'      mid360_mount_z: {current_mid360_z} -> {corrected_mid360_z:.4f}')
            print(f'    File: lio_megarover.yaml')
            print(f'      t_il: [0.09, 0.0, {current_mid360_z}] -> '
                  f'[0.09, 0.0, {corrected_mid360_z:.4f}]')

        # D455
        if d455_error is None:
            print(f'\n  D455: NOT MEASURED (no depth data received)')
        elif d455_ok:
            print(f'\n  D455: OK (relative error = {d455_error:+.4f}m, < 2cm)')
        else:
            print(f'\n  D455: NEEDS UPDATE (relative error = {d455_error:+.4f}m)')
            print(f'    File: calibration_offsets.xacro')
            print(f'      d455_front_mount_z: 0.16 -> {corrected_d455_z:.4f}')

        if mid_ok and d455_ok:
            print(f'\n  All sensor heights are within tolerance.')
        else:
            print(f'\n  After editing, rebuild URDF and restart:')
            print(f'    colcon build --packages-select megarover_description --symlink-install')
            print(f'    # Then restart the SLAM system and re-run this script to verify.')

        if mid_std > 0.05:
            print(f'\n  [NOTE] High ground variance (std={mid_std:.3f}m). '
                  f'Results may be less accurate.')
            print(f'  Re-run on a flatter surface for better precision.')

        print(f'{"="*64}\n')


def main():
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)

    rclpy.init()
    node = SensorHeightMeasurer()

    collect_time = 8.0
    print(f'Collecting data for {collect_time:.0f} seconds...')
    print('Keep robot stationary on FLAT ground with clear space in front.')
    start = time.time()
    while time.time() - start < collect_time:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.time() - start
        if int(elapsed) != int(elapsed - 0.1) or elapsed < 0.15:
            gn = len(node.ground_frames)
            bn = len(node.body_frames)
            dn = len(node.d455_frames)
            if gn > 0 or bn > 0 or dn > 0:
                print(f'  [{elapsed:.1f}s] Ground: {gn}, Body: {bn}, D455: {dn}')

    print(f'\nCollection done. Ground: {len(node.ground_frames)}, '
          f'Body: {len(node.body_frames)}, D455: {len(node.d455_frames)}')
    print('Analyzing...')
    node.analyze()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
