#!/usr/bin/env python3
"""
D455 Extrinsic Auto-Calibration Tool

Calibrates D455 front camera mounting parameters (z, pitch, y) by comparing
point cloud observations against expected geometry.

Modes:
  ground - Fit ground plane from D455 to calibrate z and pitch
  wall   - Compare D455 vs LiDAR wall distance to calibrate y
  full   - Run ground then wall (complete calibration)
  verify - Check current calibration without modifying

Usage:
  # Place robot on flat ground, clear space 0.5-2.5m ahead:
  python3 calibrate_d455.py --mode ground

  # Place robot facing flat wall 1-2m away:
  python3 calibrate_d455.py --mode wall

  # Full calibration (ground first, then wall):
  python3 calibrate_d455.py --mode full

  # Verify only:
  python3 calibrate_d455.py --mode verify

Requires: SLAM system running (FAST-LIO2 + Patchwork++), D455 streaming.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


# ─── Paths ───────────────────────────────────────────────────────────────────

XACRO_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', 'megarover_description',
    'urdf', 'calibration_offsets.xacro'
)

# Resolve to absolute path
XACRO_PATH = os.path.normpath(XACRO_PATH)


# ─── Utility functions ───────────────────────────────────────────────────────

def quat_to_rot(qx, qy, qz, qw):
    """Quaternion to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ])


def fit_ground_plane(points, n_iter=500, threshold=0.02):
    """RANSAC ground plane fitting. Returns (normal, d, inlier_mask).
    Ground plane expected to be roughly horizontal (normal.z > 0.9).
    Returns None triplet on failure.
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
        # Require nearly horizontal
        if abs(normal[2]) < 0.9:
            continue
        d = -np.dot(normal, p0)
        dists = np.abs(points @ normal + d)
        inliers = np.sum(dists < threshold)
        if inliers > best_inliers:
            best_inliers = inliers
            best_model = (normal, d, dists < threshold)

    if best_model is None:
        return None, None, None

    # Refit with all inliers (SVD)
    normal, d, mask = best_model
    inlier_pts = points[mask]
    centroid = inlier_pts.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_pts - centroid, full_matrices=False)
    normal = vh[2]
    if normal[2] < 0:
        normal = -normal
    d = -np.dot(normal, centroid)
    final_dists = np.abs(points @ normal + d)
    mask = final_dists < threshold
    return normal, d, mask


def fit_plane_ransac(points, n_iter=500, threshold=0.02):
    """General RANSAC plane fitting (no orientation constraint).
    Returns (normal, d, inlier_mask) where normal.dot(p) + d = 0.
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
        d = -np.dot(normal, p0)
        dists = np.abs(points @ normal + d)
        inliers = np.sum(dists < threshold)
        if inliers > best_inliers:
            best_inliers = inliers
            best_model = (normal, d, dists < threshold)

    if best_model is None:
        return None, None, None

    # Refit with all inliers (SVD)
    normal, d, mask = best_model
    inlier_pts = points[mask]
    centroid = inlier_pts.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_pts - centroid, full_matrices=False)
    normal = vh[2]
    d = -np.dot(normal, centroid)
    final_dists = np.abs(points @ normal + d)
    mask = final_dists < threshold
    return normal, d, mask


# ─── XacroManager ────────────────────────────────────────────────────────────

class XacroManager:
    """Read/write calibration_offsets.xacro properties."""

    # Properties we care about
    PROPERTIES = [
        'd455_front_mount_x', 'd455_front_mount_y', 'd455_front_mount_z',
        'd455_front_mount_roll', 'd455_front_mount_pitch', 'd455_front_mount_yaw',
    ]

    def __init__(self, path=None):
        self.path = path or XACRO_PATH
        self.values = {}
        self._raw = ''

    def read(self):
        """Read current values from xacro file."""
        with open(self.path, 'r') as f:
            self._raw = f.read()

        for prop in self.PROPERTIES:
            pattern = rf'<xacro:property\s+name="{re.escape(prop)}"\s+value="([^"]+)"'
            match = re.search(pattern, self._raw)
            if match:
                self.values[prop] = float(match.group(1))
            else:
                print(f'  [WARN] Property "{prop}" not found in xacro')
        return self.values

    def write(self, updates):
        """Write updated values back to xacro file.
        Creates a backup first. updates is a dict of {property_name: new_value}.
        """
        # Backup
        backup = self.path + '.bak'
        shutil.copy2(self.path, backup)
        print(f'  Backup saved: {backup}')

        content = self._raw
        for prop, new_val in updates.items():
            pattern = rf'(<xacro:property\s+name="{re.escape(prop)}"\s+value=")([^"]+)(")'
            content = re.sub(pattern, rf'\g<1>{new_val:.6f}\g<3>', content)

        with open(self.path, 'w') as f:
            f.write(content)
        print(f'  Updated: {self.path}')

    def print_current(self):
        """Print current D455 front mount values."""
        print(f'\n  Current D455 front mount values ({os.path.basename(self.path)}):')
        for prop in self.PROPERTIES:
            val = self.values.get(prop, '???')
            if isinstance(val, float):
                print(f'    {prop} = {val:.6f}')
            else:
                print(f'    {prop} = {val}')


# ─── CalibrationCollector ────────────────────────────────────────────────────

class CalibrationCollector(Node):
    """ROS2 node that collects D455, LiDAR, and ground point clouds."""

    def __init__(self):
        super().__init__('d455_calibrator')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        patchwork_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Storage
        self.d455_frames = []
        self.lidar_frames = []
        self.ground_frames = []
        self.d455_frame_id = None
        self.lidar_frame_id = None
        self.ground_frame_id = None

        # Subscriptions
        self.create_subscription(
            PointCloud2, '/camera/d455_front/depth/color/points',
            self._d455_cb, sensor_qos)
        self.create_subscription(
            PointCloud2, '/body_cloud',
            self._lidar_cb, reliable_qos)
        self.create_subscription(
            PointCloud2, '/patchworkpp/ground',
            self._ground_cb, patchwork_qos)

        self.get_logger().info('Waiting for data...')

    def _d455_cb(self, msg):
        self.d455_frame_id = msg.header.frame_id
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        dist = np.linalg.norm(pts, axis=1)
        pts = pts[(dist > 0.2) & (dist < 3.0)]
        if len(pts) > 5000:
            idx = np.random.choice(len(pts), 5000, replace=False)
            pts = pts[idx]
        if len(pts) > 0:
            self.d455_frames.append(pts)

    def _lidar_cb(self, msg):
        self.lidar_frame_id = msg.header.frame_id
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        dist = np.linalg.norm(pts, axis=1)
        # Filter: skip zero-padding points (0,0,0) and far points
        pts = pts[(dist > 0.1) & (dist < 5.0)]
        if len(pts) > 5000:
            idx = np.random.choice(len(pts), 5000, replace=False)
            pts = pts[idx]
        if len(pts) > 0:
            self.lidar_frames.append(pts)

    def _ground_cb(self, msg):
        self.ground_frame_id = msg.header.frame_id
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) > 0:
            self.ground_frames.append(pts)

    def get_tf(self, target, source):
        """Look up TF: source -> target. Returns (R, T) or (None, None)."""
        try:
            tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            R = quat_to_rot(q.x, q.y, q.z, q.w)
            T = np.array([t.x, t.y, t.z])
            return R, T
        except Exception as e:
            self.get_logger().warn(f'TF {source}->{target}: {e}')
            return None, None

    def get_tf_stamped(self, target, source):
        """Look up TF: source -> target. Returns full TransformStamped or None."""
        try:
            return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    def collect(self, duration=10.0):
        """Collect data for the given duration. Returns counts dict."""
        print(f'\nCollecting data for {duration:.0f} seconds...')
        print('Keep robot stationary.')
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start
            if int(elapsed) != int(elapsed - 0.1) or elapsed < 0.15:
                d = len(self.d455_frames)
                l = len(self.lidar_frames)
                g = len(self.ground_frames)
                if d > 0 or l > 0 or g > 0:
                    print(f'  [{elapsed:.1f}s] D455: {d}, LiDAR: {l}, Ground: {g}')

        counts = {
            'd455': len(self.d455_frames),
            'lidar': len(self.lidar_frames),
            'ground': len(self.ground_frames),
        }
        print(f'\nCollection done: D455={counts["d455"]}, LiDAR={counts["lidar"]}, '
              f'Ground={counts["ground"]}')
        return counts

    def transform_points(self, frames, frame_id, target_frame):
        """Transform collected frames to target_frame. Returns Nx3 array or None."""
        if not frames or frame_id is None:
            return None
        R, T = self.get_tf(target_frame, frame_id)
        if R is None:
            return None
        all_pts = np.vstack(frames)
        return (R @ all_pts.T).T + T


# ─── GroundCalibrator ────────────────────────────────────────────────────────

class GroundCalibrator:
    """Ground plane method: calibrate z and pitch from D455 ground observation."""

    # Tolerances
    Z_TOL = 0.02       # 2cm
    PITCH_TOL = 0.5    # 0.5 degrees
    MIN_INLIERS = 200
    MIN_INLIER_RATIO = 0.4

    def __init__(self, collector, xacro_mgr):
        self.collector = collector
        self.xacro = xacro_mgr

    def run(self):
        """Run ground calibration. Returns dict with corrections or None on failure.

        Works in base_footprint frame where ground should be at z ≈ 0.
        """
        print(f'\n{"="*64}')
        print(f'  Ground Plane Calibration (z + pitch)')
        print(f'{"="*64}')

        # Transform D455 to base_footprint (ground = z ≈ 0)
        d455_bf = self.collector.transform_points(
            self.collector.d455_frames, self.collector.d455_frame_id, 'base_footprint')
        if d455_bf is None or len(d455_bf) < 100:
            print('\n  [ERROR] Not enough D455 data in base_footprint.')
            print('  Ensure D455 is streaming and TF is available.')
            return None

        print(f'\n  D455 points in base_footprint: {len(d455_bf)}')
        print(f'  Z range: [{d455_bf[:,2].min():.3f}, {d455_bf[:,2].max():.3f}]')

        # Filter forward ground region in base_footprint
        # base_footprint: X=forward, Y=left, Z=up  (ROS standard)
        fwd_mask = (
            (d455_bf[:, 0] > 0.3) & (d455_bf[:, 0] < 2.5) &  # X forward 0.3~2.5m
            (np.abs(d455_bf[:, 1]) < 1.5)                      # |Y| < 1.5m
        )
        fwd_pts = d455_bf[fwd_mask]
        print(f'  Forward region (X=0.3~2.5, |Y|<1.5): {len(fwd_pts)} pts')

        if len(fwd_pts) < 100:
            print('  [ERROR] Too few points in forward ground region.')
            print('  Ensure flat ground is visible 0.5-2.5m ahead.')
            return None

        # Ground candidates: take lowest 25% of z values, cap at z < 0.3m
        # (in base_footprint, ground should be near z=0)
        z_thresh = np.percentile(fwd_pts[:, 2], 25)
        z_cap = min(z_thresh, 0.3)
        ground_candidates = fwd_pts[fwd_pts[:, 2] < z_cap]
        if len(ground_candidates) < 50:
            z_thresh = np.percentile(fwd_pts[:, 2], 35)
            z_cap = min(z_thresh, 0.5)
            ground_candidates = fwd_pts[fwd_pts[:, 2] < z_cap]

        print(f'  Ground candidates (z < {z_cap:.2f}): {len(ground_candidates)} pts')
        if len(ground_candidates) > 0:
            print(f'  Candidate Z range: [{ground_candidates[:,2].min():.3f}, '
                  f'{ground_candidates[:,2].max():.3f}]')

        if len(ground_candidates) < 50:
            print('  [ERROR] Too few ground candidate points.')
            return None

        # Downsample for RANSAC performance
        if len(ground_candidates) > 20000:
            idx = np.random.choice(len(ground_candidates), 20000, replace=False)
            ground_candidates = ground_candidates[idx]
            print(f'  Downsampled to {len(ground_candidates)} pts for RANSAC')

        # RANSAC fit horizontal plane
        normal, d, inlier_mask = fit_ground_plane(ground_candidates)
        if normal is None:
            print('  [ERROR] RANSAC ground plane fitting failed.')
            return None

        n_inliers = np.sum(inlier_mask)
        inlier_ratio = n_inliers / len(ground_candidates)
        print(f'\n  RANSAC result:')
        print(f'    Normal: [{normal[0]:+.4f}, {normal[1]:+.4f}, {normal[2]:+.4f}]')
        print(f'    d = {d:+.4f}')
        print(f'    Inliers: {n_inliers} / {len(ground_candidates)} ({inlier_ratio:.1%})')

        if n_inliers < self.MIN_INLIERS:
            print(f'  [WARN] Only {n_inliers} inliers (need {self.MIN_INLIERS}). '
                  f'Result may be unreliable.')
        if inlier_ratio < self.MIN_INLIER_RATIO:
            print(f'  [WARN] Low inlier ratio {inlier_ratio:.1%} '
                  f'(need {self.MIN_INLIER_RATIO:.0%}). Surface may not be flat.')

        inlier_pts = ground_candidates[inlier_mask]
        ground_z_mean = inlier_pts[:, 2].mean()
        ground_z_std = inlier_pts[:, 2].std()

        print(f'  D455 ground z in base_footprint: mean={ground_z_mean:+.4f}m, '
              f'std={ground_z_std:.4f}m (should be ~0.0)')

        # Cross-validate with LiDAR ground if available
        lidar_ground_z = None
        lidar_ground_normal = None

        # Try Patchwork++ ground first
        if self.collector.ground_frames:
            ground_bf = self.collector.transform_points(
                self.collector.ground_frames, self.collector.ground_frame_id,
                'base_footprint')
            if ground_bf is not None and len(ground_bf) > 50:
                gfwd = ground_bf[
                    (ground_bf[:, 0] > -1.0) & (ground_bf[:, 0] < 5.0) &
                    (np.abs(ground_bf[:, 1]) < 2.0)
                ]
                if len(gfwd) > 50:
                    gn, gd, gm = fit_ground_plane(gfwd)
                    if gn is not None:
                        lidar_inliers = gfwd[gm]
                        lidar_ground_z = lidar_inliers[:, 2].mean()
                        lidar_ground_normal = gn
                        print(f'\n  LiDAR ground reference (Patchwork++):')
                        print(f'    Ground z: mean={lidar_ground_z:+.4f}m '
                              f'({np.sum(gm)} inliers)')

        # Fallback: extract ground from body_cloud
        if lidar_ground_z is None and self.collector.lidar_frames:
            body_bf = self.collector.transform_points(
                self.collector.lidar_frames, self.collector.lidar_frame_id,
                'base_footprint')
            if body_bf is not None and len(body_bf) > 100:
                body_fwd = body_bf[
                    (body_bf[:, 0] > -1.0) & (body_bf[:, 0] < 5.0) &
                    (np.abs(body_bf[:, 1]) < 2.0)
                ]
                if len(body_fwd) > 100:
                    bz_thresh = np.percentile(body_fwd[:, 2], 20)
                    body_low = body_fwd[body_fwd[:, 2] < min(bz_thresh, 0.15)]
                    if len(body_low) > 30:
                        bn, bd, bm = fit_ground_plane(body_low)
                        if bn is not None:
                            lidar_ground_z = body_low[bm][:, 2].mean()
                            lidar_ground_normal = bn
                            print(f'\n  LiDAR ground reference (body_cloud RANSAC):')
                            print(f'    Ground z: mean={lidar_ground_z:+.4f}m')

        # Compute z error
        if lidar_ground_z is not None:
            z_error = ground_z_mean - lidar_ground_z
            print(f'\n  Z error (D455 ground - LiDAR ground): {z_error:+.4f}m')
        else:
            # No LiDAR reference — ground should be at z=0 in base_footprint
            z_error = ground_z_mean
            print(f'\n  Z error (D455 ground, no LiDAR ref): {z_error:+.4f}m')

        # Pitch error: ground plane normal tilt in the X direction
        # (base_footprint: X=forward, so forward-tilt = arctan(normal_x / normal_z))
        pitch_error_rad = np.arctan2(normal[0], normal[2])
        pitch_error_deg = np.degrees(pitch_error_rad)

        # If we have lidar ground, compute differential pitch
        if lidar_ground_normal is not None:
            lidar_pitch_rad = np.arctan2(lidar_ground_normal[0], lidar_ground_normal[2])
            diff_pitch_rad = pitch_error_rad - lidar_pitch_rad
            diff_pitch_deg = np.degrees(diff_pitch_rad)
            print(f'  D455 ground pitch: {pitch_error_deg:+.2f}°')
            print(f'  LiDAR ground pitch: {np.degrees(lidar_pitch_rad):+.2f}°')
            print(f'  Differential pitch error: {diff_pitch_deg:+.2f}°')
            pitch_correction = -diff_pitch_rad
            pitch_error_deg = diff_pitch_deg
        else:
            pitch_correction = -pitch_error_rad
            print(f'  Ground pitch error: {pitch_error_deg:+.2f}°')

        # z correction: if D455 ground appears above reference (positive z_error),
        # mount_z is too high → decrease it
        z_correction = -z_error

        result = {
            'z_error': z_error,
            'z_correction': z_correction,
            'pitch_error_deg': pitch_error_deg,
            'pitch_correction_rad': pitch_correction,
            'ground_z_mean': ground_z_mean,
            'ground_z_std': ground_z_std,
            'n_inliers': n_inliers,
            'inlier_ratio': inlier_ratio,
            'normal': normal,
        }

        # Summary
        print(f'\n{"─"*64}')
        print(f'  Ground Calibration Results')
        print(f'{"─"*64}')
        z_ok = abs(z_error) < self.Z_TOL
        pitch_ok = abs(pitch_error_deg) < self.PITCH_TOL
        print(f'  Z error:     {z_error:+.4f}m  {"[OK]" if z_ok else "[NEEDS CORRECTION]"}')
        print(f'  Pitch error: {pitch_error_deg:+.2f}°  {"[OK]" if pitch_ok else "[NEEDS CORRECTION]"}')

        if z_ok and pitch_ok:
            print(f'\n  Ground calibration is within tolerance.')

        return result


# ─── WallCalibrator ──────────────────────────────────────────────────────────

class WallCalibrator:
    """Wall comparison method: calibrate y by comparing D455 vs LiDAR wall distance."""

    Y_TOL = 0.02  # 2cm

    def __init__(self, collector, xacro_mgr):
        self.collector = collector
        self.xacro = xacro_mgr

    def run(self):
        """Run wall calibration. Returns dict with corrections or None on failure."""
        print(f'\n{"="*64}')
        print(f'  Wall Comparison Calibration (y)')
        print(f'{"="*64}')

        # Transform both to base_link
        d455_bl = self.collector.transform_points(
            self.collector.d455_frames, self.collector.d455_frame_id, 'base_link')
        lidar_bl = self.collector.transform_points(
            self.collector.lidar_frames, self.collector.lidar_frame_id, 'base_link')

        if d455_bl is None or len(d455_bl) < 100:
            print('\n  [ERROR] Not enough D455 data.')
            return None
        if lidar_bl is None or len(lidar_bl) < 100:
            print('\n  [ERROR] Not enough LiDAR data.')
            return None

        print(f'\n  D455 in base_link: {len(d455_bl)} pts')
        print(f'  LiDAR in base_link: {len(lidar_bl)} pts')

        # Filter wall region: z=0.1-1.5m, |x|<1m, y=0.3-3m
        def wall_filter(pts):
            mask = (
                (pts[:, 2] > 0.10) & (pts[:, 2] < 1.5) &
                (np.abs(pts[:, 0]) < 1.0) &
                (pts[:, 1] > 0.3) & (pts[:, 1] < 3.0)
            )
            return pts[mask]

        d455_wall = wall_filter(d455_bl)
        lidar_wall = wall_filter(lidar_bl)

        print(f'  D455 wall region: {len(d455_wall)} pts')
        print(f'  LiDAR wall region: {len(lidar_wall)} pts')

        if len(d455_wall) < 50 or len(lidar_wall) < 50:
            print('\n  [ERROR] Not enough wall points.')
            print('  Place robot facing a flat wall, 1-2m distance.')
            return None

        # Cap for performance
        if len(d455_wall) > 10000:
            d455_wall = d455_wall[np.random.choice(len(d455_wall), 10000, replace=False)]
        if len(lidar_wall) > 10000:
            lidar_wall = lidar_wall[np.random.choice(len(lidar_wall), 10000, replace=False)]

        # RANSAC vertical plane fitting
        d455_n, d455_d, d455_mask = fit_plane_ransac(d455_wall)
        lidar_n, lidar_d, lidar_mask = fit_plane_ransac(lidar_wall)

        if d455_n is None or lidar_n is None:
            print('\n  [ERROR] Plane fitting failed on wall points.')
            return None

        # Ensure normals point same direction
        if np.dot(d455_n, lidar_n) < 0:
            d455_n = -d455_n
            d455_d = -d455_d

        d455_is_wall = abs(d455_n[1]) > 0.8
        lidar_is_wall = abs(lidar_n[1]) > 0.8

        print(f'\n  D455 plane: normal=[{d455_n[0]:+.3f}, {d455_n[1]:+.3f}, {d455_n[2]:+.3f}], '
              f'd={d455_d:+.4f}, inliers={np.sum(d455_mask)}')
        print(f'  LiDAR plane: normal=[{lidar_n[0]:+.3f}, {lidar_n[1]:+.3f}, {lidar_n[2]:+.3f}], '
              f'd={lidar_d:+.4f}, inliers={np.sum(lidar_mask)}')

        if not d455_is_wall:
            print(f'  [WARN] D455 plane normal Y={d455_n[1]:+.3f} — may not be a vertical wall')
        if not lidar_is_wall:
            print(f'  [WARN] LiDAR plane normal Y={lidar_n[1]:+.3f} — may not be a vertical wall')

        if not d455_is_wall or not lidar_is_wall:
            print('  Place a VERTICAL WALL directly in front (1-2m) for accurate calibration.')

        # Wall distance along Y (forward) axis
        d455_wall_dist = -d455_d / d455_n[1] if abs(d455_n[1]) > 0.5 else np.nan
        lidar_wall_dist = -lidar_d / lidar_n[1] if abs(lidar_n[1]) > 0.5 else np.nan

        if np.isnan(d455_wall_dist) or np.isnan(lidar_wall_dist):
            print('\n  [ERROR] Cannot compute wall distance (plane not perpendicular to Y).')
            return None

        y_error = d455_wall_dist - lidar_wall_dist
        y_correction = -y_error  # If D455 sees wall further, reduce mount_y

        # Also compute inlier centroid comparison
        d455_inlier_c = d455_wall[d455_mask].mean(axis=0)
        lidar_inlier_c = lidar_wall[lidar_mask].mean(axis=0)
        centroid_diff = d455_inlier_c - lidar_inlier_c

        # Normal angle difference
        angle_diff = np.degrees(np.arccos(np.clip(np.dot(d455_n, lidar_n), -1, 1)))

        print(f'\n  Wall distance (Y-axis):')
        print(f'    LiDAR: {lidar_wall_dist:.4f}m')
        print(f'    D455:  {d455_wall_dist:.4f}m')
        print(f'    Difference: {y_error:+.4f}m (positive = D455 sees wall further)')
        print(f'  Normal angle difference: {angle_diff:.2f}°')
        print(f'  Inlier centroid diff: X={centroid_diff[0]:+.4f}, '
              f'Y={centroid_diff[1]:+.4f}, Z={centroid_diff[2]:+.4f}')

        result = {
            'y_error': y_error,
            'y_correction': y_correction,
            'd455_wall_dist': d455_wall_dist,
            'lidar_wall_dist': lidar_wall_dist,
            'angle_diff': angle_diff,
            'centroid_diff': centroid_diff,
        }

        # Summary
        print(f'\n{"─"*64}')
        print(f'  Wall Calibration Results')
        print(f'{"─"*64}')
        y_ok = abs(y_error) < self.Y_TOL
        print(f'  Y error: {y_error:+.4f}m  {"[OK]" if y_ok else "[NEEDS CORRECTION]"}')
        if y_ok:
            print(f'\n  Wall calibration is within tolerance.')

        return result


# ─── TF Chain Diagnostics ────────────────────────────────────────────────────

def diagnose_tf_chain(collector):
    """Print complete TF chain diagnostics for D455."""
    print(f'\n{"="*64}')
    print(f'  TF Chain Diagnostics')
    print(f'{"="*64}')

    # Key frames in the expected chain
    chain = [
        ('map', 'odom'),
        ('odom', 'lio_base'),
        ('lio_base', 'base_footprint'),
        ('base_footprint', 'base_link'),
        ('base_link', 'd455_front_link'),
        ('d455_front_link', 'd455_front_depth_frame'),
        ('d455_front_depth_frame', 'd455_front_depth_optical_frame'),
    ]

    for parent, child in chain:
        tf_s = collector.get_tf_stamped(parent, child)
        if tf_s is None:
            print(f'\n  {parent} <- {child}: [NOT AVAILABLE]')
            continue
        t = tf_s.transform.translation
        q = tf_s.transform.rotation
        # Convert quaternion to RPY for readability
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        # Extract roll, pitch, yaw from rotation matrix
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])

        print(f'\n  {parent} <- {child}:')
        print(f'    xyz:  [{t.x:+.4f}, {t.y:+.4f}, {t.z:+.4f}]')
        print(f'    quat: [{q.x:+.4f}, {q.y:+.4f}, {q.z:+.4f}, {q.w:+.4f}]')
        print(f'    rpy:  [{np.degrees(roll):+.1f}°, {np.degrees(pitch):+.1f}°, '
              f'{np.degrees(yaw):+.1f}°]')

    # Also show the composed TF from base_link to D455 optical frame
    print(f'\n  Composed TF (base_link -> d455_front_depth_optical_frame):')
    R, T = collector.get_tf('base_link', 'd455_front_depth_optical_frame')
    if R is not None:
        print(f'    translation: [{T[0]:+.4f}, {T[1]:+.4f}, {T[2]:+.4f}]')
        print(f'    rotation matrix:')
        for row in R:
            print(f'      [{row[0]:+.4f}, {row[1]:+.4f}, {row[2]:+.4f}]')
    else:
        print(f'    [NOT AVAILABLE]')

    if collector.d455_frame_id:
        print(f'\n  D455 point cloud frame_id: {collector.d455_frame_id}')
    if collector.lidar_frame_id:
        print(f'  LiDAR body_cloud frame_id: {collector.lidar_frame_id}')


# ─── Main ────────────────────────────────────────────────────────────────────

def apply_corrections(xacro_mgr, ground_result, wall_result, is_verify=False):
    """Display before/after comparison and optionally apply corrections."""
    updates = {}

    print(f'\n{"="*64}')
    print(f'  Correction Summary')
    print(f'{"="*64}')

    current_z = xacro_mgr.values.get('d455_front_mount_z', 0.235)
    current_pitch = xacro_mgr.values.get('d455_front_mount_pitch', 0.0)
    current_y = xacro_mgr.values.get('d455_front_mount_y', 0.135)

    all_ok = True

    if ground_result:
        z_err = ground_result['z_error']
        new_z = current_z + ground_result['z_correction']
        pitch_corr = ground_result['pitch_correction_rad']
        new_pitch = current_pitch + pitch_corr

        z_ok = abs(z_err) < GroundCalibrator.Z_TOL
        pitch_ok = abs(ground_result['pitch_error_deg']) < GroundCalibrator.PITCH_TOL

        print(f'\n  d455_front_mount_z:')
        print(f'    Current:   {current_z:.6f}')
        print(f'    Corrected: {new_z:.6f}  (error was {z_err:+.4f}m)')
        if z_ok:
            print(f'    Status: [OK] within {GroundCalibrator.Z_TOL*100:.0f}cm tolerance')
        else:
            print(f'    Status: [NEEDS UPDATE]')
            updates['d455_front_mount_z'] = new_z
            all_ok = False

        print(f'\n  d455_front_mount_pitch:')
        print(f'    Current:   {current_pitch:.6f} ({np.degrees(current_pitch):+.2f}°)')
        print(f'    Corrected: {new_pitch:.6f} ({np.degrees(new_pitch):+.2f}°)')
        print(f'    Pitch error: {ground_result["pitch_error_deg"]:+.2f}°')
        if pitch_ok:
            print(f'    Status: [OK] within {GroundCalibrator.PITCH_TOL:.1f}° tolerance')
        else:
            print(f'    Status: [NEEDS UPDATE]')
            updates['d455_front_mount_pitch'] = new_pitch
            all_ok = False

    if wall_result:
        y_err = wall_result['y_error']
        new_y = current_y + wall_result['y_correction']
        y_ok = abs(y_err) < WallCalibrator.Y_TOL

        print(f'\n  d455_front_mount_y:')
        print(f'    Current:   {current_y:.6f}')
        print(f'    Corrected: {new_y:.6f}  (error was {y_err:+.4f}m)')
        if y_ok:
            print(f'    Status: [OK] within {WallCalibrator.Y_TOL*100:.0f}cm tolerance')
        else:
            print(f'    Status: [NEEDS UPDATE]')
            updates['d455_front_mount_y'] = new_y
            all_ok = False

    if all_ok:
        print(f'\n  All parameters are within tolerance. No changes needed.')
        return True

    if is_verify:
        print(f'\n  [VERIFY MODE] Not applying changes.')
        return False

    if not updates:
        print(f'\n  No updates to apply.')
        return True

    # Ask user confirmation
    print(f'\n  Changes to apply:')
    for prop, val in updates.items():
        old = xacro_mgr.values.get(prop, '???')
        print(f'    {prop}: {old} -> {val:.6f}')

    try:
        answer = input('\n  Apply these changes? [y/N] ').strip().lower()
    except EOFError:
        answer = 'n'

    if answer != 'y':
        print('  Skipped. No changes applied.')
        return False

    # Write updates
    xacro_mgr.write(updates)

    # Rebuild URDF
    print(f'\n  Rebuilding megarover_description...')
    workspace = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
    result = subprocess.run(
        ['colcon', 'build', '--packages-select', 'megarover_description', '--symlink-install'],
        cwd=workspace,
        capture_output=True, text=True
    )
    if result.returncode == 0:
        print('  Build successful.')
    else:
        print(f'  Build failed: {result.stderr[:500]}')
        return False

    return True


def verify_after_apply(collector, xacro_mgr, mode):
    """Re-collect data and verify the calibration after applying changes."""
    print(f'\n{"="*64}')
    print(f'  Post-Calibration Verification')
    print(f'{"="*64}')
    print(f'  Waiting 3 seconds for TF to update...')
    time.sleep(3.0)

    # Clear old data
    collector.d455_frames.clear()
    collector.lidar_frames.clear()
    collector.ground_frames.clear()

    # Re-collect
    collector.collect(duration=8.0)

    if mode in ('ground', 'full'):
        ground_cal = GroundCalibrator(collector, xacro_mgr)
        g_result = ground_cal.run()
        if g_result:
            z_ok = abs(g_result['z_error']) < GroundCalibrator.Z_TOL
            pitch_ok = abs(g_result['pitch_error_deg']) < GroundCalibrator.PITCH_TOL
            if z_ok and pitch_ok:
                print(f'\n  [VERIFIED] Ground calibration: z and pitch OK')
            else:
                print(f'\n  [WARN] Ground calibration still has errors')

    if mode in ('wall', 'full'):
        wall_cal = WallCalibrator(collector, xacro_mgr)
        w_result = wall_cal.run()
        if w_result:
            y_ok = abs(w_result['y_error']) < WallCalibrator.Y_TOL
            if y_ok:
                print(f'\n  [VERIFIED] Wall calibration: y OK')
            else:
                print(f'\n  [WARN] Wall calibration still has errors')


def main():
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)

    parser = argparse.ArgumentParser(
        description='D455 Extrinsic Auto-Calibration Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--mode', choices=['ground', 'wall', 'full', 'verify'],
                        default='full',
                        help='Calibration mode (default: full)')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Data collection duration in seconds (default: 10)')
    parser.add_argument('--xacro', type=str, default=None,
                        help='Path to calibration_offsets.xacro (auto-detected)')
    args = parser.parse_args()

    # Read xacro
    xacro_mgr = XacroManager(args.xacro)
    try:
        xacro_mgr.read()
    except FileNotFoundError:
        print(f'[ERROR] Xacro file not found: {xacro_mgr.path}')
        print(f'Specify with --xacro /path/to/calibration_offsets.xacro')
        sys.exit(1)

    xacro_mgr.print_current()

    # Init ROS2
    rclpy.init()
    collector = CalibrationCollector()

    # Collect data
    counts = collector.collect(duration=args.duration)

    # Check minimum data
    if counts['d455'] == 0:
        print('\n[ERROR] No D455 data received!')
        print('Ensure D455 is streaming on /camera/d455_front/depth/color/points')
        collector.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # TF diagnostics
    diagnose_tf_chain(collector)

    # Run calibration
    ground_result = None
    wall_result = None
    is_verify = (args.mode == 'verify')

    if args.mode in ('ground', 'full', 'verify'):
        ground_cal = GroundCalibrator(collector, xacro_mgr)
        ground_result = ground_cal.run()

    if args.mode in ('wall', 'full', 'verify'):
        if counts['lidar'] == 0:
            print('\n[WARN] No LiDAR data — wall calibration requires body_cloud.')
        else:
            wall_cal = WallCalibrator(collector, xacro_mgr)
            wall_result = wall_cal.run()

    if ground_result is None and wall_result is None:
        print('\n[ERROR] No calibration results. Check data and sensor topics.')
        collector.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Apply corrections
    applied = apply_corrections(xacro_mgr, ground_result, wall_result, is_verify)

    # Verify if changes were applied
    if applied and not is_verify and (ground_result or wall_result):
        # Re-read updated xacro
        xacro_mgr.read()
        verify_after_apply(collector, xacro_mgr, args.mode)

    print(f'\n{"="*64}')
    print(f'  Done.')
    print(f'{"="*64}\n')

    collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
