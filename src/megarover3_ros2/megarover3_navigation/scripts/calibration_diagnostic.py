#!/usr/bin/env python3
"""
Sensor Calibration Diagnostic Tool

Compares LiDAR (patchworkpp/nonground) and D455 point clouds on a common
flat surface (wall) to measure extrinsic calibration offset.

Usage:
  1. Place a flat wall/board ~1-2m in FRONT of the robot
  2. Keep robot stationary
  3. Run:  python3 calibration_diagnostic.py
  4. Results show xyz offset to correct in calibration_offsets.xacro

Coordinate convention (base_link):
  X = left/right, Y = forward/backward, Z = height
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
import time


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ])


def fit_plane_ransac(points, n_iter=200, threshold=0.02):
    """RANSAC plane fitting. Returns (normal, d) where normal.dot(p) + d = 0."""
    best_inliers = 0
    best_model = None
    n = len(points)
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
    # Refit with all inliers
    normal, d, mask = best_model
    inlier_pts = points[mask]
    centroid = inlier_pts.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_pts - centroid)
    normal = vh[2]
    d = -np.dot(normal, centroid)
    return normal, d, mask


class CalibrationDiagnostic(Node):
    def __init__(self):
        super().__init__('calibration_diagnostic')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # /world_cloud (FASTLIO2) uses RELIABLE QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.lidar_frames = []
        self.d455_frames = []
        self.lidar_frame_id = None
        self.d455_frame_id = None

        self.create_subscription(
            PointCloud2, '/body_cloud', self.lidar_cb, reliable_qos)
        self.create_subscription(
            PointCloud2, '/d455_front_restamped', self.d455_cb, sensor_qos)

    def lidar_cb(self, msg):
        self.lidar_frame_id = msg.header.frame_id
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        # Filter outliers: keep points within 10m of origin
        dist = np.linalg.norm(pts, axis=1)
        pts = pts[dist < 10.0]
        if len(pts) > 0:
            self.lidar_frames.append(pts)

    def d455_cb(self, msg):
        self.d455_frame_id = msg.header.frame_id
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        # Downsample D455 to max 5000 points per frame (avoid memory/CPU issue)
        if len(pts) > 5000:
            idx = np.random.choice(len(pts), 5000, replace=False)
            pts = pts[idx]
        self.d455_frames.append(pts)

    def filter_forward_region(self, pts, y_min=0.3, y_max=3.0, x_range=2.0, z_min=-0.5, z_max=2.0):
        """Filter points to the forward region in base_link frame.
        base_link: X=left/right, Y=forward, Z=up
        """
        mask = (
            (pts[:, 1] > y_min) & (pts[:, 1] < y_max) &  # forward
            (np.abs(pts[:, 0]) < x_range) &                # left-right
            (pts[:, 2] > z_min) & (pts[:, 2] < z_max)     # height
        )
        return pts[mask]

    def filter_wall_points(self, pts, z_min=0.10, z_max=1.5, x_range=1.0, y_min=0.3, y_max=3.0):
        """Filter to keep only points likely on a vertical wall (exclude floor/ceiling).
        Uses strict Z filter to remove floor and tight X to focus on front.
        """
        mask = (
            (pts[:, 2] > z_min) & (pts[:, 2] < z_max) &   # above floor, below ceiling
            (np.abs(pts[:, 0]) < x_range) &                  # narrow left-right
            (pts[:, 1] > y_min) & (pts[:, 1] < y_max)       # forward range
        )
        return pts[mask]

    def get_tf(self, target_frame, source_frame):
        """Look up TF: source_frame -> target_frame."""
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            R = quat_to_rot(q.x, q.y, q.z, q.w)
            T = np.array([t.x, t.y, t.z])
            return R, T
        except Exception as e:
            self.get_logger().error(f'TF lookup {source_frame}->{target_frame} failed: {e}')
            return None, None

    def analyze(self):
        if not self.lidar_frames or not self.d455_frames:
            self.get_logger().error('No data collected!')
            return

        # Transform D455 (optical frame) -> base_link
        Rd, Td = self.get_tf('base_link', self.d455_frame_id)
        if Rd is None:
            return

        # LiDAR: /body_cloud is already in base_link frame
        lidar_all = np.vstack(self.lidar_frames)

        # D455: transform from optical frame to base_link
        d455_raw = np.vstack(self.d455_frames)
        d455_all = (Rd @ d455_raw.T).T + Td

        print(f'\n{"="*60}')
        print(f'  Sensor Calibration Diagnostic')
        print(f'{"="*60}')
        print(f'\n  LiDAR topic: /body_cloud (frame: {self.lidar_frame_id})')
        print(f'  D455  topic: /d455_front_restamped (frame: {self.d455_frame_id})')
        print(f'  TF d455->base_link: t=[{Td[0]:.4f}, {Td[1]:.4f}, {Td[2]:.4f}]')
        print(f'  TF rotation matrix:')
        for row in Rd:
            print(f'    [{row[0]:+.4f}, {row[1]:+.4f}, {row[2]:+.4f}]')
        print(f'  LiDAR frames collected: {len(self.lidar_frames)} ({len(lidar_all)} pts)')
        print(f'  D455  frames collected: {len(self.d455_frames)} ({len(d455_all)} pts)')

        # Debug: show point distribution before filtering
        # D455 raw distribution (optical frame: Z=forward, X=right, Y=down)
        print(f'\n  D455 RAW (optical frame) distribution:')
        print(f'    X(right)=[{d455_raw[:,0].min():.2f}, {d455_raw[:,0].max():.2f}] '
              f'Y(down)=[{d455_raw[:,1].min():.2f}, {d455_raw[:,1].max():.2f}] '
              f'Z(fwd)=[{d455_raw[:,2].min():.2f}, {d455_raw[:,2].max():.2f}]')
        # Sample a few raw points for sanity check
        sample_idx = np.random.choice(len(d455_raw), min(5, len(d455_raw)), replace=False)
        for i in sample_idx:
            raw_p = d455_raw[i]
            tf_p = d455_all[i]
            print(f'    raw({raw_p[0]:+.3f},{raw_p[1]:+.3f},{raw_p[2]:+.3f}) → base({tf_p[0]:+.3f},{tf_p[1]:+.3f},{tf_p[2]:+.3f})')

        print(f'\n  Point distribution (base_link frame):')
        for label, pts in [('LiDAR', lidar_all), ('D455', d455_all)]:
            print(f'    {label}: X=[{pts[:,0].min():.2f}, {pts[:,0].max():.2f}] '
                  f'Y=[{pts[:,1].min():.2f}, {pts[:,1].max():.2f}] '
                  f'Z=[{pts[:,2].min():.2f}, {pts[:,2].max():.2f}]')

        # Filter to forward region
        lidar_fwd = self.filter_forward_region(lidar_all)
        d455_fwd = self.filter_forward_region(d455_all)

        # Cap points for RANSAC performance
        if len(lidar_fwd) > 10000:
            idx = np.random.choice(len(lidar_fwd), 10000, replace=False)
            lidar_fwd = lidar_fwd[idx]
        if len(d455_fwd) > 10000:
            idx = np.random.choice(len(d455_fwd), 10000, replace=False)
            d455_fwd = d455_fwd[idx]

        print(f'\n  Forward region (Y=0.5~3.0m):')
        print(f'    LiDAR points: {len(lidar_fwd)}')
        print(f'    D455  points: {len(d455_fwd)}')

        if len(lidar_fwd) < 20 or len(d455_fwd) < 20:
            print('\n  [ERROR] Not enough points in forward region!')
            print(f'  LiDAR has {len(lidar_fwd)} pts, D455 has {len(d455_fwd)} pts (need >= 20)')
            print('  Make sure obstacles are in front of robot (0.3~5.0m range).')
            return

        # ---- Method 1: Centroid comparison ----
        lidar_centroid = lidar_fwd.mean(axis=0)
        d455_centroid = d455_fwd.mean(axis=0)
        centroid_diff = d455_centroid - lidar_centroid

        print(f'\n--- Method 1: Centroid Comparison ---')
        print(f'  LiDAR centroid (base_link): X={lidar_centroid[0]:+.4f}  Y={lidar_centroid[1]:+.4f}  Z={lidar_centroid[2]:+.4f}')
        print(f'  D455  centroid (base_link): X={d455_centroid[0]:+.4f}  Y={d455_centroid[1]:+.4f}  Z={d455_centroid[2]:+.4f}')
        print(f'  Difference (D455 - LiDAR):  X={centroid_diff[0]:+.4f}  Y={centroid_diff[1]:+.4f}  Z={centroid_diff[2]:+.4f}')

        # ---- Method 2: Plane fitting ----
        print(f'\n--- Method 2: Plane Fitting (RANSAC) ---')

        ln, ld, lmask = fit_plane_ransac(lidar_fwd)
        dn, dd, dmask = fit_plane_ransac(d455_fwd)

        if ln is None or dn is None:
            print('  [ERROR] Plane fitting failed. Need a flatter surface.')
            return

        # Ensure normals point in the same direction
        if np.dot(ln, dn) < 0:
            dn = -dn
            dd = -dd

        # Check if both planes are vertical walls (Y-dominant normal)
        lidar_is_wall = abs(ln[1]) > 0.5
        d455_is_wall = abs(dn[1]) > 0.5
        if not lidar_is_wall or not d455_is_wall:
            print(f'  [WARNING] Sensors see different surfaces!')
            if not lidar_is_wall:
                print(f'    LiDAR: normal Y={ln[1]:+.3f} → NOT a wall (floor/ceiling?)')
            if not d455_is_wall:
                print(f'    D455:  normal Y={dn[1]:+.3f} → NOT a wall (floor/ceiling?)')
            print(f'  Place a VERTICAL WALL 1-2m in front of robot for accurate calibration.')

        # Plane distance from origin along forward (Y) axis
        # For a wall perpendicular to Y axis, the Y-component of the normal
        # tells us how aligned it is. Distance = -d / normal_y (approximately)
        lidar_wall_dist = -ld / ln[1] if abs(ln[1]) > 0.5 else np.nan
        d455_wall_dist = -dd / dn[1] if abs(dn[1]) > 0.5 else np.nan

        lidar_inliers = np.sum(lmask)
        d455_inliers = np.sum(dmask)

        print(f'  LiDAR plane: normal=[{ln[0]:+.3f}, {ln[1]:+.3f}, {ln[2]:+.3f}], '
              f'd={ld:+.4f}, inliers={lidar_inliers}')
        print(f'  D455  plane: normal=[{dn[0]:+.3f}, {dn[1]:+.3f}, {dn[2]:+.3f}], '
              f'd={dd:+.4f}, inliers={d455_inliers}')

        if not np.isnan(lidar_wall_dist) and not np.isnan(d455_wall_dist):
            dist_diff = d455_wall_dist - lidar_wall_dist
            print(f'\n  Wall distance (along Y/forward):')
            print(f'    LiDAR: {lidar_wall_dist:.4f} m')
            print(f'    D455:  {d455_wall_dist:.4f} m')
            print(f'    Difference: {dist_diff:+.4f} m (positive = D455 sees wall further)')

        # Normal angle difference
        angle_diff = np.arccos(np.clip(np.dot(ln, dn), -1, 1))
        print(f'  Normal angle difference: {np.degrees(angle_diff):.2f}°')

        # ---- Method 3: Per-axis distance distribution ----
        print(f'\n--- Method 3: Per-Axis Statistics ---')
        print(f'  {"Axis":<6} {"LiDAR mean":>12} {"D455 mean":>12} {"Diff":>12} {"LiDAR std":>12} {"D455 std":>12}')
        axis_names = ['X(L/R)', 'Y(Fwd)', 'Z(Up)']
        for i, name in enumerate(axis_names):
            lm = lidar_fwd[:, i].mean()
            dm = d455_fwd[:, i].mean()
            ls = lidar_fwd[:, i].std()
            ds = d455_fwd[:, i].std()
            print(f'  {name:<6} {lm:>+12.4f} {dm:>+12.4f} {dm-lm:>+12.4f} {ls:>12.4f} {ds:>12.4f}')

        # ---- Method 4: Wall-only inlier comparison ----
        if lidar_is_wall and d455_is_wall:
            lidar_wall_pts = lidar_fwd[lmask]
            d455_wall_pts = d455_fwd[dmask]
            if len(lidar_wall_pts) >= 10 and len(d455_wall_pts) >= 10:
                lw_centroid = lidar_wall_pts.mean(axis=0)
                dw_centroid = d455_wall_pts.mean(axis=0)
                wall_diff = dw_centroid - lw_centroid
                print(f'\n--- Method 4: Wall Inliers Only (BEST) ---')
                print(f'  LiDAR wall inliers: {len(lidar_wall_pts)}')
                print(f'  D455  wall inliers: {len(d455_wall_pts)}')
                print(f'  LiDAR wall centroid: X={lw_centroid[0]:+.4f}  Y={lw_centroid[1]:+.4f}  Z={lw_centroid[2]:+.4f}')
                print(f'  D455  wall centroid: X={dw_centroid[0]:+.4f}  Y={dw_centroid[1]:+.4f}  Z={dw_centroid[2]:+.4f}')
                print(f'  Difference (D455-LiDAR): X={wall_diff[0]:+.4f}  Y={wall_diff[1]:+.4f}  Z={wall_diff[2]:+.4f}')
                centroid_diff = wall_diff
        else:
            print(f'\n--- Method 4: Wall Inliers Only ---')
            print(f'  [SKIPPED] Both sensors must detect a vertical wall.')

        # ---- Method 5: Y-axis histogram peak comparison ----
        print(f'\n--- Method 5: Y-Distance Histogram Peak ---')
        for label, pts_fwd in [('LiDAR', lidar_fwd), ('D455', d455_fwd)]:
            if len(pts_fwd) < 10:
                print(f'  {label}: not enough points')
                continue
            y_vals = pts_fwd[:, 1]
            # Histogram with 5cm bins
            bins = np.arange(0.3, 3.05, 0.05)
            hist, edges = np.histogram(y_vals, bins=bins)
            peak_idx = np.argmax(hist)
            peak_y = (edges[peak_idx] + edges[peak_idx + 1]) / 2
            peak_count = hist[peak_idx]
            print(f'  {label}: Y peak at {peak_y:.3f}m (count={peak_count}, bin={edges[peak_idx]:.2f}~{edges[peak_idx+1]:.2f}m)')
        # Compute peak difference
        if len(lidar_fwd) >= 10 and len(d455_fwd) >= 10:
            bins = np.arange(0.3, 3.05, 0.05)
            lhist, ledges = np.histogram(lidar_fwd[:, 1], bins=bins)
            dhist, dedges = np.histogram(d455_fwd[:, 1], bins=bins)
            lpeak = (ledges[np.argmax(lhist)] + ledges[np.argmax(lhist) + 1]) / 2
            dpeak = (dedges[np.argmax(dhist)] + dedges[np.argmax(dhist) + 1]) / 2
            print(f'  Y-distance offset (D455 - LiDAR): {dpeak - lpeak:+.4f} m')
            print(f'  (Positive = D455 sees wall further away)')

        # ---- Method 6: Wall-filtered analysis (exclude floor) ----
        print(f'\n--- Method 6: Wall-Filtered (Z=0.10~1.5, |X|<1.0) ---')
        lidar_wall = self.filter_wall_points(lidar_all)
        d455_wall = self.filter_wall_points(d455_all)
        print(f'  LiDAR wall-region pts: {len(lidar_wall)}')
        print(f'  D455  wall-region pts: {len(d455_wall)}')

        if len(lidar_wall) >= 20 and len(d455_wall) >= 20:
            # Cap for performance
            if len(lidar_wall) > 5000:
                lidar_wall = lidar_wall[np.random.choice(len(lidar_wall), 5000, replace=False)]
            if len(d455_wall) > 5000:
                d455_wall = d455_wall[np.random.choice(len(d455_wall), 5000, replace=False)]

            lw_c = lidar_wall.mean(axis=0)
            dw_c = d455_wall.mean(axis=0)
            diff6 = dw_c - lw_c
            print(f'  LiDAR centroid: X={lw_c[0]:+.4f}  Y={lw_c[1]:+.4f}  Z={lw_c[2]:+.4f}')
            print(f'  D455  centroid: X={dw_c[0]:+.4f}  Y={dw_c[1]:+.4f}  Z={dw_c[2]:+.4f}')
            print(f'  Difference:     X={diff6[0]:+.4f}  Y={diff6[1]:+.4f}  Z={diff6[2]:+.4f}')

            # Y histogram on wall-filtered points
            bins = np.arange(0.3, 3.05, 0.05)
            lh, le = np.histogram(lidar_wall[:, 1], bins=bins)
            dh, de = np.histogram(d455_wall[:, 1], bins=bins)
            lpeak = (le[np.argmax(lh)] + le[np.argmax(lh) + 1]) / 2
            dpeak = (de[np.argmax(dh)] + de[np.argmax(dh) + 1]) / 2
            print(f'  LiDAR Y-peak: {lpeak:.3f}m (count={lh.max()})')
            print(f'  D455  Y-peak: {dpeak:.3f}m (count={dh.max()})')
            print(f'  Y-peak offset (D455-LiDAR): {dpeak - lpeak:+.4f} m')

            # RANSAC on wall-filtered points
            ln6, ld6, lm6 = fit_plane_ransac(lidar_wall)
            dn6, dd6, dm6 = fit_plane_ransac(d455_wall)
            if ln6 is not None and dn6 is not None:
                if np.dot(ln6, dn6) < 0:
                    dn6 = -dn6
                    dd6 = -dd6
                print(f'  LiDAR plane: normal=[{ln6[0]:+.3f}, {ln6[1]:+.3f}, {ln6[2]:+.3f}], d={ld6:+.4f}')
                print(f'  D455  plane: normal=[{dn6[0]:+.3f}, {dn6[1]:+.3f}, {dn6[2]:+.3f}], d={dd6:+.4f}')
                angle6 = np.degrees(np.arccos(np.clip(np.dot(ln6, dn6), -1, 1)))
                print(f'  Normal angle diff: {angle6:.2f}°')
                if abs(ln6[1]) > 0.5 and abs(dn6[1]) > 0.5:
                    lwd = -ld6 / ln6[1]
                    dwd = -dd6 / dn6[1]
                    print(f'  ** WALL DISTANCE (Y): LiDAR={lwd:.4f}m, D455={dwd:.4f}m, diff={dwd-lwd:+.4f}m **')
                    # Use this as the best offset estimate for Y
                    centroid_diff = diff6
                    centroid_diff[1] = dwd - lwd
        else:
            print(f'  [INSUFFICIENT DATA] Need >= 20 points each.')
            print(f'  Ensure a wall is visible from ground level up to ~1m height.')

        # ---- Method 7: Z-axis calibration via board top edge ----
        print(f'\n--- Method 7: Z-Calibration (Board Top Edge) ---')
        # Use wall-filtered points from Method 6 region
        lidar_wf = self.filter_wall_points(lidar_all)
        d455_wf = self.filter_wall_points(d455_all)
        # Further filter to wall Y-range using histogram peak
        if len(lidar_wf) >= 20 and len(d455_wf) >= 20:
            bins = np.arange(0.3, 3.05, 0.05)
            lh, le = np.histogram(lidar_wf[:, 1], bins=bins)
            dh, de = np.histogram(d455_wf[:, 1], bins=bins)
            lpeak_y = (le[np.argmax(lh)] + le[np.argmax(lh) + 1]) / 2
            dpeak_y = (de[np.argmax(dh)] + de[np.argmax(dh) + 1]) / 2
            # Filter both to narrow Y band around the wall (±0.15m from peak)
            lidar_on_wall = lidar_wf[np.abs(lidar_wf[:, 1] - lpeak_y) < 0.15]
            d455_on_wall = d455_wf[np.abs(d455_wf[:, 1] - dpeak_y) < 0.15]
            print(f'  LiDAR on-wall pts: {len(lidar_on_wall)} (Y={lpeak_y:.2f}±0.15m)')
            print(f'  D455  on-wall pts: {len(d455_on_wall)} (Y={dpeak_y:.2f}±0.15m)')

            if len(lidar_on_wall) >= 10 and len(d455_on_wall) >= 10:
                # Z range of each sensor on the wall
                lz_min, lz_max = lidar_on_wall[:, 2].min(), lidar_on_wall[:, 2].max()
                dz_min, dz_max = d455_on_wall[:, 2].min(), d455_on_wall[:, 2].max()
                # Use percentiles to be robust to noise
                lz_p5, lz_p95 = np.percentile(lidar_on_wall[:, 2], [5, 95])
                dz_p5, dz_p95 = np.percentile(d455_on_wall[:, 2], [5, 95])
                print(f'  LiDAR Z range: [{lz_min:.3f}, {lz_max:.3f}], P5={lz_p5:.3f}, P95={lz_p95:.3f}')
                print(f'  D455  Z range: [{dz_min:.3f}, {dz_max:.3f}], P5={dz_p5:.3f}, P95={dz_p95:.3f}')

                # Find Z overlap region
                z_overlap_lo = max(lz_p5, dz_p5)
                z_overlap_hi = min(lz_p95, dz_p95)
                print(f'  Z overlap: [{z_overlap_lo:.3f}, {z_overlap_hi:.3f}]')

                if z_overlap_hi > z_overlap_lo + 0.05:
                    # Both sensors see this Z range on the wall — compare Y distances here
                    l_overlap = lidar_on_wall[(lidar_on_wall[:, 2] >= z_overlap_lo) &
                                              (lidar_on_wall[:, 2] <= z_overlap_hi)]
                    d_overlap = d455_on_wall[(d455_on_wall[:, 2] >= z_overlap_lo) &
                                             (d455_on_wall[:, 2] <= z_overlap_hi)]
                    if len(l_overlap) >= 5 and len(d_overlap) >= 5:
                        # Z-histogram in overlap region to compare distributions
                        zbins = np.arange(z_overlap_lo, z_overlap_hi + 0.02, 0.02)
                        lzh, lze = np.histogram(l_overlap[:, 2], bins=zbins)
                        dzh, dze = np.histogram(d_overlap[:, 2], bins=zbins)
                        # Weighted mean Z in overlap
                        lz_mean = l_overlap[:, 2].mean()
                        dz_mean = d_overlap[:, 2].mean()
                        z_diff = dz_mean - lz_mean
                        print(f'  Overlap LiDAR: {len(l_overlap)} pts, Z mean={lz_mean:.4f}')
                        print(f'  Overlap D455:  {len(d_overlap)} pts, Z mean={dz_mean:.4f}')
                        print(f'  ** Z OFFSET (overlap): {z_diff:+.4f}m **')
                    else:
                        print(f'  [INSUFFICIENT] overlap pts: LiDAR={len(l_overlap)}, D455={len(d_overlap)}')

                    # Also compare top edge (board top)
                    top_diff = dz_p95 - lz_p95
                    bot_diff = dz_p5 - lz_p5
                    print(f'  Top edge (P95):    LiDAR={lz_p95:.4f}, D455={dz_p95:.4f}, diff={top_diff:+.4f}m')
                    print(f'  Bottom edge (P5):  LiDAR={lz_p5:.4f}, D455={dz_p5:.4f}, diff={bot_diff:+.4f}m')
                    print(f'  ** Z OFFSET (top edge): {top_diff:+.4f}m **')
                else:
                    print(f'  [NO Z OVERLAP] Sensors see completely different height ranges.')
                    print(f'  Use a taller board (floor to >1m) to create overlap.')
                    top_diff = dz_p95 - lz_p5  # rough estimate
                    print(f'  Rough estimate from nearest edges: LiDAR P5={lz_p5:.3f}, D455 P95={dz_p95:.3f}')
            else:
                print(f'  [INSUFFICIENT DATA] on-wall pts too few')
        else:
            print(f'  [INSUFFICIENT DATA] wall-region pts: LiDAR={len(lidar_wf)}, D455={len(d455_wf)}')

        # ---- Summary and correction suggestion ----
        print(f'\n{"="*60}')
        print(f'  Suggested Correction (calibration_offsets.xacro)')
        print(f'{"="*60}')
        print(f'  Current D455 front mount:')
        print(f'    d455_front_mount_x = 0.0')
        print(f'    d455_front_mount_y = 0.135')
        print(f'    d455_front_mount_z = 0.16')
        print(f'\n  Measured offset (D455 - LiDAR):')
        print(f'    dX (left/right) = {centroid_diff[0]:+.4f} m')
        print(f'    dY (forward)    = {centroid_diff[1]:+.4f} m')
        print(f'    dZ (height)     = {centroid_diff[2]:+.4f} m')
        print(f'\n  To correct, SUBTRACT the offset from current mount values:')
        print(f'    d455_front_mount_x = {0.0 - centroid_diff[0]:.4f}  (was 0.0)')
        print(f'    d455_front_mount_y = {0.135 - centroid_diff[1]:.4f}  (was 0.135)')
        print(f'    d455_front_mount_z = {0.16 - centroid_diff[2]:.4f}  (was 0.16)')
        print(f'\n  NOTE: These are approximate values based on centroid comparison.')
        print(f'  Plane fitting difference (Y-axis): {dist_diff:+.4f} m' if not np.isnan(lidar_wall_dist) and not np.isnan(d455_wall_dist) else '')
        print(f'  Run multiple times for consistency. Adjust and re-verify.')
        print(f'{"="*60}\n')


def main():
    # Force unbuffered output (critical when running through pipes/subprocess)
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)

    rclpy.init()
    node = CalibrationDiagnostic()

    print('Collecting data for 8 seconds... (keep robot still, obstacles in front)')
    start = time.time()
    while time.time() - start < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.time() - start
        if int(elapsed) != int(elapsed - 0.1) or elapsed < 0.15:
            lidar_n = len(node.lidar_frames)
            d455_n = len(node.d455_frames)
            if lidar_n > 0 or d455_n > 0:
                print(f'  [{elapsed:.1f}s] LiDAR frames: {lidar_n}, D455 frames: {d455_n}')

    print(f'Collection done. LiDAR: {len(node.lidar_frames)} frames, D455: {len(node.d455_frames)} frames')
    print('Analyzing...')
    node.analyze()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
