#!/usr/bin/env python3
"""
D455 Near-field Obstacle Filter

Dedicated node for extracting low-lying obstacles from the D455 depth camera.
Uses a distance-bin ground baseline method for robust ground/obstacle separation
that tolerates minor calibration drift, ground unevenness, and pitch variation.

Input:  /camera/d455_front/depth/color/points  (camera_optical_frame)
Output: /d455_front_obstacles                   (base_footprint)

Designed for local_costmap near-field protection (0.2~1.2m).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros


class D455NearfieldObstacleFilter(Node):
    def __init__(self):
        super().__init__('d455_nearfield_obstacle_filter')

        # --- Parameters -----------------------------------------------------------
        self.declare_parameter('input_topic', '/camera/d455_front/depth/color/points')
        self.declare_parameter('output_topic', '/d455_front_obstacles')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('throttle_factor', 2)

        # Range filter (Euclidean distance in camera frame)
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 1.2)

        # ROI in base_footprint (forward / lateral / height)
        self.declare_parameter('roi_x_min', 0.0)
        self.declare_parameter('roi_x_max', 1.2)
        self.declare_parameter('roi_y_abs_max', 0.45)
        self.declare_parameter('roi_z_max', 0.5)

        # Ground baseline estimation
        self.declare_parameter('ground_bin_size', 0.1)
        self.declare_parameter('ground_percentile', 10)
        self.declare_parameter('ground_margin', 0.03)

        # Obstacle threshold (relative to local ground baseline)
        # Effective threshold = ground_margin + obstacle_min_relative_height
        #   ground band:    rel_h <= ground_margin           (noise / ground)
        #   uncertain band: ground_margin < rel_h <= margin+thresh  (ignored)
        #   obstacle band:  rel_h > margin+thresh            (published)
        self.declare_parameter('obstacle_min_relative_height', 0.02)

        # Cluster / density filter
        self.declare_parameter('min_cluster_points', 8)
        self.declare_parameter('min_cluster_width', 0.06)

        # Output voxel size
        self.declare_parameter('voxel_size', 0.05)

        # LiDAR timestamp source for restamping
        self.declare_parameter('lidar_topic', '/patchworkpp/nonground')

        # Debug
        self.declare_parameter('debug', False)

        # --- Read parameters ------------------------------------------------------
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.throttle_factor = max(1, self.get_parameter('throttle_factor').value)

        self.min_range_sq = self.get_parameter('min_range').value ** 2
        self.max_range_sq = self.get_parameter('max_range').value ** 2

        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_abs_max = self.get_parameter('roi_y_abs_max').value
        self.roi_z_max = self.get_parameter('roi_z_max').value

        self.ground_bin_size = self.get_parameter('ground_bin_size').value
        self.ground_percentile = self.get_parameter('ground_percentile').value
        self.ground_margin = self.get_parameter('ground_margin').value

        self.obstacle_min_rel_height = self.get_parameter('obstacle_min_relative_height').value
        # Effective threshold = ground_margin + obstacle_min_relative_height
        self.obstacle_threshold = self.ground_margin + self.obstacle_min_rel_height

        self.min_cluster_points = self.get_parameter('min_cluster_points').value
        self.min_cluster_width = self.get_parameter('min_cluster_width').value

        self.voxel_size = self.get_parameter('voxel_size').value

        lidar_topic = self.get_parameter('lidar_topic').value
        self.debug = self.get_parameter('debug').value

        # --- State ----------------------------------------------------------------
        self.msg_count = 0
        self.latest_lidar_stamp = None
        self._camera_frame_id = None

        # TF: camera_optical_frame → base_footprint (full rotation + translation)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_valid = False
        self._tf_rot = None    # (3, 3) rotation matrix
        self._tf_trans = None  # (3,)   translation vector
        self.create_timer(1.0, self._lookup_tf)

        # --- QoS ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # --- Pub / Sub ------------------------------------------------------------
        self.pub = self.create_publisher(PointCloud2, output_topic, sensor_qos)

        self.camera_sub = self.create_subscription(
            PointCloud2, input_topic, self._camera_cb, sensor_qos)

        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self._lidar_stamp_cb, sensor_qos)

        self.get_logger().info(
            f'D455 nearfield obstacle filter: {input_topic} -> {output_topic}')
        self.get_logger().info(
            f'  range=[{self.get_parameter("min_range").value:.2f}, '
            f'{self.get_parameter("max_range").value:.2f}]m  '
            f'ROI x=[{self.roi_x_min:.2f},{self.roi_x_max:.2f}] '
            f'|y|<{self.roi_y_abs_max:.2f}  z<{self.roi_z_max:.2f}')
        self.get_logger().info(
            f'  ground: bin={self.ground_bin_size}m pct={self.ground_percentile} '
            f'margin={self.ground_margin}m + rel_h={self.obstacle_min_rel_height}m '
            f'= effective_thresh={self.obstacle_threshold:.3f}m')

    def _publish_cloud(self, points: np.ndarray, stamp, frame_id: str) -> None:
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        self.pub.publish(point_cloud2.create_cloud(header, fields, points))

    def _publish_empty(self, msg: PointCloud2, reason: str) -> None:
        stamp = self.latest_lidar_stamp if self.latest_lidar_stamp else msg.header.stamp
        empty = np.empty((0, 3), dtype=np.float32)
        self._publish_cloud(empty, stamp, self.target_frame)
        if self.debug:
            self.get_logger().info(
                f'nearfield: publish empty ({reason})',
                throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------
    def _lidar_stamp_cb(self, msg: PointCloud2):
        self.latest_lidar_stamp = msg.header.stamp

    def _lookup_tf(self):
        """Cache static TF from camera optical frame to base_footprint."""
        if self._tf_valid or self._camera_frame_id is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self.target_frame, self._camera_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            q = tf.transform.rotation
            t = tf.transform.translation

            # Full 3×3 rotation matrix from quaternion
            r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            r01 = 2.0 * (q.x * q.y - q.w * q.z)
            r02 = 2.0 * (q.x * q.z + q.w * q.y)
            r10 = 2.0 * (q.x * q.y + q.w * q.z)
            r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
            r12 = 2.0 * (q.y * q.z - q.w * q.x)
            r20 = 2.0 * (q.x * q.z - q.w * q.y)
            r21 = 2.0 * (q.y * q.z + q.w * q.x)
            r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

            self._tf_rot = np.array([
                [r00, r01, r02],
                [r10, r11, r12],
                [r20, r21, r22],
            ], dtype=np.float32)
            self._tf_trans = np.array([t.x, t.y, t.z], dtype=np.float32)
            self._tf_valid = True
            self.get_logger().info(
                f'TF ready: {self._camera_frame_id} -> {self.target_frame} '
                f'(t=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}])')
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'TF not available: {e}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Main processing pipeline
    # ------------------------------------------------------------------
    def _camera_cb(self, msg: PointCloud2):
        if self._camera_frame_id is None:
            self._camera_frame_id = msg.header.frame_id

        self.msg_count += 1
        if self.msg_count % self.throttle_factor != 0:
            return
        if not self._tf_valid:
            return
        self._process(msg)

    def _process(self, msg: PointCloud2):
        try:
            # 1. Read xyz --------------------------------------------------------
            points = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'))
            if len(points) == 0:
                self._publish_empty(msg, 'no_points')
                return

            # 2. Remove NaN / Inf ------------------------------------------------
            valid = np.isfinite(points).all(axis=1)
            points = points[valid]
            if len(points) == 0:
                self._publish_empty(msg, 'no_finite_points')
                return

            # 3. Range filter in camera frame ------------------------------------
            dist_sq = np.sum(points ** 2, axis=1)
            mask = (dist_sq >= self.min_range_sq) & (dist_sq <= self.max_range_sq)
            points = points[mask]
            if len(points) == 0:
                self._publish_empty(msg, 'range_filtered_empty')
                return
            raw_after_range = len(points)

            # 4. Transform to base_footprint ------------------------------------
            #    pts_base = R @ pts.T + t
            pts_base = (self._tf_rot @ points.T).T + self._tf_trans

            # 5. ROI crop in base_footprint -------------------------------------
            roi = (
                (pts_base[:, 0] >= self.roi_x_min) &
                (pts_base[:, 0] <= self.roi_x_max) &
                (np.abs(pts_base[:, 1]) <= self.roi_y_abs_max) &
                (pts_base[:, 2] <= self.roi_z_max)
            )
            pts_base = pts_base[roi]
            if len(pts_base) == 0:
                self._publish_empty(msg, 'roi_empty')
                return
            after_roi = len(pts_base)

            # 6. Distance-bin ground baseline ------------------------------------
            x_vals = pts_base[:, 0]
            z_vals = pts_base[:, 2]

            n_bins = max(1, int(np.ceil(
                (self.roi_x_max - self.roi_x_min) / self.ground_bin_size)))
            bin_idx = np.clip(
                ((x_vals - self.roi_x_min) / self.ground_bin_size).astype(np.int32),
                0, n_bins - 1)

            # Per-bin ground baseline (low percentile of z)
            baseline = np.full(n_bins, np.nan, dtype=np.float64)
            for b in range(n_bins):
                bm = (bin_idx == b)
                if bm.sum() >= 3:
                    baseline[b] = np.percentile(z_vals[bm], self.ground_percentile)

            # Fill NaN gaps (forward then backward)
            last = np.nan
            for b in range(n_bins):
                if np.isnan(baseline[b]):
                    baseline[b] = last
                else:
                    last = baseline[b]
            last = np.nan
            for b in range(n_bins - 1, -1, -1):
                if np.isnan(baseline[b]):
                    baseline[b] = last
                else:
                    last = baseline[b]

            if np.all(np.isnan(baseline)):
                self._publish_empty(msg, 'baseline_all_nan')
                return

            # Smooth with 3-tap moving average
            if n_bins >= 3:
                sm = baseline.copy()
                for b in range(1, n_bins - 1):
                    vals = [baseline[b]]
                    if not np.isnan(baseline[b - 1]):
                        vals.append(baseline[b - 1])
                    if not np.isnan(baseline[b + 1]):
                        vals.append(baseline[b + 1])
                    sm[b] = np.mean(vals)
                baseline = sm

            # 7. Relative-height obstacle detection ------------------------------
            #    ground band:    rel_h <= ground_margin             (0.03m)
            #    uncertain band: margin < rel_h <= obstacle_thresh  (0.03~0.05m)
            #    obstacle band:  rel_h > obstacle_thresh            (>0.05m)
            local_ground = baseline[bin_idx]
            rel_h = z_vals - local_ground
            obs_mask = rel_h > self.obstacle_threshold
            pts_obs = pts_base[obs_mask]
            if len(pts_obs) < self.min_cluster_points:
                self._publish_empty(msg, 'below_min_cluster_points_after_obstacle_filter')
                return

            # 8. Voxel downsample ------------------------------------------------
            voxel = self.voxel_size
            keys = np.floor(pts_obs / voxel).astype(np.int32)
            _, uniq_idx = np.unique(keys, axis=0, return_index=True)
            pts_vox = pts_obs[uniq_idx]

            # 9. Density filter (coarse 2-D grid, remove isolated voxels) --------
            ngrid = 0.10  # 10 cm neighbourhood grid
            gk = np.floor(pts_vox[:, :2] / ngrid).astype(np.int32)
            _, inv, cnt = np.unique(gk, axis=0, return_inverse=True,
                                    return_counts=True)
            dense = cnt[inv] >= 2  # at least 2 voxel-points per cell
            pts_out = pts_vox[dense]

            if len(pts_out) < self.min_cluster_points:
                self._publish_empty(msg, 'below_min_cluster_points_after_density_filter')
                return

            # Lateral / longitudinal extent check
            extent = max(
                pts_out[:, 0].max() - pts_out[:, 0].min(),
                pts_out[:, 1].max() - pts_out[:, 1].min(),
            )
            if extent < self.min_cluster_width:
                self._publish_empty(msg, 'below_min_cluster_width')
                return

            # 10. Publish --------------------------------------------------------
            stamp = self.latest_lidar_stamp if self.latest_lidar_stamp else msg.header.stamp
            self._publish_cloud(pts_out.astype(np.float32, copy=False), stamp, self.target_frame)

            if self.debug:
                self.get_logger().info(
                    f'nearfield: range={raw_after_range} roi={after_roi} '
                    f'obs={obs_mask.sum()} vox={len(pts_vox)} '
                    f'out={len(pts_out)}',
                    throttle_duration_sec=2.0)

        except Exception as e:
            self.get_logger().warn(
                f'Processing error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = D455NearfieldObstacleFilter()
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
