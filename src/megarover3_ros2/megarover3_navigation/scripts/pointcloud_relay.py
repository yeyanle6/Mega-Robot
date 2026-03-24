#!/usr/bin/env python3
"""
Point Cloud Relay Node

Forwards both LiDAR (Patchwork++ nonground) and D455 depth camera point clouds
to a single /merged_cloud topic for OctoMap consumption.

Additionally publishes a restamped D455 cloud on /d455_front_restamped for
Nav2 costmap consumption (NaN cleaning + timestamp replacement only — no voxel
downsampling or range filtering, since costmap has its own filtering).

LiDAR points pass through directly. D455 points are throttled, voxel-downsampled,
and range-filtered to reduce bandwidth. OctoMap uses each message's header.frame_id
to look up the correct TF transform internally, so no coordinate transformation
is needed here.
"""

import numpy as np
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros


class PointCloudRelay(Node):
    def __init__(self):
        super().__init__('pointcloud_relay')

        # Declare parameters
        self.declare_parameter('lidar_topic', '/patchworkpp/nonground')
        self.declare_parameter('camera_topic', '/camera/d455_front/depth/color/points')
        self.declare_parameter('output_topic', '/merged_cloud')
        self.declare_parameter('camera_throttle_factor', 3)  # publish every Nth message (~2fps from ~5fps native)
        self.declare_parameter('camera_max_range', 4.0)  # meters
        self.declare_parameter('camera_min_range', 1.0)  # meters - filter D455 robot body returns for OctoMap
        self.declare_parameter('camera_voxel_size', 0.05)  # meters
        self.declare_parameter('lidar_throttle_factor', 4)  # publish every Nth LiDAR msg (~5Hz from ~20Hz)
        self.declare_parameter('enable_camera', True)
        self.declare_parameter('costmap_output_topic', '/d455_front_restamped')
        self.declare_parameter('costmap_throttle_factor', 2)  # publish every Nth message (~7-8fps)
        self.declare_parameter('costmap_min_height', 0.15)  # meters above ground in base_footprint
        self.declare_parameter('costmap_max_height', 0.7)   # meters - keep D455 focused on low obstacles
        self.declare_parameter('costmap_min_range', 0.2)    # meters - D455 min reliable depth
        self.declare_parameter('costmap_max_range', 1.8)    # meters - beyond this let LiDAR handle
        self.declare_parameter('costmap_sample_step', 3)    # keep every Nth filtered point for costmap
        self.declare_parameter('debug_costmap_stats', False)
        self.declare_parameter('debug_costmap_stats_interval_sec', 2.0)
        self.declare_parameter('debug_costmap_stats_file', '/tmp/pointcloud_relay_costmap_stats.log')

        # Robot self-filter: exclude LiDAR returns from the robot's own body/frame
        # Points within this bounding box (in base_link frame) are removed
        self.declare_parameter('self_filter_enabled', True)
        self.declare_parameter('self_filter_x_min', -0.20)  # meters behind base_link
        self.declare_parameter('self_filter_x_max', 1.05)   # meters in front
        self.declare_parameter('self_filter_y_min', -0.70)   # meters to the right
        self.declare_parameter('self_filter_y_max', 0.20)    # meters to the left
        self.declare_parameter('lidar_filtered_topic', '/nonground_filtered')

        # Read parameters
        lidar_topic = self.get_parameter('lidar_topic').value
        camera_topic = self.get_parameter('camera_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.throttle_factor = self.get_parameter('camera_throttle_factor').value
        self.lidar_throttle_factor = self.get_parameter('lidar_throttle_factor').value
        self.max_range = self.get_parameter('camera_max_range').value
        self.cam_min_range_sq = self.get_parameter('camera_min_range').value ** 2
        self.voxel_size = self.get_parameter('camera_voxel_size').value
        self.enable_camera = self.get_parameter('enable_camera').value
        costmap_output_topic = self.get_parameter('costmap_output_topic').value
        self.costmap_throttle_factor = self.get_parameter('costmap_throttle_factor').value
        self.costmap_min_height = self.get_parameter('costmap_min_height').value
        self.costmap_max_height = self.get_parameter('costmap_max_height').value
        self.costmap_min_range_sq = self.get_parameter('costmap_min_range').value ** 2
        self.costmap_max_range_sq = self.get_parameter('costmap_max_range').value ** 2
        self.costmap_sample_step = max(1, int(self.get_parameter('costmap_sample_step').value))
        self.debug_costmap_stats = bool(self.get_parameter('debug_costmap_stats').value)
        self.debug_costmap_stats_interval_sec = float(
            self.get_parameter('debug_costmap_stats_interval_sec').value
        )
        self.debug_costmap_stats_file = str(
            self.get_parameter('debug_costmap_stats_file').value
        )

        # Self-filter parameters
        self.self_filter_enabled = self.get_parameter('self_filter_enabled').value
        self.sf_x_min = self.get_parameter('self_filter_x_min').value
        self.sf_x_max = self.get_parameter('self_filter_x_max').value
        self.sf_y_min = self.get_parameter('self_filter_y_min').value
        self.sf_y_max = self.get_parameter('self_filter_y_max').value
        lidar_filtered_topic = self.get_parameter('lidar_filtered_topic').value

        # Throttle counters
        self.lidar_msg_count = 0
        self.camera_msg_count = 0
        self.costmap_msg_count = 0

        # Latest LiDAR timestamp -- used to restamp D455 messages so they
        # fall within the FASTLIO2 TF cache (LiDAR clock lags system clock by ~1.5s)
        self.latest_lidar_stamp = None

        # TF for ground filtering: camera optical frame -> base_footprint
        # We only need the 3rd row of the rotation matrix (to compute z in base_footprint)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._costmap_tf_valid = False
        self._tf_r2 = None     # 3rd row of rotation matrix [R20, R21, R22]
        self._tf_tz = 0.0      # z translation
        self._camera_frame_id = None
        self._last_costmap_stats_log_time = None
        self._costmap_stats_fp = None
        self.create_timer(1.0, self._lookup_camera_tf)

        if self.debug_costmap_stats:
            stats_dir = os.path.dirname(self.debug_costmap_stats_file)
            if stats_dir:
                os.makedirs(stats_dir, exist_ok=True)
            self._costmap_stats_fp = open(self.debug_costmap_stats_file, 'a', encoding='utf-8')
            self.get_logger().info(
                f'Costmap stats file: {self.debug_costmap_stats_file}'
            )

        # QoS: Best Effort to match typical sensor QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.pub = self.create_publisher(PointCloud2, output_topic, sensor_qos)
        self.costmap_pub = self.create_publisher(PointCloud2, costmap_output_topic, sensor_qos)
        self.lidar_filtered_pub = self.create_publisher(PointCloud2, lidar_filtered_topic, sensor_qos)

        # LiDAR subscriber -- direct passthrough
        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self.lidar_callback, sensor_qos
        )

        # D455 subscriber -- throttled + filtered
        if self.enable_camera:
            self.camera_sub = self.create_subscription(
                PointCloud2, camera_topic, self.camera_callback, sensor_qos
            )
            self.get_logger().info(
                f'Camera enabled: {camera_topic} (throttle={self.throttle_factor}, '
                f'max_range={self.max_range}m, voxel={self.voxel_size}m)'
            )
            self.get_logger().info(
                f'Costmap output: {costmap_output_topic} (throttle={self.costmap_throttle_factor}, '
                f'sample_step={self.costmap_sample_step})'
            )

        if self.self_filter_enabled:
            self.get_logger().info(
                f'Self-filter enabled: exclude box x=[{self.sf_x_min}, {self.sf_x_max}], '
                f'y=[{self.sf_y_min}, {self.sf_y_max}] in base_link frame'
            )

        self.get_logger().info(
            f'Relay: {lidar_topic} + {camera_topic} -> {output_topic}'
        )
        self.get_logger().info(
            f'Filtered LiDAR: {lidar_filtered_topic}'
        )

    def _apply_self_filter(self, msg: PointCloud2):
        """Remove points within the robot body bounding box. Returns filtered PointCloud2."""
        if not self.self_filter_enabled:
            return msg

        # Parse point cloud (assumes float32 x,y,z,intensity layout from FAST-LIO2)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        x = pts[:, 0]
        y = pts[:, 1]

        # Keep points OUTSIDE the robot body bounding box
        inside = ((x >= self.sf_x_min) & (x <= self.sf_x_max) &
                  (y >= self.sf_y_min) & (y <= self.sf_y_max))
        keep = ~inside

        if keep.all():
            return msg

        filtered_raw = raw[keep]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(keep.sum())
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.width * out.point_step
        out.data = filtered_raw.tobytes()
        out.is_dense = msg.is_dense
        return out

    def lidar_callback(self, msg: PointCloud2):
        """Forward LiDAR cloud with throttling and self-filter."""
        self.latest_lidar_stamp = msg.header.stamp
        self.lidar_msg_count += 1
        if self.lidar_msg_count % self.lidar_throttle_factor == 0:
            filtered = self._apply_self_filter(msg)
            self.pub.publish(filtered)
            self.lidar_filtered_pub.publish(filtered)

    def _lookup_camera_tf(self):
        """Try to look up static TF from camera optical frame to base_footprint (once)."""
        if self._costmap_tf_valid or self._camera_frame_id is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'base_footprint', self._camera_frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            q = tf.transform.rotation
            t = tf.transform.translation
            # Only need 3rd row of rotation matrix for z computation:
            # z_base = R[2,0]*x + R[2,1]*y + R[2,2]*z + tz
            self._tf_r2 = np.array([
                2.0 * (q.x * q.z - q.w * q.y),
                2.0 * (q.y * q.z + q.w * q.x),
                1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            ], dtype=np.float32)
            self._tf_tz = float(t.z)
            self._costmap_tf_valid = True
            self.get_logger().info(
                f'Ground filter TF ready: {self._camera_frame_id} -> base_footprint '
                f'(R_z={self._tf_r2}, tz={self._tf_tz:.3f})')
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'Ground filter TF not yet available ({self._camera_frame_id} -> base_footprint): {e}',
                throttle_duration_sec=5.0)

    def camera_callback(self, msg: PointCloud2):
        """Dispatch D455 cloud to costmap and OctoMap pipelines with independent throttling."""
        if self._camera_frame_id is None:
            self._camera_frame_id = msg.header.frame_id
        self.camera_msg_count += 1
        self.costmap_msg_count += 1

        # Costmap output: lightweight (NaN clean + restamp only)
        if self.costmap_msg_count % self.costmap_throttle_factor == 0:
            self._publish_costmap_cloud(msg)

        # OctoMap output: full processing (NaN clean + range filter + voxel downsample + restamp)
        if self.camera_msg_count % self.throttle_factor == 0:
            self._publish_octomap_cloud(msg)

    def _publish_costmap_cloud(self, msg: PointCloud2):
        """Publish D455 cloud for Nav2 costmap with filtering and downsampling.

        Pipeline: NaN removal -> range clip (0.2~2.5m) -> ground height filter
                  -> voxel downsample (0.05m) -> restamp.
        Ground filter uses cached static TF to compute z in base_footprint frame.
        Voxel downsample converts point density into occupancy geometry to prevent
        costmap flooding when D455 faces dense near-field structures.
        """
        try:
            start_time = self.get_clock().now()
            points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
            raw_count = len(points)

            # Remove NaN/Inf points
            valid = np.isfinite(points).all(axis=1)
            points = points[valid]
            finite_count = len(points)

            if len(points) == 0:
                self._maybe_log_costmap_stats(raw_count, 0, 0, 0, 0, start_time)
                return

            # Range filter: keep only points within [min_range, max_range] from sensor
            dist_sq = np.sum(points ** 2, axis=1)
            range_mask = (dist_sq >= self.costmap_min_range_sq) & (dist_sq <= self.costmap_max_range_sq)
            points = points[range_mask]
            range_count = len(points)

            if len(points) == 0:
                self._maybe_log_costmap_stats(raw_count, finite_count, 0, 0, 0, start_time)
                return

            # Ground height filter: compute z in base_footprint, reject ground and ceiling
            if self._costmap_tf_valid:
                z_base = (self._tf_r2[0] * points[:, 0] +
                          self._tf_r2[1] * points[:, 1] +
                          self._tf_r2[2] * points[:, 2] +
                          self._tf_tz)
                height_mask = (z_base >= self.costmap_min_height) & (z_base <= self.costmap_max_height)
                points = points[height_mask]
            height_count = len(points)

            if len(points) == 0:
                self._maybe_log_costmap_stats(
                    raw_count, finite_count, range_count, 0, 0, start_time
                )
                return

            # Voxel downsample: quantize to 0.05m grid (matches costmap resolution),
            # keep one point per cell. This converts "point density" into "occupancy
            # geometry" — the local costmap only cares whether a cell is occupied,
            # not how many points fall in it.
            voxel = self.voxel_size  # reuse the same 0.05m parameter
            # Use floor so negative coordinates are assigned to stable voxel cells.
            keys = np.floor(points / voxel).astype(np.int32)
            _, unique_idx = np.unique(keys, axis=0, return_index=True)
            points = points[unique_idx]

            if self.costmap_sample_step > 1:
                points = points[::self.costmap_sample_step]
            sampled_count = len(points)

            if len(points) == 0:
                self._maybe_log_costmap_stats(
                    raw_count, finite_count, range_count, height_count, 0, start_time
                )
                return

            header = Header()
            header.stamp = self.latest_lidar_stamp if self.latest_lidar_stamp else msg.header.stamp
            header.frame_id = msg.header.frame_id

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            out_msg = point_cloud2.create_cloud(header, fields, points)
            self.costmap_pub.publish(out_msg)
            self._maybe_log_costmap_stats(
                raw_count, finite_count, range_count, height_count, sampled_count, start_time
            )

        except Exception as e:
            self.get_logger().warn(f'Costmap cloud processing error: {e}', throttle_duration_sec=5.0)

    def _maybe_log_costmap_stats(
        self,
        raw_count: int,
        finite_count: int,
        range_count: int,
        height_count: int,
        output_count: int,
        start_time,
    ) -> None:
        if not self.debug_costmap_stats:
            return

        now = self.get_clock().now()
        if self._last_costmap_stats_log_time is not None:
            if now - self._last_costmap_stats_log_time < Duration(
                seconds=self.debug_costmap_stats_interval_sec
            ):
                return

        elapsed_ms = (now - start_time).nanoseconds / 1e6
        self._last_costmap_stats_log_time = now
        self.get_logger().info(
            'costmap_stats '
            f'raw={raw_count} finite={finite_count} range={range_count} '
            f'height={height_count} output={output_count} proc_ms={elapsed_ms:.1f}'
        )
        if self._costmap_stats_fp is not None:
            stamp = now.to_msg()
            self._costmap_stats_fp.write(
                f'{stamp.sec}.{stamp.nanosec:09d} '
                f'raw={raw_count} finite={finite_count} range={range_count} '
                f'height={height_count} output={output_count} proc_ms={elapsed_ms:.1f}\n'
            )
            self._costmap_stats_fp.flush()

    def _publish_octomap_cloud(self, msg: PointCloud2):
        """Publish D455 cloud for OctoMap: NaN filter + range filter + ground filter + voxel downsample + restamp."""
        try:
            # Read xyz from the point cloud
            points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))

            # Remove NaN/Inf points
            valid = np.isfinite(points).all(axis=1)
            points = points[valid]

            if len(points) == 0:
                return

            # Range filter (Euclidean distance from sensor origin)
            # Min range removes robot body returns seen by D455
            dist_sq = np.sum(points ** 2, axis=1)
            points = points[(dist_sq >= self.cam_min_range_sq) & (dist_sq <= self.max_range ** 2)]

            if len(points) == 0:
                return

            # Ground height filter: remove D455 ground points from OctoMap
            # Uses same cached TF as costmap path
            if self._costmap_tf_valid:
                z_base = (self._tf_r2[0] * points[:, 0] +
                          self._tf_r2[1] * points[:, 1] +
                          self._tf_r2[2] * points[:, 2] +
                          self._tf_tz)
                points = points[z_base >= self.costmap_min_height]

            if len(points) == 0:
                return

            # Voxel downsample: quantize to grid then keep unique cells
            voxel = self.voxel_size
            keys = (points / voxel).astype(np.int32)
            _, unique_idx = np.unique(keys, axis=0, return_index=True)
            points = points[unique_idx]

            # Build output PointCloud2
            # Restamp with latest LiDAR timestamp so the message falls within
            # FASTLIO2's TF cache. LiDAR clock lags system clock by ~1.5s;
            # D455 uses system clock, so its timestamps are "in the future"
            # relative to the TF buffer.
            header = Header()
            header.stamp = self.latest_lidar_stamp if self.latest_lidar_stamp else msg.header.stamp
            header.frame_id = msg.header.frame_id

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            out_msg = point_cloud2.create_cloud(header, fields, points)
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().warn(f'Camera cloud processing error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if getattr(node, '_costmap_stats_fp', None) is not None:
            node._costmap_stats_fp.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
