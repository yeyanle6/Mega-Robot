#!/usr/bin/env python3
"""
YOLO Obstacle Detector for D455

Uses YOLOv8n (ONNX) on D455 RGB to detect obstacles that geometric methods
miss — transparent/reflective objects, thin structures, people.  Projects
detections to 3-D using aligned depth and publishes a PointCloud2 that the
local costmap consumes as an independent observation source.

Architecture:  independent safety layer (Scheme A).
If this node crashes or lags, the geometric nearfield filter keeps working.

Input:
  /camera/d455_front/color/image_raw              (RGB)
  /camera/d455_front/aligned_depth_to_color/image_raw  (uint16, mm)
  /camera/d455_front/color/camera_info            (intrinsics)

Output:
  /yolo_obstacles   (PointCloud2 in base_footprint)
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros


class YoloObstacleDetector(Node):
    def __init__(self):
        super().__init__('yolo_obstacle_detector')

        # --- Parameters -----------------------------------------------------------
        self.declare_parameter('model_path', '')
        self.declare_parameter('output_topic', '/yolo_obstacles')
        self.declare_parameter('marker_topic', '/yolo_obstacle_markers')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('throttle_interval_sec', 0.25)  # ~4 Hz
        self.declare_parameter('hold_duration_sec', 0.8)

        # Depth estimation
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('max_depth_m', 2.5)
        self.declare_parameter('depth_roi_ratio', 0.5)  # central fraction of bbox for depth
        self.declare_parameter('depth_percentile', 25)   # use 25th pct (closest reliable)

        # Virtual block generation
        self.declare_parameter('block_grid_resolution', 0.05)  # grid spacing (match costmap)
        self.declare_parameter('block_min_thickness', 0.10)    # min depth of block (m)
        self.declare_parameter('block_z_layers', [0.10, 0.20]) # z heights in base_footprint
        self.declare_parameter('block_max_width', 1.5)         # clamp unreasonable sizes
        self.declare_parameter('block_max_thickness', 0.8)     # clamp depth extent

        # LiDAR timestamp
        self.declare_parameter('lidar_topic', '/patchworkpp/nonground')

        # Topic names
        self.declare_parameter('rgb_topic',
                               '/camera/d455_front/color/image_raw')
        self.declare_parameter('depth_topic',
                               '/camera/d455_front/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/camera/d455_front/color/camera_info')

        # COCO class IDs to treat as obstacles (empty = all detections)
        # Default: person, bicycle, car, motorcycle, bus, truck, bench,
        #          cat, dog, backpack, umbrella, handbag, suitcase,
        #          bottle, wine_glass, cup, chair, couch, potted_plant,
        #          bed, dining_table, tv, laptop, suitcase
        self.declare_parameter('obstacle_class_ids', [
            0, 1, 2, 3, 5, 7, 13, 15, 16,
            24, 25, 26, 28, 39, 40, 41,
            56, 57, 58, 59, 60, 62, 63,
        ])

        self.declare_parameter('debug', False)

        # --- Read parameters ------------------------------------------------------
        model_path = self.get_parameter('model_path').value
        output_topic = self.get_parameter('output_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.throttle_sec = self.get_parameter('throttle_interval_sec').value
        self.hold_duration_sec = self.get_parameter('hold_duration_sec').value

        self.min_depth = self.get_parameter('min_depth_m').value
        self.max_depth = self.get_parameter('max_depth_m').value
        self.depth_roi_ratio = self.get_parameter('depth_roi_ratio').value
        self.depth_percentile = self.get_parameter('depth_percentile').value

        self.block_res = self.get_parameter('block_grid_resolution').value
        self.block_min_thick = self.get_parameter('block_min_thickness').value
        self.block_z_layers = self.get_parameter('block_z_layers').value
        self.block_max_w = self.get_parameter('block_max_width').value
        self.block_max_thick = self.get_parameter('block_max_thickness').value

        lidar_topic = self.get_parameter('lidar_topic').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        class_ids = self.get_parameter('obstacle_class_ids').value
        self.obstacle_class_ids = set(int(c) for c in class_ids) if class_ids else None

        self.debug = self.get_parameter('debug').value

        # --- Load YOLO model (ONNX via ultralytics) -------------------------------
        if not model_path:
            # Default: look in package models/ directory
            model_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'models', 'yolov8n.onnx')

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path, task='detect')
            # Warmup
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model(dummy, verbose=False)
            self.get_logger().info('YOLO model loaded and warmed up')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        # --- State ----------------------------------------------------------------
        self.bridge = CvBridge()
        self.latest_depth = None       # cached depth image (uint16, mm)
        self.latest_lidar_stamp = None
        self.camera_fx = None
        self.camera_fy = None
        self.camera_cx = None
        self.camera_cy = None
        self.last_process_time = self.get_clock().now()
        self._last_detection_time = None
        self._last_cloud_points = None
        self._last_markers = None

        # TF: camera_color_optical_frame → base_footprint
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_valid = False
        self._tf_rot = None
        self._tf_trans = None
        self._camera_optical_frame = None
        self.create_timer(1.0, self._lookup_tf)

        # --- QoS ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # --- Pub / Sub ------------------------------------------------------------
        self.pub = self.create_publisher(PointCloud2, output_topic, sensor_qos)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self._rgb_cb, sensor_qos)

        self.depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_cb, sensor_qos)

        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self._info_cb, sensor_qos)

        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self._lidar_stamp_cb, sensor_qos)

        self.get_logger().info(
            f'YOLO obstacle detector: {rgb_topic} -> {output_topic}')
        self.get_logger().info(
            f'  conf={self.conf_thresh}  depth=[{self.min_depth},{self.max_depth}]m'
            f'  throttle={self.throttle_sec}s  hold={self.hold_duration_sec}s')

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------
    def _lidar_stamp_cb(self, msg: PointCloud2):
        self.latest_lidar_stamp = msg.header.stamp

    def _lookup_tf(self):
        if self._tf_valid or self._camera_optical_frame is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self.target_frame, self._camera_optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            q = tf.transform.rotation
            t = tf.transform.translation

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
                f'TF ready: {self._camera_optical_frame} -> {self.target_frame}')
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'TF not available: {e}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def _info_cb(self, msg: CameraInfo):
        if self.camera_fx is None:
            self.camera_fx = msg.k[0]
            self.camera_fy = msg.k[4]
            self.camera_cx = msg.k[2]
            self.camera_cy = msg.k[5]
            self._camera_optical_frame = msg.header.frame_id
            self.get_logger().info(
                f'Camera intrinsics: fx={self.camera_fx:.1f} fy={self.camera_fy:.1f} '
                f'cx={self.camera_cx:.1f} cy={self.camera_cy:.1f} '
                f'frame={self._camera_optical_frame}')

    def _depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception:
            pass

    def _rgb_cb(self, msg: Image):
        # Throttle
        now = self.get_clock().now()
        elapsed = (now - self.last_process_time).nanoseconds / 1e9
        if elapsed < self.throttle_sec:
            return

        # Prerequisites
        if (self.latest_depth is None or self.camera_fx is None
                or not self._tf_valid):
            return

        self.last_process_time = now

        try:
            self._process(msg)
        except Exception as e:
            self.get_logger().warn(
                f'Processing error: {e}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Main pipeline
    # ------------------------------------------------------------------
    def _process(self, rgb_msg: Image):
        # 1. Convert RGB --------------------------------------------------------
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

        # 2. YOLO inference -----------------------------------------------------
        results = self.model(
            cv_image,
            conf=self.conf_thresh,
            verbose=False,
            imgsz=640,
        )

        if not results or len(results[0].boxes) == 0:
            self._publish_with_hold(rgb_msg, reason='no_boxes')
            return

        boxes = results[0].boxes
        all_points = []
        markers = []
        det_count = 0

        depth_img = self.latest_depth  # uint16, mm
        h_img, w_img = depth_img.shape[:2]

        names = results[0].names
        det_info = []  # for debug logging

        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i].item())
            conf = float(boxes.conf[i].item())

            # Class filter
            if self.obstacle_class_ids and cls_id not in self.obstacle_class_ids:
                continue

            # Bbox (xyxy in pixel coords)
            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_img, x2), min(h_img, y2)
            bw, bh = x2 - x1, y2 - y1
            if bw < 5 or bh < 5:
                continue

            # 3. Depth estimation (central region of bbox) ----------------------
            margin_x = int(bw * (1.0 - self.depth_roi_ratio) / 2)
            margin_y = int(bh * (1.0 - self.depth_roi_ratio) / 2)
            roi = depth_img[y1 + margin_y:y2 - margin_y,
                            x1 + margin_x:x2 - margin_x]

            valid_depth = roi[roi > 0].astype(np.float64) / 1000.0  # → meters
            valid_depth = valid_depth[
                (valid_depth >= self.min_depth) & (valid_depth <= self.max_depth)]

            if len(valid_depth) < 10:
                continue

            # Front face = closest reliable depth; back face from depth spread
            depth_front = np.percentile(valid_depth, self.depth_percentile)
            depth_back = np.percentile(valid_depth, 75)
            thickness = max(self.block_min_thick,
                            min(depth_back - depth_front, self.block_max_thick))

            # 4. Compute real-world size from bbox + depth ----------------------
            real_width = min((bw * depth_front) / self.camera_fx,
                             self.block_max_w)

            # Center of bbox in camera optical frame
            cx_px = (x1 + x2) / 2.0
            cy_px = (y1 + y2) / 2.0
            center_cam = np.array([
                (cx_px - self.camera_cx) * depth_front / self.camera_fx,
                (cy_px - self.camera_cy) * depth_front / self.camera_fy,
                depth_front,
            ], dtype=np.float32)

            # Transform center to base_footprint
            center_base = self._tf_rot @ center_cam + self._tf_trans

            # 5. Generate virtual block grid in base_footprint ------------------
            # The block is a filled rectangle at the obstacle's footprint.
            # In base_footprint: x = forward, y = left.
            # "width" maps to the lateral (y) spread of the detection.
            # "thickness" maps to the forward (x) extent.
            res = self.block_res
            nx = max(1, int(np.ceil(thickness / res)))
            ny = max(1, int(np.ceil(real_width / res)))

            xs = np.linspace(center_base[0] - thickness / 2,
                             center_base[0] + thickness / 2, nx)
            ys = np.linspace(center_base[1] - real_width / 2,
                             center_base[1] + real_width / 2, ny)
            gx, gy = np.meshgrid(xs, ys)
            gx, gy = gx.ravel(), gy.ravel()

            # Replicate at each z layer
            for z_val in self.block_z_layers:
                layer = np.column_stack([
                    gx, gy, np.full_like(gx, z_val)
                ]).astype(np.float32)
                all_points.append(layer)

            markers.extend(self._make_markers(
                marker_id=det_count,
                stamp=(self.latest_lidar_stamp
                       if self.latest_lidar_stamp else rgb_msg.header.stamp),
                center_x=float(center_base[0]),
                center_y=float(center_base[1]),
                center_z=float(np.mean(self.block_z_layers)),
                size_x=float(thickness),
                size_y=float(real_width),
                size_z=float(max(self.block_z_layers) - min(self.block_z_layers) + self.block_res),
                label=names.get(cls_id, str(cls_id)),
                conf=conf,
                depth_front=float(depth_front),
            ))

            det_count += 1
            det_info.append(
                f'{names.get(cls_id, cls_id)}({conf:.2f} '
                f'd={depth_front:.2f} w={real_width:.2f} t={thickness:.2f})')

        if det_count == 0 or not all_points:
            self._publish_with_hold(rgb_msg, reason='no_valid_projected_detections')
            return

        # 6. Publish obstacle cloud ---------------------------------------------
        pts_out = np.vstack(all_points).astype(np.float32)
        stamp = (self.latest_lidar_stamp
                 if self.latest_lidar_stamp else rgb_msg.header.stamp)

        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame

        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        self.pub.publish(point_cloud2.create_cloud(header, fields, pts_out))
        self._publish_markers(markers, header.stamp)
        self._last_detection_time = self.get_clock().now()
        self._last_cloud_points = pts_out.copy()
        self._last_markers = list(markers)

        if self.debug:
            self.get_logger().info(
                f'yolo: {det_count} blocks, {len(pts_out)} pts '
                f'[{"; ".join(det_info)}]',
                throttle_duration_sec=2.0)

    def _publish_with_hold(self, msg, reason=''):
        now = self.get_clock().now()
        if (self._last_detection_time is not None
                and self._last_cloud_points is not None
                and self._last_markers is not None):
            held_sec = (now - self._last_detection_time).nanoseconds / 1e9
            if held_sec <= self.hold_duration_sec:
                stamp = (self.latest_lidar_stamp
                         if self.latest_lidar_stamp else msg.header.stamp)
                header = Header()
                header.stamp = stamp
                header.frame_id = self.target_frame
                fields = [
                    PointField(name='x', offset=0,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,
                               datatype=PointField.FLOAT32, count=1),
                ]
                self.pub.publish(point_cloud2.create_cloud(
                    header, fields, self._last_cloud_points))
                self._publish_markers(self._last_markers, header.stamp)
                if self.debug:
                    self.get_logger().info(
                        f'yolo: hold last detection ({held_sec:.2f}s, reason={reason})',
                        throttle_duration_sec=1.0)
                return
        self._publish_empty(msg)

    def _publish_empty(self, msg):
        stamp = (self.latest_lidar_stamp
                 if self.latest_lidar_stamp else msg.header.stamp)
        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame
        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        empty = np.empty((0, 3), dtype=np.float32)
        self.pub.publish(point_cloud2.create_cloud(header, fields, empty))
        self._publish_markers([], header.stamp)

    def _publish_markers(self, markers, stamp):
        msg = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.target_frame
        clear.header.stamp = stamp
        clear.ns = 'yolo_obstacles'
        clear.id = 0
        clear.action = Marker.DELETEALL
        msg.markers.append(clear)
        msg.markers.extend(markers)
        self.marker_pub.publish(msg)

    def _make_markers(self, marker_id, stamp, center_x, center_y, center_z,
                      size_x, size_y, size_z, label, conf, depth_front):
        cube = Marker()
        cube.header.frame_id = self.target_frame
        cube.header.stamp = stamp
        cube.ns = 'yolo_obstacles'
        cube.id = marker_id * 2 + 1
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = center_x
        cube.pose.position.y = center_y
        cube.pose.position.z = center_z
        cube.pose.orientation.w = 1.0
        cube.scale.x = max(size_x, self.block_res * 2.0)
        cube.scale.y = max(size_y, self.block_res * 2.0)
        cube.scale.z = max(size_z, self.block_res * 2.0)
        cube.color.r = 1.0
        cube.color.g = 0.25
        cube.color.b = 0.1
        cube.color.a = 0.60
        cube.lifetime.sec = 1

        text = Marker()
        text.header.frame_id = self.target_frame
        text.header.stamp = stamp
        text.ns = 'yolo_obstacle_labels'
        text.id = marker_id * 2 + 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = center_x
        text.pose.position.y = center_y + max(size_y * 0.5, 0.08)
        text.pose.position.z = center_z + max(size_z * 0.5, 0.12)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.16
        text.color.r = 1.0
        text.color.g = 0.1
        text.color.b = 0.1
        text.color.a = 0.95
        text.lifetime.sec = 1
        text.text = f'{label} {conf:.2f} {depth_front:.2f}m'

        return [cube, text]


def main(args=None):
    rclpy.init(args=args)
    node = YoloObstacleDetector()
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
