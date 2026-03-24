"""
Interactive map widget for navigation test workbench.

Features:
  - Load and display 2D occupancy grid (PGM + YAML)
  - Left-click to add numbered waypoints (unlimited)
  - Right-click on waypoint to remove it
  - Drag waypoints to reposition
  - Real-time overlay of 4 trajectory layers
  - Robot pose indicator
  - World ↔ pixel coordinate transforms
"""

import math
import os
import yaml
from typing import List, Tuple, Optional

from PyQt5.QtWidgets import QWidget, QMenu, QAction
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal, QTimer
from PyQt5.QtGui import (QPainter, QPixmap, QImage, QColor, QPen, QBrush,
                          QFont, QPolygonF, QTransform, QWheelEvent)


class Waypoint:
    def __init__(self, x: float, y: float, yaw: float = 0.0, label: str = ''):
        self.x = x      # world meters
        self.y = y
        self.yaw = yaw
        self.label = label


class MapWidget(QWidget):
    """Interactive 2D map with waypoint editing and trajectory display."""

    # Signals
    waypoints_changed = pyqtSignal()                    # waypoint list modified
    coordinate_clicked = pyqtSignal(float, float)       # world x, y
    waypoint_added = pyqtSignal(int, float, float)      # index, x, y
    pose_estimate_set = pyqtSignal(float, float, float) # x, y, yaw

    # Trajectory layer colors
    COLOR_THEORETICAL = QColor(0, 140, 0, 230)      # dark green
    COLOR_GLOBAL_PLAN = QColor(0, 0, 200, 230)       # dark blue
    COLOR_LOCAL_PLAN = QColor(200, 0, 200, 230)       # magenta
    COLOR_ACTUAL = QColor(220, 0, 0, 240)             # dark red
    COLOR_WAYPOINT = QColor(255, 80, 0)               # orange-red
    COLOR_WAYPOINT_SELECTED = QColor(255, 220, 0)     # yellow
    COLOR_ROBOT = QColor(50, 200, 50)                  # green
    COLOR_WAYPOINT_LINE = QColor(180, 180, 180, 120)   # gray

    WAYPOINT_RADIUS_PX = 12

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 300)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        # Map data
        self._map_pixmap: Optional[QPixmap] = None
        self._map_origin_x = 0.0   # world meters
        self._map_origin_y = 0.0
        self._map_resolution = 0.1  # meters/pixel
        self._map_width_px = 0
        self._map_height_px = 0

        # View transform
        self._zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._panning = False
        self._pan_start = None

        # Waypoints
        self.waypoints: List[Waypoint] = []
        self._dragging_wp_idx = -1
        self._hover_wp_idx = -1

        # Trajectories (lists of (x, y) in world coords)
        self._theoretical_path: List[Tuple[float, float]] = []
        self._global_plan: List[Tuple[float, float]] = []
        self._local_plan: List[Tuple[float, float]] = []
        self._actual_trajectory: List[Tuple[float, float]] = []

        # Robot pose
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._robot_visible = False

        # Interaction mode: 'waypoint' (default) or 'pose_estimate'
        self._mode = 'waypoint'
        # Pose estimate drag state
        self._pose_est_dragging = False
        self._pose_est_start = None   # (world_x, world_y)
        self._pose_est_yaw = 0.0

        # Pose estimate result marker (persists until cleared)
        self._pose_marker = None       # (x, y, yaw) or None
        self._pose_marker_status = 'pending'  # 'pending', 'ok', 'warn'

        # Coordinate label
        self._mouse_world_x = 0.0
        self._mouse_world_y = 0.0

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------
    def load_map(self, yaml_path: str) -> bool:
        """Load a 2D map from YAML + PGM files."""
        try:
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)

            map_dir = os.path.dirname(yaml_path)
            pgm_path = os.path.join(map_dir, meta['image'])

            self._map_resolution = float(meta['resolution'])
            origin = meta['origin']
            self._map_origin_x = float(origin[0])
            self._map_origin_y = float(origin[1])

            # Load PGM as QImage
            img = QImage(pgm_path)
            if img.isNull():
                return False

            self._map_width_px = img.width()
            self._map_height_px = img.height()

            # Convert grayscale: free=white, occupied=black, unknown=gray
            self._map_pixmap = QPixmap.fromImage(img)

            # Auto-fit view
            self._auto_fit()
            self.update()
            return True
        except Exception as e:
            print(f'Map load error: {e}')
            return False

    def _auto_fit(self):
        """Fit the map to the widget."""
        if not self._map_pixmap:
            return
        w_ratio = self.width() / max(1, self._map_pixmap.width())
        h_ratio = self.height() / max(1, self._map_pixmap.height())
        self._zoom = min(w_ratio, h_ratio) * 0.95
        self._pan_x = (self.width() - self._map_pixmap.width() * self._zoom) / 2
        self._pan_y = (self.height() - self._map_pixmap.height() * self._zoom) / 2

    # ------------------------------------------------------------------
    # Coordinate transforms
    # ------------------------------------------------------------------
    def world_to_pixel(self, wx: float, wy: float) -> Tuple[float, float]:
        """World meters → map pixel coordinates."""
        px = (wx - self._map_origin_x) / self._map_resolution
        # Map image Y is flipped (top=max_y in world)
        py = self._map_height_px - (wy - self._map_origin_y) / self._map_resolution
        return px, py

    def pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """Map pixel → world meters."""
        wx = px * self._map_resolution + self._map_origin_x
        wy = (self._map_height_px - py) * self._map_resolution + self._map_origin_y
        return wx, wy

    def world_to_screen(self, wx: float, wy: float) -> Tuple[float, float]:
        """World meters → widget screen coordinates."""
        px, py = self.world_to_pixel(wx, wy)
        sx = px * self._zoom + self._pan_x
        sy = py * self._zoom + self._pan_y
        return sx, sy

    def screen_to_world(self, sx: float, sy: float) -> Tuple[float, float]:
        """Widget screen coordinates → world meters."""
        px = (sx - self._pan_x) / self._zoom
        py = (sy - self._pan_y) / self._zoom
        return self.pixel_to_world(px, py)

    # ------------------------------------------------------------------
    # Waypoint management
    # ------------------------------------------------------------------
    def add_waypoint(self, wx: float, wy: float, yaw: float = 0.0):
        idx = len(self.waypoints) + 1
        wp = Waypoint(wx, wy, yaw, label=str(idx))
        self.waypoints.append(wp)
        self.waypoints_changed.emit()
        self.waypoint_added.emit(len(self.waypoints) - 1, wx, wy)
        self.update()

    def remove_waypoint(self, index: int):
        if 0 <= index < len(self.waypoints):
            self.waypoints.pop(index)
            # Re-label
            for i, wp in enumerate(self.waypoints):
                wp.label = str(i + 1)
            self.waypoints_changed.emit()
            self.update()

    def clear_waypoints(self):
        self.waypoints.clear()
        self.waypoints_changed.emit()
        self.update()

    def get_waypoint_at_screen(self, sx: float, sy: float) -> int:
        """Return waypoint index at screen pos, or -1."""
        r = self.WAYPOINT_RADIUS_PX + 4
        for i, wp in enumerate(self.waypoints):
            wx_s, wy_s = self.world_to_screen(wp.x, wp.y)
            if (sx - wx_s) ** 2 + (sy - wy_s) ** 2 <= r * r:
                return i
        return -1

    # ------------------------------------------------------------------
    # Trajectory data setters
    # ------------------------------------------------------------------
    def set_theoretical_path(self, pts: List[Tuple[float, float]]):
        self._theoretical_path = list(pts)
        self.update()

    def set_global_plan(self, pts: List[Tuple[float, float]]):
        self._global_plan = list(pts)
        self.update()

    def set_local_plan(self, pts: List[Tuple[float, float]]):
        self._local_plan = list(pts)
        self.update()

    def append_actual_pose(self, wx: float, wy: float):
        self._actual_trajectory.append((wx, wy))
        self.update()

    def set_actual_trajectory(self, pts: List[Tuple[float, float]]):
        self._actual_trajectory = list(pts)
        self.update()

    def clear_trajectories(self):
        self._theoretical_path.clear()
        self._global_plan.clear()
        self._local_plan.clear()
        self._actual_trajectory.clear()
        self.update()

    def set_robot_pose(self, x: float, y: float, yaw: float):
        self._robot_x = x
        self._robot_y = y
        self._robot_yaw = yaw
        self._robot_visible = True
        self.update()

    def center_on_world(self, wx: float, wy: float):
        """Center the current view on a world-coordinate point."""
        sx, sy = self.world_to_screen(wx, wy)
        self._pan_x += self.width() * 0.5 - sx
        self._pan_y += self.height() * 0.5 - sy
        self.update()

    def center_on_robot(self) -> bool:
        """Center the view on the robot if a pose is available."""
        if not self._robot_visible:
            return False
        self.center_on_world(self._robot_x, self._robot_y)
        return True

    def set_mode(self, mode: str):
        """Switch interaction mode: 'waypoint' or 'pose_estimate'."""
        self._mode = mode
        self._pose_est_dragging = False
        self._pose_est_start = None
        if mode == 'pose_estimate':
            self.setCursor(Qt.CrossCursor)
        else:
            self.setCursor(Qt.ArrowCursor)

    def set_pose_estimate_marker(self, x: float, y: float, yaw: float,
                                  status: str = 'pending'):
        """Show a pose estimate marker on the map. status: pending/ok/warn."""
        self._pose_marker = (x, y, yaw)
        self._pose_marker_status = status
        self.update()

    def clear_pose_estimate_marker(self):
        self._pose_marker = None
        self.update()

    # ------------------------------------------------------------------
    # Painting
    # ------------------------------------------------------------------
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor(30, 30, 30))

        if self._map_pixmap:
            # Draw map
            painter.save()
            painter.translate(self._pan_x, self._pan_y)
            painter.scale(self._zoom, self._zoom)
            painter.drawPixmap(0, 0, self._map_pixmap)
            painter.restore()

        # Trajectory layers
        self._draw_path(painter, self._theoretical_path, self.COLOR_THEORETICAL, 3.0)
        self._draw_path(painter, self._global_plan, self.COLOR_GLOBAL_PLAN, 2.5)
        self._draw_path(painter, self._local_plan, self.COLOR_LOCAL_PLAN, 2.0)
        self._draw_path(painter, self._actual_trajectory, self.COLOR_ACTUAL, 2.5)

        # Waypoint connecting lines
        if len(self.waypoints) >= 2:
            pen = QPen(self.COLOR_WAYPOINT_LINE, 1.5, Qt.DashLine)
            painter.setPen(pen)
            for i in range(len(self.waypoints) - 1):
                s1 = self.world_to_screen(self.waypoints[i].x, self.waypoints[i].y)
                s2 = self.world_to_screen(self.waypoints[i + 1].x, self.waypoints[i + 1].y)
                painter.drawLine(QPointF(*s1), QPointF(*s2))

        # Waypoints
        for i, wp in enumerate(self.waypoints):
            self._draw_waypoint(painter, wp, i)

        # Robot
        if self._robot_visible:
            self._draw_robot(painter)

        # Pose estimate preview arrow (while dragging)
        if self._pose_est_dragging and self._pose_est_start:
            self._draw_pose_estimate_arrow(painter)

        # Pose estimate result marker
        if self._pose_marker:
            self._draw_pose_marker(painter)

        # Coordinate info (bottom-left)
        painter.setPen(QColor(200, 200, 200))
        painter.setFont(QFont('Monospace', 9))
        painter.drawText(10, self.height() - 10,
                         f'({self._mouse_world_x:.2f}, {self._mouse_world_y:.2f})')

        # Legend (top-right)
        self._draw_legend(painter)

        painter.end()

    def _draw_path(self, painter: QPainter,
                   pts: List[Tuple[float, float]],
                   color: QColor, width: float):
        if len(pts) < 2:
            return
        pen = QPen(color, width)
        pen.setCosmetic(True)
        painter.setPen(pen)
        for i in range(len(pts) - 1):
            s1 = self.world_to_screen(pts[i][0], pts[i][1])
            s2 = self.world_to_screen(pts[i + 1][0], pts[i + 1][1])
            painter.drawLine(QPointF(*s1), QPointF(*s2))

    def _draw_waypoint(self, painter: QPainter, wp: Waypoint, index: int):
        sx, sy = self.world_to_screen(wp.x, wp.y)
        r = self.WAYPOINT_RADIUS_PX

        is_hover = (index == self._hover_wp_idx)
        color = self.COLOR_WAYPOINT_SELECTED if is_hover else self.COLOR_WAYPOINT

        # Circle
        painter.setPen(QPen(Qt.white, 2))
        painter.setBrush(QBrush(color))
        painter.drawEllipse(QPointF(sx, sy), r, r)

        # Label number
        painter.setPen(Qt.white)
        painter.setFont(QFont('Arial', 10, QFont.Bold))
        text_rect = QRectF(sx - r, sy - r, r * 2, r * 2)
        painter.drawText(text_rect, Qt.AlignCenter, wp.label)

        # Coordinate below
        painter.setFont(QFont('Monospace', 7))
        painter.setPen(QColor(220, 220, 220))
        painter.drawText(QPointF(sx - 30, sy + r + 12),
                         f'({wp.x:.2f},{wp.y:.2f})')

    def _draw_robot(self, painter: QPainter):
        sx, sy = self.world_to_screen(self._robot_x, self._robot_y)
        size = 10

        painter.save()
        painter.translate(sx, sy)
        # Map yaw to screen rotation: in world +yaw=CCW, screen Y is flipped
        painter.rotate(-math.degrees(self._robot_yaw))

        # Triangle pointing forward (+X in world)
        tri = QPolygonF([
            QPointF(size * 1.5, 0),
            QPointF(-size, -size),
            QPointF(-size, size),
        ])
        painter.setPen(QPen(Qt.white, 1.5))
        painter.setBrush(QBrush(self.COLOR_ROBOT))
        painter.drawPolygon(tri)
        painter.restore()

    def _draw_pose_estimate_arrow(self, painter: QPainter):
        """Draw a green arrow showing the pose estimate being set."""
        wx, wy = self._pose_est_start
        sx, sy = self.world_to_screen(wx, wy)
        yaw = self._pose_est_yaw
        length = 40  # pixels

        painter.save()
        painter.translate(sx, sy)
        painter.rotate(-math.degrees(yaw))

        pen = QPen(QColor(0, 255, 100), 3)
        pen.setCosmetic(True)
        painter.setPen(pen)

        # Shaft
        painter.drawLine(QPointF(0, 0), QPointF(length, 0))
        # Arrowhead
        painter.drawLine(QPointF(length, 0), QPointF(length - 10, -6))
        painter.drawLine(QPointF(length, 0), QPointF(length - 10, 6))

        # Circle at base
        painter.setBrush(QBrush(QColor(0, 255, 100, 80)))
        painter.drawEllipse(QPointF(0, 0), 8, 8)

        painter.restore()

    def _draw_pose_marker(self, painter: QPainter):
        """Draw the pose estimate result marker with status color."""
        wx, wy, yaw = self._pose_marker
        sx, sy = self.world_to_screen(wx, wy)

        colors = {
            'pending': QColor(255, 200, 0),     # yellow
            'ok':      QColor(0, 220, 100),      # green
            'warn':    QColor(255, 80, 0),        # orange
        }
        color = colors.get(self._pose_marker_status, colors['pending'])

        painter.save()
        painter.translate(sx, sy)
        painter.rotate(-math.degrees(yaw))

        pen = QPen(color, 2.5)
        pen.setCosmetic(True)
        painter.setPen(pen)

        # Arrow shaft
        length = 35
        painter.drawLine(QPointF(0, 0), QPointF(length, 0))
        # Arrowhead
        painter.drawLine(QPointF(length, 0), QPointF(length - 8, -5))
        painter.drawLine(QPointF(length, 0), QPointF(length - 8, 5))

        # Circle at base
        painter.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 60)))
        painter.drawEllipse(QPointF(0, 0), 10, 10)

        painter.restore()

        # Status label
        label = {'pending': 'Verifying...', 'ok': 'OK', 'warn': 'Offset!'}
        painter.setPen(color)
        painter.setFont(QFont('Arial', 8, QFont.Bold))
        painter.drawText(QPointF(sx + 14, sy - 14),
                         label.get(self._pose_marker_status, ''))

    def _draw_legend(self, painter: QPainter):
        x0 = self.width() - 170
        y0 = 10
        line_h = 18
        items = [
            (self.COLOR_THEORETICAL, 'Theoretical Path'),
            (self.COLOR_GLOBAL_PLAN, 'Global Plan'),
            (self.COLOR_LOCAL_PLAN, 'Local Plan'),
            (self.COLOR_ACTUAL, 'Actual Trajectory'),
        ]
        # Semi-transparent background so legend is readable on any map
        bg = QRectF(x0 - 6, y0 - 4, 172, len(items) * line_h + 8)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 140))
        painter.drawRoundedRect(bg, 4, 4)

        painter.setFont(QFont('Arial', 8, QFont.Bold))
        for i, (color, label) in enumerate(items):
            y = y0 + i * line_h
            painter.setPen(QPen(color, 3))
            painter.drawLine(x0, y + 6, x0 + 20, y + 6)
            painter.setPen(color)
            painter.drawText(x0 + 25, y + 10, label)

    # ------------------------------------------------------------------
    # Mouse events
    # ------------------------------------------------------------------
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self._mode == 'pose_estimate':
                # Start pose estimate drag
                wx, wy = self.screen_to_world(event.x(), event.y())
                self._pose_est_start = (wx, wy)
                self._pose_est_yaw = 0.0
                self._pose_est_dragging = True
                self.update()
                return

            wp_idx = self.get_waypoint_at_screen(event.x(), event.y())
            if wp_idx >= 0:
                # Start dragging existing waypoint
                self._dragging_wp_idx = wp_idx
            else:
                # Add new waypoint
                wx, wy = self.screen_to_world(event.x(), event.y())
                self.add_waypoint(wx, wy)
                self.coordinate_clicked.emit(wx, wy)

        elif event.button() == Qt.RightButton:
            wp_idx = self.get_waypoint_at_screen(event.x(), event.y())
            if wp_idx >= 0:
                # Context menu
                menu = QMenu(self)
                remove_action = menu.addAction(f'Remove waypoint {wp_idx + 1}')
                action = menu.exec_(event.globalPos())
                if action == remove_action:
                    self.remove_waypoint(wp_idx)
            else:
                # Start panning
                self._panning = True
                self._pan_start = (event.x(), event.y())

    def mouseMoveEvent(self, event):
        # Update coordinate display
        wx, wy = self.screen_to_world(event.x(), event.y())
        self._mouse_world_x = wx
        self._mouse_world_y = wy

        if self._pose_est_dragging and self._pose_est_start:
            # Update yaw from drag direction
            sx0, sy0 = self.world_to_screen(*self._pose_est_start)
            dx = event.x() - sx0
            dy = event.y() - sy0
            if abs(dx) > 3 or abs(dy) > 3:
                # Screen Y is flipped relative to world Y
                self._pose_est_yaw = math.atan2(-dy, dx)
            self.update()
            return

        if self._dragging_wp_idx >= 0:
            # Drag waypoint
            wp = self.waypoints[self._dragging_wp_idx]
            wp.x, wp.y = wx, wy
            self.waypoints_changed.emit()
            self.update()
        elif self._panning and self._pan_start:
            dx = event.x() - self._pan_start[0]
            dy = event.y() - self._pan_start[1]
            self._pan_x += dx
            self._pan_y += dy
            self._pan_start = (event.x(), event.y())
            self.update()
        else:
            # Hover highlight
            old_hover = self._hover_wp_idx
            self._hover_wp_idx = self.get_waypoint_at_screen(event.x(), event.y())
            if old_hover != self._hover_wp_idx:
                self.update()

        self.update()  # for coordinate label

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self._pose_est_dragging and self._pose_est_start:
                # Emit pose estimate and reset mode
                wx, wy = self._pose_est_start
                self.pose_estimate_set.emit(wx, wy, self._pose_est_yaw)
                self._pose_est_dragging = False
                self._pose_est_start = None
                # Auto-switch back to waypoint mode
                self.set_mode('waypoint')
                self.update()
                return
            self._dragging_wp_idx = -1
        elif event.button() == Qt.RightButton:
            self._panning = False
            self._pan_start = None

    def wheelEvent(self, event: QWheelEvent):
        # Zoom toward mouse cursor
        old_wx, old_wy = self.screen_to_world(event.x(), event.y())
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self._zoom = max(0.1, min(50.0, self._zoom * factor))
        # Adjust pan so the world point under cursor stays fixed
        new_sx, new_sy = self.world_to_screen(old_wx, old_wy)
        self._pan_x += event.x() - new_sx
        self._pan_y += event.y() - new_sy
        self.update()

    def resizeEvent(self, event):
        if self._map_pixmap:
            self._auto_fit()
        super().resizeEvent(event)
