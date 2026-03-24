from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QWidget

from map_repair_qt.document import BRUSH_COLORS, BRUSH_VALUES

TOOL_BRUSH = "brush"
TOOL_RECT = "rect"
TOOL_LINE = "line"
TOOL_ERASER = "eraser"


def bresenham(x0, y0, x1, y1):
    """Return list of (x, y) cells along a line."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells


class MapCanvas(QWidget):
    hover_changed = pyqtSignal(int, int, int)
    tool_changed = pyqtSignal(str)

    def __init__(self, document, parent=None):
        super().__init__(parent)
        self.document = document
        self.document.changed.connect(self.update)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self.zoom = 12.0
        self.offset = QPointF(40.0, 40.0)
        self.show_grid = True
        self.brush_mode = "occupied"
        self.brush_size = 1
        self.tool = TOOL_BRUSH

        # Interaction state
        self.painting = False
        self.panning = False
        self.last_pan_pos = QPoint()
        self.current_cell = None
        self._stroke_cache = {}

        # Rectangle / Line drag state
        self._shape_start = None
        self._shape_end = None
        self._shape_dragging = False

    # ---- public setters ----

    def set_tool(self, tool):
        self.tool = tool
        self.tool_changed.emit(tool)
        self.update()

    def set_brush_mode(self, mode):
        self.brush_mode = mode
        self.update()

    def set_brush_size(self, size):
        self.brush_size = max(1, int(size))
        self.update()

    def set_show_grid(self, enabled):
        self.show_grid = bool(enabled)
        self.update()

    def reset_view(self):
        self.zoom = 12.0
        self.offset = QPointF(40.0, 40.0)
        self.update()

    def fit_to_window(self):
        if not self.document.has_map():
            return
        w = self.document.width
        h = self.document.height
        if w <= 0 or h <= 0:
            return
        pad = 40.0
        zx = max(1.0, (self.width() - pad * 2) / w)
        zy = max(1.0, (self.height() - pad * 2) / h)
        self.zoom = max(1.0, min(zx, zy))
        dw = w * self.zoom
        dh = h * self.zoom
        self.offset = QPointF((self.width() - dw) * 0.5,
                              (self.height() - dh) * 0.5)
        self.update()

    # ---- painting ----

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(34, 37, 43))

        if not self.document.has_map():
            painter.setPen(QColor(180, 180, 180))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             "\u8bf7\u6253\u5f00 ROS \u5730\u56fe YAML \u6587\u4ef6\u5f00\u59cb\u7f16\u8f91")
            return

        target = QRectF(self.offset.x(), self.offset.y(),
                        self.document.width * self.zoom,
                        self.document.height * self.zoom)
        painter.drawImage(target, self.document.image)

        if self.show_grid and self.zoom >= 6.0:
            self._draw_grid(painter)

        if self._shape_dragging and self._shape_start and self._shape_end:
            self._draw_shape_preview(painter)
        elif self.current_cell is not None:
            self._draw_cursor(painter)

    def _draw_grid(self, painter):
        pen = QPen(QColor(70, 120, 170, 100))
        pen.setWidth(1)
        painter.setPen(pen)

        ox = self.offset.x()
        oy = self.offset.y()
        z = self.zoom
        vw = self.width()
        vh = self.height()

        col0 = max(0, int((0 - ox) / z))
        col1 = min(self.document.width, int((vw - ox) / z) + 1)
        row0 = max(0, int((0 - oy) / z))
        row1 = min(self.document.height, int((vh - oy) / z) + 1)

        y_top = max(0, int(oy + row0 * z))
        y_bot = min(vh, int(oy + row1 * z))
        for c in range(col0, col1 + 1):
            px = int(ox + c * z)
            painter.drawLine(px, y_top, px, y_bot)

        x_left = max(0, int(ox + col0 * z))
        x_right = min(vw, int(ox + col1 * z))
        for r in range(row0, row1 + 1):
            py = int(oy + r * z)
            painter.drawLine(x_left, py, x_right, py)

    def _draw_shape_preview(self, painter):
        sx, sy = self._shape_start
        ex, ey = self._shape_end
        color = BRUSH_COLORS[self.brush_mode]
        ox = self.offset.x()
        oy = self.offset.y()
        z = self.zoom

        if self.tool == TOOL_RECT:
            x0, x1 = min(sx, ex), max(sx, ex)
            y0, y1 = min(sy, ey), max(sy, ey)
            rect = QRectF(ox + x0 * z, oy + y0 * z,
                          (x1 - x0 + 1) * z, (y1 - y0 + 1) * z)
            fill = QColor(color)
            fill.setAlpha(80)
            painter.fillRect(rect, fill)
            pen = QPen(color)
            pen.setWidth(2)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.drawRect(rect)

        elif self.tool == TOOL_LINE:
            pen = QPen(color)
            pen.setWidth(2)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(
                int(ox + (sx + 0.5) * z), int(oy + (sy + 0.5) * z),
                int(ox + (ex + 0.5) * z), int(oy + (ey + 0.5) * z),
            )

    def _draw_cursor(self, painter):
        cx, cy = self.current_cell
        ox = self.offset.x()
        oy = self.offset.y()
        z = self.zoom

        if self.tool == TOOL_ERASER:
            color = QColor(255, 100, 100)
        else:
            color = BRUSH_COLORS[self.brush_mode]

        pen = QPen(color)
        pen.setWidth(2)
        painter.setPen(pen)

        if self.tool in (TOOL_BRUSH, TOOL_ERASER):
            r = self.brush_size - 1
            side = self.brush_size * 2 - 1
            rect = QRectF(ox + (cx - r) * z, oy + (cy - r) * z,
                          side * z, side * z)
            painter.drawRect(rect)
        else:
            rect = QRectF(ox + cx * z, oy + cy * z, z, z)
            painter.drawRect(rect)

    # ---- mouse events ----

    def wheelEvent(self, event):
        if not self.document.has_map():
            return
        before = self._widget_to_cell_float(event.pos())
        scale = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.zoom = max(1.0, min(64.0, self.zoom * scale))
        after = self._widget_to_cell_float(event.pos())
        if before is not None and after is not None:
            d = after - before
            self.offset += QPointF(d.x() * self.zoom, d.y() * self.zoom)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.panning = True
            self.last_pan_pos = event.pos()
            return

        if event.button() != Qt.LeftButton:
            return

        if self.tool in (TOOL_BRUSH, TOOL_ERASER):
            self.painting = True
            self._stroke_cache = {}
            self._paint_at(event.pos())
        elif self.tool in (TOOL_RECT, TOOL_LINE):
            cell = self._widget_to_cell(event.pos())
            if cell is not None:
                self._shape_start = cell
                self._shape_end = cell
                self._shape_dragging = True
                self.update()

    def mouseMoveEvent(self, event):
        cell = self._widget_to_cell(event.pos())
        self.current_cell = cell
        if cell is not None:
            self.hover_changed.emit(cell[0], cell[1],
                                    self.document.value_at(cell[0], cell[1]))

        if self.panning:
            delta = event.pos() - self.last_pan_pos
            self.offset += QPointF(delta.x(), delta.y())
            self.last_pan_pos = event.pos()
            self.update()
        elif self.painting:
            self._paint_at(event.pos())
        elif self._shape_dragging and cell is not None:
            self._shape_end = cell
            self.update()
        else:
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.panning = False
            return
        if event.button() != Qt.LeftButton:
            return

        if self.painting:
            self.painting = False
            changes = list(self._stroke_cache.values())
            self._stroke_cache = {}
            self.document.apply_stroke(changes)
        elif self._shape_dragging:
            self._shape_dragging = False
            if self._shape_start and self._shape_end:
                if self.tool == TOOL_RECT:
                    self._commit_rect()
                elif self.tool == TOOL_LINE:
                    self._commit_line()
            self._shape_start = None
            self._shape_end = None
            self.update()

    def leaveEvent(self, _event):
        self.current_cell = None
        self.update()

    # ---- tool commit helpers ----

    def _paint_at(self, pos):
        center = self._widget_to_cell(pos)
        if center is None:
            return

        radius = self.brush_size - 1
        for y in range(center[1] - radius, center[1] + radius + 1):
            for x in range(center[0] - radius, center[0] + radius + 1):
                if not (0 <= x < self.document.width
                        and 0 <= y < self.document.height):
                    continue
                if self.tool == TOOL_ERASER:
                    value = self.document.original_value_at(x, y)
                else:
                    value = BRUSH_VALUES[self.brush_mode]
                key = (x, y)
                old = self.document.set_value(x, y, value)
                if old is None:
                    continue
                if key not in self._stroke_cache:
                    self._stroke_cache[key] = (x, y, old, value)
                else:
                    self._stroke_cache[key] = (x, y,
                                               self._stroke_cache[key][2],
                                               value)
        self.update()

    def _commit_rect(self):
        sx, sy = self._shape_start
        ex, ey = self._shape_end
        x0, x1 = min(sx, ex), max(sx, ex)
        y0, y1 = min(sy, ey), max(sy, ey)

        value = BRUSH_VALUES[self.brush_mode]
        changes = []
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                old = self.document.set_value(x, y, value)
                if old is not None:
                    changes.append((x, y, old, value))
        self.document.apply_stroke(changes)

    def _commit_line(self):
        sx, sy = self._shape_start
        ex, ey = self._shape_end

        value = BRUSH_VALUES[self.brush_mode]
        cells = bresenham(sx, sy, ex, ey)
        cache = {}
        radius = self.brush_size - 1
        for cx, cy in cells:
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    x, y = cx + dx, cy + dy
                    if not (0 <= x < self.document.width
                            and 0 <= y < self.document.height):
                        continue
                    key = (x, y)
                    old = self.document.set_value(x, y, value)
                    if old is None:
                        continue
                    if key not in cache:
                        cache[key] = (x, y, old, value)
                    else:
                        cache[key] = (x, y, cache[key][2], value)
        self.document.apply_stroke(list(cache.values()))

    # ---- coordinate helpers ----

    def _widget_to_cell(self, pos):
        if not self.document.has_map():
            return None
        x = int((pos.x() - self.offset.x()) / self.zoom)
        y = int((pos.y() - self.offset.y()) / self.zoom)
        if 0 <= x < self.document.width and 0 <= y < self.document.height:
            return x, y
        return None

    def _widget_to_cell_float(self, pos):
        if not self.document.has_map():
            return None
        return QPointF(
            (pos.x() - self.offset.x()) / self.zoom,
            (pos.y() - self.offset.y()) / self.zoom,
        )
