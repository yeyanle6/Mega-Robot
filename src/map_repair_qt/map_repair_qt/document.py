import os
from dataclasses import dataclass

import yaml
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QColor, QImage


FREE_VALUE = 254
OCCUPIED_VALUE = 0
UNKNOWN_VALUE = 205

BRUSH_VALUES = {
    "occupied": OCCUPIED_VALUE,
    "free": FREE_VALUE,
    "unknown": UNKNOWN_VALUE,
}

BRUSH_COLORS = {
    "occupied": QColor(0, 0, 0),
    "free": QColor(255, 255, 255),
    "unknown": QColor(128, 128, 128),
}


@dataclass
class MapMetadata:
    image: str = ""
    resolution: float = 0.05
    origin: list = None
    negate: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196
    mode: str = "trinary"

    def to_dict(self):
        return {
            "image": self.image,
            "resolution": self.resolution,
            "origin": self.origin or [0.0, 0.0, 0.0],
            "negate": self.negate,
            "occupied_thresh": self.occupied_thresh,
            "free_thresh": self.free_thresh,
            "mode": self.mode,
        }


class MapDocument(QObject):
    changed = pyqtSignal()
    metadata_changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.image = QImage()
        self.original_image = QImage()
        self.metadata = MapMetadata(origin=[0.0, 0.0, 0.0])
        self.yaml_path = ""
        self.image_path = ""
        self.dirty = False
        self.undo_stack = []
        self.redo_stack = []

    @property
    def width(self):
        return self.image.width()

    @property
    def height(self):
        return self.image.height()

    def has_map(self):
        return not self.image.isNull()

    def load_yaml(self, yaml_path):
        with open(yaml_path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        image_ref = data.get("image", "")
        image_path = image_ref
        if image_ref and not os.path.isabs(image_ref):
            image_path = os.path.normpath(os.path.join(os.path.dirname(yaml_path), image_ref))

        image = QImage(image_path)
        if image.isNull():
            raise ValueError(f"failed to load image: {image_path}")

        self.image = image.convertToFormat(QImage.Format_RGB32)
        self.original_image = self.image.copy()
        self.yaml_path = yaml_path
        self.image_path = image_path
        self.metadata = MapMetadata(
            image=image_ref,
            resolution=float(data.get("resolution", 0.05)),
            origin=list(data.get("origin", [0.0, 0.0, 0.0])),
            negate=int(data.get("negate", 0)),
            occupied_thresh=float(data.get("occupied_thresh", 0.65)),
            free_thresh=float(data.get("free_thresh", 0.196)),
            mode=str(data.get("mode", "trinary")),
        )
        self.dirty = False
        self.undo_stack.clear()
        self.redo_stack.clear()
        self.metadata_changed.emit()
        self.changed.emit()

    def save_yaml(self, yaml_path, image_path=None):
        if not self.has_map():
            raise ValueError("no map loaded")

        if image_path is None:
            image_path = self.image_path or os.path.splitext(yaml_path)[0] + ".pgm"

        if not os.path.isabs(image_path):
            abs_image_path = os.path.normpath(os.path.join(os.path.dirname(yaml_path), image_path))
        else:
            abs_image_path = image_path

        grayscale = self.image.convertToFormat(QImage.Format_Grayscale8)
        if not grayscale.save(abs_image_path):
            raise ValueError(f"failed to save image: {abs_image_path}")

        image_ref = image_path
        if os.path.isabs(image_path):
            image_ref = os.path.relpath(abs_image_path, os.path.dirname(yaml_path))

        self.metadata.image = image_ref
        with open(yaml_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(self.metadata.to_dict(), stream, sort_keys=False)

        self.yaml_path = yaml_path
        self.image_path = abs_image_path
        self.dirty = False
        self.metadata_changed.emit()

    def value_at(self, x, y):
        color = self.image.pixelColor(x, y)
        return color.red()

    def original_value_at(self, x, y):
        if self.original_image.isNull():
            return self.value_at(x, y)
        color = self.original_image.pixelColor(x, y)
        return color.red()

    def set_value(self, x, y, value):
        if not (0 <= x < self.width and 0 <= y < self.height):
            return None
        old_value = self.value_at(x, y)
        if old_value == value:
            return None
        self.image.setPixelColor(x, y, QColor(value, value, value))
        self.dirty = True
        return old_value

    def apply_stroke(self, stroke_changes):
        if not stroke_changes:
            return
        self.undo_stack.append(stroke_changes)
        self.redo_stack.clear()
        self.dirty = True
        self.changed.emit()

    def undo(self):
        if not self.undo_stack:
            return
        changes = self.undo_stack.pop()
        redo_changes = []
        for x, y, old_value, new_value in reversed(changes):
            self.image.setPixelColor(x, y, QColor(old_value, old_value, old_value))
            redo_changes.append((x, y, old_value, new_value))
        self.redo_stack.append(list(reversed(redo_changes)))
        self.dirty = True
        self.changed.emit()

    def redo(self):
        if not self.redo_stack:
            return
        changes = self.redo_stack.pop()
        undo_changes = []
        for x, y, old_value, new_value in changes:
            self.image.setPixelColor(x, y, QColor(new_value, new_value, new_value))
            undo_changes.append((x, y, old_value, new_value))
        self.undo_stack.append(undo_changes)
        self.dirty = True
        self.changed.emit()
