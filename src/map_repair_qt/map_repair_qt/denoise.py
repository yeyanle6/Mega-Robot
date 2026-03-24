"""Connected-component denoise for occupancy grid maps."""

import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from PyQt5.QtGui import QColor, QImage
from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QHBoxLayout,
    QLabel,
    QRadioButton,
    QSpinBox,
    QVBoxLayout,
)

from map_repair_qt.document import FREE_VALUE, OCCUPIED_VALUE


def qimage_to_gray(image):
    """Convert Format_RGB32 QImage to grayscale numpy array (h, w)."""
    gray = image.convertToFormat(QImage.Format_Grayscale8)
    h, w = gray.height(), gray.width()
    ptr = gray.constBits()
    ptr.setsize(h * gray.bytesPerLine())
    arr = np.frombuffer(ptr, dtype=np.uint8).reshape(h, gray.bytesPerLine())
    return arr[:, :w].copy()


def denoise(document, mode, min_size):
    """Run connected-component denoise on the document image.

    Args:
        document: MapDocument with a loaded map.
        mode: ``"remove_obstacles"`` — delete small black blobs;
              ``"fill_holes"`` — fill small non-black pockets.
        min_size: components with fewer pixels than this are processed.

    Returns:
        (changes, num_components, num_pixels) where *changes* is a list of
        ``(x, y, old_value, new_value)`` tuples suitable for
        ``document.apply_stroke()``.
    """
    gray = qimage_to_gray(document.image)
    occ_thresh = int((1.0 - document.metadata.occupied_thresh) * 255)

    if mode == "remove_obstacles":
        mask = (gray <= occ_thresh).astype(np.uint8)
        fill_value = FREE_VALUE
    else:
        mask = (gray > occ_thresh).astype(np.uint8)
        fill_value = OCCUPIED_VALUE

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask, connectivity=8)

    if num_labels <= 1:
        return [], 0, 0

    areas = stats[1:, cv2.CC_STAT_AREA]
    small_ids = np.where(areas < min_size)[0] + 1
    if len(small_ids) == 0:
        return [], 0, 0

    small_mask = np.isin(labels, small_ids)
    ys, xs = np.where(small_mask)

    fill_color = QColor(fill_value, fill_value, fill_value)
    changes = []
    for y, x in zip(ys.tolist(), xs.tolist()):
        old_value = int(gray[y, x])
        if old_value != fill_value:
            document.image.setPixelColor(x, y, fill_color)
            changes.append((x, y, old_value, fill_value))

    return changes, int(len(small_ids)), len(changes)


class DenoiseDialog(QDialog):
    """Parameter dialog for connected-component denoise."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("\u8fde\u901a\u57df\u53bb\u566a")
        self.setMinimumWidth(380)

        layout = QVBoxLayout(self)

        self.remove_radio = QRadioButton(
            "\u5220\u9664\u5c0f\u969c\u788d\u5757 (\u9ed1 \u2192 \u767d)")
        self.fill_radio = QRadioButton(
            "\u586b\u8865\u5c0f\u7a7a\u6d1e (\u975e\u9ed1 \u2192 \u9ed1)")
        self.remove_radio.setChecked(True)
        layout.addWidget(self.remove_radio)
        layout.addWidget(self.fill_radio)

        size_row = QHBoxLayout()
        size_row.addWidget(QLabel("\u6700\u5927\u8fde\u901a\u57df\u9762\u79ef (\u50cf\u7d20):"))
        self.size_spin = QSpinBox()
        self.size_spin.setRange(1, 100000)
        self.size_spin.setValue(10)
        size_row.addWidget(self.size_spin)
        layout.addLayout(size_row)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    @property
    def mode(self):
        if self.remove_radio.isChecked():
            return "remove_obstacles"
        return "fill_holes"

    @property
    def min_size(self):
        return self.size_spin.value()
