#!/usr/bin/env python3
"""
Navigation Test Workbench — entry point.

Usage:
  ros2 run megarover_control_panel_qt nav_test_workbench.py
  # or directly:
  python3 nav_test_workbench.py
"""

import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
candidates = [
    script_dir,
    os.path.dirname(script_dir),
]
for candidate in candidates:
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt

from megarover_control_panel_qt.nav_workbench.workbench_window import WorkbenchWindow


def main():
    # Allow high-DPI scaling
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Dark palette
    from PyQt5.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(45, 45, 45))
    palette.setColor(QPalette.WindowText, QColor(208, 208, 208))
    palette.setColor(QPalette.Base, QColor(30, 30, 30))
    palette.setColor(QPalette.AlternateBase, QColor(45, 45, 45))
    palette.setColor(QPalette.ToolTipBase, QColor(45, 45, 45))
    palette.setColor(QPalette.ToolTipText, QColor(208, 208, 208))
    palette.setColor(QPalette.Text, QColor(208, 208, 208))
    palette.setColor(QPalette.Button, QColor(55, 55, 55))
    palette.setColor(QPalette.ButtonText, QColor(208, 208, 208))
    palette.setColor(QPalette.BrightText, QColor(255, 50, 50))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    window = WorkbenchWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
