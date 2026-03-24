#!/usr/bin/env python3

import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
candidates = [
    script_dir,
    os.path.dirname(script_dir),
]

for candidate in candidates:
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication

from map_repair_qt.main_window import MapRepairMainWindow


def main():
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    app.setApplicationName("\u5730\u56fe\u4fee\u590d\u5de5\u5177")
    app.setOrganizationName("Demo8")
    app.setStyle("Fusion")

    window = MapRepairMainWindow()
    if len(sys.argv) > 1:
        window.load_map(sys.argv[1])
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
