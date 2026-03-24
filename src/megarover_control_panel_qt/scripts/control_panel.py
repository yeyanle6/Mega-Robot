#!/usr/bin/env python3
"""
MegaRover3 Control Panel startup script.
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

from megarover_control_panel_qt.main_window import MainWindow


def main():
    """Main entry point."""
    app = QApplication(sys.argv)

    # Set application metadata
    app.setApplicationName('MegaRover3 Control Panel')
    app.setOrganizationName('MegaRover3')

    # Create and show main window
    window = MainWindow()
    window.show()

    # Run event loop
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
