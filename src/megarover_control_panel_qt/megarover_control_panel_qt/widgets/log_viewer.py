#!/usr/bin/env python3
"""
Log viewer widget for displaying component logs.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTextEdit, QPushButton, QHBoxLayout
from PyQt5.QtGui import QTextCursor
from PyQt5.QtCore import Qt


class LogViewer(QWidget):
    """Widget for displaying colored log output."""

    MAX_LINES = 1000  # Limit to prevent memory issues

    def __init__(self):
        super().__init__()
        self.init_ui()
        self.line_count = 0

    def init_ui(self):
        """Initialize the UI."""
        layout = QVBoxLayout()

        # Log text area
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setLineWrapMode(QTextEdit.NoWrap)
        self.text_edit.setStyleSheet("""
            QTextEdit {
                background-color: #1E1E1E;
                color: #D4D4D4;
                font-family: 'Courier New', monospace;
                font-size: 10pt;
                border: 1px solid #3E3E3E;
            }
        """)

        layout.addWidget(self.text_edit)

        # Control buttons
        button_layout = QHBoxLayout()

        self.clear_btn = QPushButton('清除日志')
        self.clear_btn.clicked.connect(self.clear_logs)
        button_layout.addWidget(self.clear_btn)

        self.autoscroll_btn = QPushButton('自动滚动: 开')
        self.autoscroll_btn.setCheckable(True)
        self.autoscroll_btn.setChecked(True)
        self.autoscroll_btn.clicked.connect(self.toggle_autoscroll)
        button_layout.addWidget(self.autoscroll_btn)

        button_layout.addStretch()

        layout.addLayout(button_layout)

        self.setLayout(layout)

    def append_log(self, html_message):
        """
        Append a log message (HTML formatted).

        Args:
            html_message: HTML formatted log message
        """
        # Limit line count
        if self.line_count >= self.MAX_LINES:
            # Remove first line
            cursor = self.text_edit.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.select(QTextCursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()  # Remove newline
            self.line_count -= 1

        # Append new line
        self.text_edit.append(html_message)
        self.line_count += 1

        # Auto-scroll to bottom
        if self.autoscroll_btn.isChecked():
            scrollbar = self.text_edit.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

    def clear_logs(self):
        """Clear all logs."""
        self.text_edit.clear()
        self.line_count = 0

    def toggle_autoscroll(self):
        """Toggle auto-scroll feature."""
        if self.autoscroll_btn.isChecked():
            self.autoscroll_btn.setText('自动滚动: 开')
        else:
            self.autoscroll_btn.setText('自动滚动: 关')
