#!/usr/bin/env python3
"""
Component control widget for starting/stopping ROS2 components.
"""

import os
from PyQt5.QtWidgets import (QWidget, QHBoxLayout, QLabel, QPushButton,
                              QVBoxLayout, QFrame, QFileDialog, QMessageBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPalette, QColor


class ComponentControlWidget(QWidget):
    """Widget for controlling a single ROS2 component."""

    def __init__(self, component_id, component_config, process_manager, status_aggregator):
        super().__init__()
        self.component_id = component_id
        self.component_config = component_config
        self.process_manager = process_manager
        self.status_aggregator = status_aggregator

        self.init_ui()

        # Timer for periodic status updates
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)  # Update every second

    def init_ui(self):
        """Initialize the UI."""
        layout = QHBoxLayout()

        # Status indicator (colored circle)
        self.status_indicator = QLabel('●')
        self.status_indicator.setStyleSheet('font-size: 16pt; color: gray;')
        self.status_indicator.setFixedWidth(30)
        layout.addWidget(self.status_indicator)

        # Component name
        name_label = QLabel(self.component_config['name'])
        name_label.setStyleSheet('font-weight: bold; font-size: 11pt;')
        name_label.setFixedWidth(180)
        layout.addWidget(name_label)

        # Toggle button (启动/停止)
        self.toggle_btn = QPushButton('启动')
        self.toggle_btn.setFixedWidth(100)
        self.toggle_btn.clicked.connect(self.toggle_component)
        self._update_button_style(is_running=False)
        layout.addWidget(self.toggle_btn)

        # Status text
        self.status_label = QLabel('未运行')
        self.status_label.setStyleSheet('color: gray; font-size: 10pt;')
        self.status_label.setFixedWidth(120)
        layout.addWidget(self.status_label)

        layout.addStretch()

        self.setLayout(layout)

    def _update_button_style(self, is_running):
        """Update button text and style based on running state."""
        if is_running:
            # Running - show "停止" button in red
            self.toggle_btn.setText('停止')
            self.toggle_btn.setStyleSheet("""
                QPushButton {
                    background-color: #E74C3C;
                    color: white;
                    border: none;
                    padding: 5px;
                    border-radius: 3px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #C0392B;
                }
                QPushButton:disabled {
                    background-color: #7F8C8D;
                }
            """)
        else:
            # Stopped - show "启动" button in green
            self.toggle_btn.setText('启动')
            self.toggle_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2ECC71;
                    color: white;
                    border: none;
                    padding: 5px;
                    border-radius: 3px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #27AE60;
                }
                QPushButton:disabled {
                    background-color: #7F8C8D;
                }
            """)

    def toggle_component(self):
        """Toggle component state (start/stop)."""
        # Disable button during operation to prevent double-clicks
        self.toggle_btn.setEnabled(False)
        original_text = self.toggle_btn.text()

        # Check if component is currently running
        is_running = self.process_manager.is_running(self.component_id)

        try:
            if is_running:
                # Stop the component
                self.toggle_btn.setText('停止中...')
                self.process_manager.stop_component(self.component_id)
            else:
                # Start the component
                # Special handling for navigation - need to select map
                if self.component_id == 'navigation':
                    # Re-enable button for dialog
                    self.toggle_btn.setEnabled(True)
                    self.toggle_btn.setText(original_text)

                    map_path, _ = QFileDialog.getOpenFileName(
                        self,
                        '选择2D地图文件',
                        '/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps',
                        'YAML Files (*.yaml)'
                    )

                    if not map_path:
                        return  # User cancelled

                    self.process_manager.set_param('navigation', 'map_path', map_path)

                    # Disable again for start operation
                    self.toggle_btn.setEnabled(False)

                # Special handling for navigation_pgo - need to select 2D map AND point cloud map
                elif self.component_id == 'navigation_pgo':
                    # Re-enable button for dialogs
                    self.toggle_btn.setEnabled(True)
                    self.toggle_btn.setText(original_text)

                    # Show info message first
                    from PyQt5.QtWidgets import QMessageBox
                    QMessageBox.information(
                        self,
                        'PGO 导航 - 地图选择',
                        '启动 PGO 导航需要选择两个地图文件：\n\n'
                        '1. 2D 栅格地图 (.yaml) - 用于路径规划\n'
                        '2. 3D 点云地图 (.pcd) - 用于精确定位\n\n'
                        '请在接下来的两个对话框中依次选择这两个文件。'
                    )

                    # Select 2D map (first dialog)
                    from PyQt5.QtCore import Qt
                    dialog = QFileDialog(self)
                    dialog.setWindowTitle('【步骤 1/2】选择 2D 地图文件 (.yaml)')
                    dialog.setDirectory('/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps')
                    dialog.setNameFilter('YAML Files (*.yaml)')
                    dialog.setFileMode(QFileDialog.ExistingFile)
                    dialog.setWindowFlags(dialog.windowFlags() | Qt.WindowStaysOnTopHint)

                    if dialog.exec_() != QFileDialog.Accepted:
                        return  # User cancelled

                    map_path = dialog.selectedFiles()[0]

                    # Select point cloud map (second dialog)
                    dialog2 = QFileDialog(self)
                    dialog2.setWindowTitle('【步骤 2/2】选择 3D 点云地图文件 (.pcd)')
                    dialog2.setDirectory('/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps')
                    dialog2.setNameFilter('PCD Files (*.pcd);;All Files (*)')
                    dialog2.setFileMode(QFileDialog.ExistingFile)
                    dialog2.setWindowFlags(dialog2.windowFlags() | Qt.WindowStaysOnTopHint)

                    if dialog2.exec_() != QFileDialog.Accepted:
                        return  # User cancelled

                    pcd_path = dialog2.selectedFiles()[0]

                    # Set parameters
                    self.process_manager.set_param('navigation_pgo', 'map_path', map_path)
                    self.process_manager.set_param('navigation_pgo', 'pcd_path', pcd_path)

                    # Show confirmation
                    QMessageBox.information(
                        self,
                        '地图已选择',
                        f'已选择地图文件：\n\n'
                        f'2D 地图:\n{os.path.basename(map_path)}\n\n'
                        f'3D 点云:\n{os.path.basename(pcd_path)}\n\n'
                        f'即将启动 PGO 导航...'
                    )

                    # Disable again for start operation
                    self.toggle_btn.setEnabled(False)

                # Start the component
                self.toggle_btn.setText('启动中...')
                self.process_manager.start_component(self.component_id)
        finally:
            # Re-enable button after operation
            self.toggle_btn.setEnabled(True)

    def update_status(self):
        """Update status indicator based on current state."""
        status = self.status_aggregator.get_component_status(
            self.component_id,
            self.component_config
        )

        # Determine if running (running or partial states)
        is_running = status in ['running', 'partial']

        # Update button style
        self._update_button_style(is_running)

        # Update indicator color and status text
        if status == 'running':
            self.status_indicator.setStyleSheet('font-size: 16pt; color: #2ECC71;')  # Green
            self.status_label.setText('运行中')
            self.status_label.setStyleSheet('color: #2ECC71; font-size: 10pt;')
        elif status == 'partial':
            self.status_indicator.setStyleSheet('font-size: 16pt; color: #F39C12;')  # Orange
            self.status_label.setText('部分运行')
            self.status_label.setStyleSheet('color: #F39C12; font-size: 10pt;')
        elif status == 'error':
            self.status_indicator.setStyleSheet('font-size: 16pt; color: #E74C3C;')  # Red
            self.status_label.setText('错误')
            self.status_label.setStyleSheet('color: #E74C3C; font-size: 10pt;')
        else:  # stopped
            self.status_indicator.setStyleSheet('font-size: 16pt; color: #7F8C8D;')  # Gray
            self.status_label.setText('未运行')
            self.status_label.setStyleSheet('color: #7F8C8D; font-size: 10pt;')


class ComponentControlPanel(QWidget):
    """Panel containing all component control widgets."""

    def __init__(self, config, process_manager, status_aggregator):
        super().__init__()
        self.config = config
        self.process_manager = process_manager
        self.status_aggregator = status_aggregator

        self.init_ui()

    def init_ui(self):
        """Initialize the UI."""
        layout = QVBoxLayout()

        # Title
        title = QLabel('组件控制')
        title.setStyleSheet('font-size: 14pt; font-weight: bold; margin-bottom: 10px;')
        layout.addWidget(title)

        # Add separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator)

        # Add component control widgets
        for component_id, component_config in self.config['components'].items():
            widget = ComponentControlWidget(
                component_id,
                component_config,
                self.process_manager,
                self.status_aggregator
            )
            layout.addWidget(widget)

        layout.addStretch()

        self.setLayout(layout)
