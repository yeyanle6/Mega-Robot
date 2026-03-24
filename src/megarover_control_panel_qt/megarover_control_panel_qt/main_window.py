#!/usr/bin/env python3
"""
Main window for MegaRover3 control panel.
"""

import os
import yaml
import time
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                              QPushButton, QSplitter, QMessageBox, QFileDialog,
                              QLabel, QFrame)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIcon

from .process_manager import ProcessManager
from .ros2_monitor import ROS2Monitor, ComponentStatusAggregator
from .log_handler import LogFormatter
from .widgets import ComponentControlPanel, LogViewer


class MainWindow(QMainWindow):
    """Main control panel window."""

    def __init__(self):
        super().__init__()
        self.config = self.load_config()
        self.process_manager = ProcessManager(self.config)
        self.status_aggregator = ComponentStatusAggregator()
        self.ros2_monitor = None

        self.init_ui()
        self.connect_signals()
        self.start_ros2_monitor()
        self.start_health_monitor()

    def load_config(self):
        """Load configuration from YAML file."""
        config_path = os.path.join(
            os.path.dirname(__file__),
            'config',
            'panel_config.yaml'
        )

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception as e:
            QMessageBox.critical(
                None,
                '配置错误',
                f'无法加载配置文件: {str(e)}'
            )
            raise

    def init_ui(self):
        """Initialize the UI."""
        self.setWindowTitle('MegaRover3 控制面板')

        # Set initial size and minimum size
        self.resize(1200, 800)
        self.setMinimumSize(800, 600)  # Allow resizing but set minimum

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout()

        # Title bar
        title_layout = QHBoxLayout()
        title_label = QLabel('🤖 MegaRover3 机器人控制面板')
        title_label.setStyleSheet('font-size: 18pt; font-weight: bold; margin: 10px;')
        title_layout.addWidget(title_label)
        title_layout.addStretch()
        main_layout.addLayout(title_layout)

        # Splitter for component control and log viewer
        splitter = QSplitter(Qt.Horizontal)

        # Left panel - Component control (with scroll area)
        from PyQt5.QtWidgets import QScrollArea

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        left_panel = QWidget()
        left_layout = QVBoxLayout()

        # Component control panel
        self.component_panel = ComponentControlPanel(
            self.config,
            self.process_manager,
            self.status_aggregator
        )
        left_layout.addWidget(self.component_panel)

        # Quick action buttons
        self.init_quick_actions(left_layout)

        left_panel.setLayout(left_layout)
        left_scroll.setWidget(left_panel)
        splitter.addWidget(left_scroll)

        # Right panel - Log viewer
        right_panel = QWidget()
        right_layout = QVBoxLayout()

        # Log viewer
        log_title = QLabel('日志输出')
        log_title.setStyleSheet('font-size: 14pt; font-weight: bold; margin-bottom: 5px;')
        right_layout.addWidget(log_title)

        self.log_viewer = LogViewer()
        right_layout.addWidget(self.log_viewer)

        right_panel.setLayout(right_layout)
        splitter.addWidget(right_panel)

        # Set splitter proportions (30% left, 70% right)
        splitter.setStretchFactor(0, 30)
        splitter.setStretchFactor(1, 70)

        main_layout.addWidget(splitter)

        central_widget.setLayout(main_layout)

        # Apply stylesheet
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2C3E50;
            }
            QWidget {
                background-color: #34495E;
                color: #ECF0F1;
            }
            QPushButton {
                background-color: #3498DB;
                color: white;
                border: none;
                padding: 8px 15px;
                border-radius: 4px;
                font-weight: bold;
                font-size: 11pt;
            }
            QPushButton:hover {
                background-color: #2980B9;
            }
            QPushButton:pressed {
                background-color: #21618C;
            }
            QLabel {
                color: #ECF0F1;
            }
        """)

    def init_quick_actions(self, layout):
        """Initialize quick action buttons."""
        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator)

        # Title
        title = QLabel('快捷操作')
        title.setStyleSheet('font-size: 14pt; font-weight: bold; margin-top: 10px; margin-bottom: 10px;')
        layout.addWidget(title)

        # One-click mapping
        self.mapping_btn = QPushButton('🗺️ 一键启动建图')
        self.mapping_btn.clicked.connect(self.start_mapping_flow)
        self.mapping_btn.setStyleSheet("""
            QPushButton {
                background-color: #16A085;
                padding: 10px;
                font-size: 12pt;
            }
            QPushButton:hover {
                background-color: #138D75;
            }
        """)
        layout.addWidget(self.mapping_btn)

        # One-click navigation
        self.nav_btn = QPushButton('🧭 一键启动导航')
        self.nav_btn.clicked.connect(self.start_navigation_flow)
        self.nav_btn.setStyleSheet("""
            QPushButton {
                background-color: #2980B9;
                padding: 10px;
                font-size: 12pt;
            }
            QPushButton:hover {
                background-color: #21618C;
            }
        """)
        layout.addWidget(self.nav_btn)

        # Save map
        self.save_map_btn = QPushButton('💾 保存地图')
        self.save_map_btn.clicked.connect(self.save_map)
        layout.addWidget(self.save_map_btn)

        # Kill all nodes
        self.kill_all_btn = QPushButton('⚠️ 清除所有节点')
        self.kill_all_btn.clicked.connect(self.kill_all_nodes)
        self.kill_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #C0392B;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #A93226;
            }
        """)
        layout.addWidget(self.kill_all_btn)

        layout.addStretch()

    def connect_signals(self):
        """Connect signals to slots."""
        # Process manager signals
        self.process_manager.log_received.connect(self.on_log_received)
        self.process_manager.status_changed.connect(self.on_status_changed)

    def on_log_received(self, component_id, level, message):
        """Handle log message from process manager."""
        # Strip ANSI codes
        message = LogFormatter.strip_ansi(message)

        # Format as HTML
        html_message = LogFormatter.format_html(level, message, component_id)

        # Display in log viewer
        self.log_viewer.append_log(html_message)

    def on_status_changed(self, component_id, status):
        """Handle status change from process manager."""
        self.status_aggregator.update_process_status(component_id, status)

    def on_topic_status_changed(self, topic, is_active, hz):
        """Handle topic status change from ROS2 monitor."""
        self.status_aggregator.update_topic_status(topic, is_active)

    def on_node_status_changed(self, node, is_active):
        """Handle node status change from ROS2 monitor."""
        self.status_aggregator.update_node_status(node, is_active)

    def start_ros2_monitor(self):
        """Start ROS2 monitoring thread."""
        try:
            self.ros2_monitor = ROS2Monitor(self.config)
            self.ros2_monitor.topic_status_changed.connect(self.on_topic_status_changed)
            self.ros2_monitor.node_status_changed.connect(self.on_node_status_changed)
            self.ros2_monitor.start()
        except Exception as e:
            self.log_viewer.append_log(
                LogFormatter.format_html('ERROR', f'Failed to start ROS2 monitor: {str(e)}', 'system')
            )

    def start_health_monitor(self):
        """Start periodic health monitoring."""
        self.health_timer = QTimer()
        self.health_timer.timeout.connect(self.process_manager.monitor_health)
        self.health_timer.start(2000)  # Check every 2 seconds

    def start_mapping_flow(self):
        """One-click start mapping flow."""
        reply = QMessageBox.question(
            self,
            '启动建图',
            '将按顺序启动：底盘 → 激光雷达 → 前方相机 → SLAM建图\n\n确定要继续吗？',
            QMessageBox.Yes | QMessageBox.No
        )

        if reply != QMessageBox.Yes:
            return

        # Start sequence
        components = ['chassis', 'lidar', 'd455_front', 'slam']

        for i, component_id in enumerate(components):
            if self.process_manager.is_running(component_id):
                self.log_viewer.append_log(
                    LogFormatter.format_html('INFO', f'{component_id} already running, skipping', 'system')
                )
                continue

            self.log_viewer.append_log(
                LogFormatter.format_html('INFO', f'Starting {component_id}... ({i+1}/{len(components)})', 'system')
            )

            success = self.process_manager.start_component(component_id)

            if not success:
                QMessageBox.warning(
                    self,
                    '启动失败',
                    f'组件 {component_id} 启动失败，流程中止。'
                )
                return

            # Wait a bit before starting next component
            if i < len(components) - 1:
                time.sleep(2)

        QMessageBox.information(
            self,
            '建图启动完成',
            '所有组件已启动，请检查状态指示灯。'
        )

    def start_navigation_flow(self):
        """One-click start navigation flow."""
        # Select map file
        map_path, _ = QFileDialog.getOpenFileName(
            self,
            '选择地图文件',
            '/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps',
            'YAML Files (*.yaml)'
        )

        if not map_path:
            return

        reply = QMessageBox.question(
            self,
            '启动导航',
            f'将使用地图: {os.path.basename(map_path)}\n\n'
            '将按顺序启动：底盘 → 激光雷达 → 前方相机 → 导航\n\n确定要继续吗？',
            QMessageBox.Yes | QMessageBox.No
        )

        if reply != QMessageBox.Yes:
            return

        # Set map path parameter
        self.process_manager.set_param('navigation', 'map_path', map_path)

        # Start sequence
        components = ['chassis', 'lidar', 'd455_front', 'navigation']

        for i, component_id in enumerate(components):
            if self.process_manager.is_running(component_id):
                self.log_viewer.append_log(
                    LogFormatter.format_html('INFO', f'{component_id} already running, skipping', 'system')
                )
                continue

            self.log_viewer.append_log(
                LogFormatter.format_html('INFO', f'Starting {component_id}... ({i+1}/{len(components)})', 'system')
            )

            success = self.process_manager.start_component(component_id)

            if not success:
                QMessageBox.warning(
                    self,
                    '启动失败',
                    f'组件 {component_id} 启动失败，流程中止。'
                )
                return

            # Wait a bit before starting next component
            if i < len(components) - 1:
                time.sleep(2)

        QMessageBox.information(
            self,
            '导航启动完成',
            '所有组件已启动，请检查状态指示灯。'
        )

    def save_map(self):
        """Save current map (2D + 3D complete) asynchronously."""
        # Ask for map name
        from PyQt5.QtWidgets import QInputDialog
        map_name, ok = QInputDialog.getText(
            self,
            '保存地图',
            '请输入地图名称（不含扩展名）:\n\n'
            '保存流程：\n'
            '1. 保存 2D 栅格地图（SLAM 运行中）\n'
            '2. 自动停止 SLAM\n'
            '3. 复制 3D 点云地图（SLAM 已停止）\n'
            '4. 询问是否重启 SLAM'
        )

        if not ok or not map_name:
            return

        # Build save path
        maps_dir = '/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps'
        map_path = os.path.join(maps_dir, map_name)

        # Start map saving (will check SLAM status in thread)
        self._start_map_saving(map_path, map_name)

    def _start_map_saving(self, map_path, map_name):
        """Start the actual map saving process"""
        # Log start
        self.log_viewer.append_log(
            LogFormatter.format_html('INFO', f'开始保存地图: {map_name}', 'system')
        )
        self.log_viewer.append_log(
            LogFormatter.format_html('INFO', '流程: PGO保存3D → 保存2D → 停止SLAM', 'system')
        )

        # Create and show progress dialog
        from .map_save_dialog import MapSaveProgressDialog, MapSaveResultDialog
        from .map_saver_thread import MapSaverThread

        self.map_save_dialog = MapSaveProgressDialog(self)
        # 传递 process_manager 用于停止 SLAM
        self.map_saver_thread = MapSaverThread(map_path, self.process_manager)

        # Connect signals
        self.map_saver_thread.progress_updated.connect(self.on_map_save_progress)
        self.map_saver_thread.save_completed.connect(self.on_map_save_completed)
        self.map_saver_thread.error_occurred.connect(self.on_map_save_error)

        # Handle dialog rejection (cancel)
        self.map_save_dialog.rejected.connect(self.on_map_save_cancelled)

        # Start saving
        self.map_saver_thread.start()
        self.map_save_dialog.exec_()

    def on_map_save_progress(self, progress, message):
        """Handle map save progress update."""
        self.map_save_dialog.update_progress(progress, message)

        # Also log to main window
        self.log_viewer.append_log(
            LogFormatter.format_html('INFO', message, 'system')
        )

    def on_map_save_completed(self, success, result):
        """Handle map save completion."""
        self.map_save_dialog.set_completed()
        self.map_save_dialog.accept()

        # Show result dialog
        from .map_save_dialog import MapSaveResultDialog
        result_dialog = MapSaveResultDialog(success, result, self)
        result_dialog.exec_()

        # Log result
        if success:
            self.log_viewer.append_log(
                LogFormatter.format_html('INFO', '✓ 地图保存成功', 'system')
            )
        else:
            self.log_viewer.append_log(
                LogFormatter.format_html('WARN', '⚠ 地图部分保存', 'system')
            )

        # Ask to restart SLAM (always, since SLAM was stopped during save)
        if hasattr(self.map_saver_thread, 'slam_component') and self.map_saver_thread.slam_component:
            from PyQt5.QtWidgets import QMessageBox
            slam_name = self._get_slam_name(self.map_saver_thread.slam_component)

            reply = QMessageBox.question(
                self,
                '重启 SLAM？',
                f'地图保存完成。\n\n'
                f'在保存过程中已停止 {slam_name}。\n\n'
                f'是否重新启动继续建图？',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )

            if reply == QMessageBox.Yes:
                self.log_viewer.append_log(
                    LogFormatter.format_html('INFO', f'重新启动 {slam_name}...', 'system')
                )
                self.process_manager.start_component(self.map_saver_thread.slam_component)
                self.log_viewer.append_log(
                    LogFormatter.format_html('INFO', '✓ SLAM 已重启，可以继续建图', 'system')
                )
            else:
                self.log_viewer.append_log(
                    LogFormatter.format_html('INFO', 'SLAM 保持停止状态', 'system')
                )

    def _get_slam_name(self, slam_id):
        """Get SLAM component display name"""
        names = {
            'slam': 'SLAM (简单模式)',
            'slam_pgo': 'SLAM PGO (优化模式)'
        }
        return names.get(slam_id, slam_id)

    def on_map_save_error(self, error_type, error_message):
        """Handle map save error."""
        self.map_save_dialog.set_error()

        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.critical(
            self.map_save_dialog,
            f'保存失败: {error_type}',
            error_message
        )

        # Log error
        self.log_viewer.append_log(
            LogFormatter.format_html('ERROR', f'{error_type}: {error_message}', 'system')
        )

    def on_map_save_cancelled(self):
        """Handle map save cancellation."""
        if self.map_saver_thread.isRunning():
            self.map_saver_thread.cancel()
            self.map_saver_thread.wait(5000)  # Wait up to 5 seconds

            self.log_viewer.append_log(
                LogFormatter.format_html('WARN', '地图保存已取消', 'system')
            )

    def kill_all_nodes(self):
        """Kill all ROS2 nodes."""
        reply = QMessageBox.warning(
            self,
            '警告',
            '这将强制终止所有ROS2进程！\n\n确定要继续吗？',
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.process_manager.kill_all_ros_processes()

    def closeEvent(self, event):
        """Handle window close event."""
        reply = QMessageBox.question(
            self,
            '确认退出',
            '确定要退出控制面板吗？\n\n所有运行中的组件将被停止。',
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Stop all processes
            self.process_manager.stop_all()

            # Stop ROS2 monitor
            if self.ros2_monitor:
                self.ros2_monitor.stop()
                self.ros2_monitor.wait(3000)

            event.accept()
        else:
            event.ignore()
