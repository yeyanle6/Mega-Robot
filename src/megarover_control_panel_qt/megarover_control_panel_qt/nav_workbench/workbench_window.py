"""
Navigation Test Workbench — main window.

Provides a Qt GUI for:
  - Loading 2D maps and clicking waypoints (unlimited)
  - Running multi-point patrol navigation tests
  - Overlaying 4 trajectory layers (theoretical, global, local, actual)
  - Displaying live metrics and event timeline
  - Comparing planned vs actual paths
"""

import os
import json
import math
import time
import subprocess
import signal
from datetime import datetime
from typing import List, Optional

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QPushButton, QLabel, QSpinBox, QFileDialog, QTableWidget,
    QTableWidgetItem, QHeaderView, QTextEdit, QGroupBox, QCheckBox,
    QMessageBox, QApplication,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QFont, QColor

from .map_widget import MapWidget, Waypoint
from .test_engine import NavTestEngine, Pose2D, RunRecord


class WorkbenchWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Navigation Test Workbench')
        self.resize(1400, 900)

        self._engine = NavTestEngine()
        self._engine.pose_in_map_updated.connect(self._on_odom)
        self._engine.global_plan_updated.connect(self._on_global_plan)
        self._engine.local_plan_updated.connect(self._on_local_plan)
        self._engine.theoretical_path_received.connect(self._on_theoretical)
        self._engine.run_started.connect(self._on_run_started)
        self._engine.run_finished.connect(self._on_run_finished)
        self._engine.event_logged.connect(self._on_event)
        self._engine.engine_ready.connect(self._on_engine_ready)
        self._engine.engine_stopped.connect(self._on_engine_stopped)
        self._engine.test_batch_finished.connect(self._on_test_batch_finished)
        self._engine.pose_refresh_result.connect(self._on_pose_refresh_result)
        self._engine.initial_pose_verified.connect(self._on_initial_pose_verified)

        self._current_run_index = -1
        self._replay_run_index = -1   # -1 = live mode
        self._nav_process = None
        self._nav_log_thread = None
        self._setup_ui()
        self._auto_load_map()

        # Start engine immediately for continuous robot pose monitoring
        self._engine.start()

    # ------------------------------------------------------------------
    # UI setup
    # ------------------------------------------------------------------
    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Left: Map
        self._map_widget = MapWidget()
        splitter.addWidget(self._map_widget)

        # Right: Controls + Metrics + Events
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(4, 4, 4, 4)
        splitter.addWidget(right_panel)

        splitter.setSizes([900, 500])

        # --- Map controls ---
        map_group = QGroupBox('Map')
        map_lay = QHBoxLayout(map_group)
        self._btn_load_map = QPushButton('Load Map')
        self._btn_load_map.clicked.connect(self._load_map)
        self._btn_refresh_pose = QPushButton('Refresh Pose')
        self._btn_refresh_pose.setToolTip('Force one immediate TF pose refresh from map to base_footprint')
        self._btn_refresh_pose.clicked.connect(self._refresh_pose)
        self._btn_center_robot = QPushButton('Center Robot')
        self._btn_center_robot.setToolTip('Center the map view on the latest robot pose')
        self._btn_center_robot.clicked.connect(self._center_robot)
        self._btn_pose_estimate = QPushButton('2D Pose Estimate')
        self._btn_pose_estimate.setToolTip(
            'Click on map to set robot initial position, drag to set orientation')
        self._btn_pose_estimate.setCheckable(True)
        self._btn_pose_estimate.setStyleSheet(
            'QPushButton:checked { background-color: #1a8a1a; color: white; font-weight: bold; }')
        self._btn_pose_estimate.clicked.connect(self._toggle_pose_estimate)
        self._btn_launch_nav = QPushButton('Launch Nav')
        self._btn_launch_nav.setToolTip('一键启动 PGO 导航（底盘+LiDAR+D455+Nav2）')
        self._btn_launch_nav.setStyleSheet(
            'background-color: #1ABC9C; color: white; font-weight: bold;')
        self._btn_launch_nav.clicked.connect(self._launch_nav)
        self._lbl_map = QLabel('No map loaded')
        self._lbl_map.setStyleSheet('color: gray;')
        map_lay.addWidget(self._btn_launch_nav)
        map_lay.addWidget(self._btn_load_map)
        map_lay.addWidget(self._btn_refresh_pose)
        map_lay.addWidget(self._btn_center_robot)
        map_lay.addWidget(self._btn_pose_estimate)
        map_lay.addWidget(self._lbl_map, 1)
        right_layout.addWidget(map_group)

        # --- Waypoints ---
        wp_group = QGroupBox(
            'Waypoints (click map to add, right-click to remove, uncheck to skip in patrol)')
        wp_lay = QVBoxLayout(wp_group)
        self._wp_table = QTableWidget(0, 5)
        self._wp_table.setHorizontalHeaderLabels(['Use', '#', 'X', 'Y', 'Yaw'])
        self._wp_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._wp_table.setMaximumHeight(160)
        wp_lay.addWidget(self._wp_table)
        btn_row = QHBoxLayout()
        self._btn_clear_wp = QPushButton('Clear All')
        self._btn_clear_wp.clicked.connect(self._clear_waypoints)
        btn_row.addWidget(self._btn_clear_wp)
        btn_row.addStretch()
        wp_lay.addLayout(btn_row)
        right_layout.addWidget(wp_group)

        self._map_widget.waypoints_changed.connect(self._refresh_wp_table)
        self._map_widget.pose_estimate_set.connect(self._on_pose_estimate)

        # --- Test control ---
        ctrl_group = QGroupBox('Test Control')
        ctrl_lay = QHBoxLayout(ctrl_group)
        ctrl_lay.addWidget(QLabel('Cycles:'))
        self._spin_cycles = QSpinBox()
        self._spin_cycles.setRange(1, 100)
        self._spin_cycles.setValue(1)
        ctrl_lay.addWidget(self._spin_cycles)
        self._chk_loop = QCheckBox('Infinite')
        self._chk_loop.setToolTip('Repeat the enabled waypoint sequence indefinitely')
        ctrl_lay.addWidget(self._chk_loop)
        ctrl_lay.addStretch()
        self._btn_start = QPushButton('Start')
        self._btn_start.setStyleSheet('background-color: #2d8c2d; color: white; font-weight: bold;')
        self._btn_start.clicked.connect(self._start_test)
        ctrl_lay.addWidget(self._btn_start)
        self._btn_stop = QPushButton('Stop')
        self._btn_stop.setStyleSheet('background-color: #c0392b; color: white;')
        self._btn_stop.clicked.connect(self._stop_test)
        self._btn_stop.setEnabled(False)
        ctrl_lay.addWidget(self._btn_stop)
        self._btn_clear_traj = QPushButton('Clear Trajectories')
        self._btn_clear_traj.clicked.connect(self._clear_trajectories)
        ctrl_lay.addWidget(self._btn_clear_traj)
        self._btn_save = QPushButton('Save Results')
        self._btn_save.clicked.connect(self._save_results)
        ctrl_lay.addWidget(self._btn_save)
        right_layout.addWidget(ctrl_group)

        # --- Metrics (click a row to replay that run's trajectories) ---
        metrics_group = QGroupBox('Run Metrics (click row to replay)')
        metrics_lay = QVBoxLayout(metrics_group)
        self._metrics_table = QTableWidget(0, 10)
        self._metrics_table.setHorizontalHeaderLabels([
            'Run', 'Status', 'Time(s)', 'Plan(m)',
            'Actual(m)', 'AvgDev(m)', 'MaxDev(m)', 'Ratio',
            'Recov', 'Stalls',
        ])
        self._metrics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._metrics_table.setSelectionBehavior(QTableWidget.SelectRows)
        self._metrics_table.setSelectionMode(QTableWidget.SingleSelection)
        self._metrics_table.setMaximumHeight(200)
        self._metrics_table.cellClicked.connect(self._on_run_selected)
        metrics_lay.addWidget(self._metrics_table)
        right_layout.addWidget(metrics_group)

        # --- Event Timeline ---
        event_group = QGroupBox('Event Timeline')
        event_lay = QVBoxLayout(event_group)
        self._event_log = QTextEdit()
        self._event_log.setReadOnly(True)
        self._event_log.setFont(QFont('Monospace', 8))
        self._event_log.setStyleSheet('background-color: #1e1e1e; color: #d4d4d4;')
        event_lay.addWidget(self._event_log)
        right_layout.addWidget(event_group)

        # --- Status bar ---
        self.statusBar().showMessage('Ready — load a map and add waypoints')

    # ------------------------------------------------------------------
    # Launch navigation
    # ------------------------------------------------------------------
    def _launch_nav(self):
        NAV_SCRIPT = '/home/ros/Code/Demo8/start_nav_new_map_20260310_0918.sh'

        if not os.path.isfile(NAV_SCRIPT):
            QMessageBox.warning(self, 'Error',
                                f'Nav script not found:\n{NAV_SCRIPT}')
            return

        if self._nav_process and self._nav_process.poll() is None:
            QMessageBox.information(self, 'Info', 'Nav stack already running.')
            return

        reply = QMessageBox.question(
            self, 'Launch Nav',
            'Start chassis + LiDAR + D455 + Nav2 (PGO map)?\n\n'
            'This runs start_nav_new_map_20260310_0918.sh',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        try:
            self._nav_process = subprocess.Popen(
                ['bash', NAV_SCRIPT],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                universal_newlines=True,
                preexec_fn=os.setsid,
            )
            self._btn_launch_nav.setText('Nav Running')
            self._btn_launch_nav.setStyleSheet(
                'background-color: #7F8C8D; color: white; font-weight: bold;')

            # Feed script output into event log
            from ..log_handler import LogReaderThread
            self._nav_log_thread = LogReaderThread(self._nav_process, 'nav')
            self._nav_log_thread.log_received.connect(
                lambda _lvl, msg: self._log_event(f'[NAV] {msg}'))
            self._nav_log_thread.start()

            self._log_event(f'[NAV] Script launched (PID {self._nav_process.pid})')
            self.statusBar().showMessage('Nav stack launching...')

            # Poll for process exit to reset button
            self._nav_poll_timer = QTimer()
            self._nav_poll_timer.timeout.connect(self._poll_nav_process)
            self._nav_poll_timer.start(2000)

        except Exception as e:
            QMessageBox.critical(self, 'Launch Failed', str(e))

    def _poll_nav_process(self):
        if self._nav_process and self._nav_process.poll() is not None:
            self._nav_poll_timer.stop()
            code = self._nav_process.returncode
            self._log_event(f'[NAV] Script exited (code {code})')
            self._btn_launch_nav.setText('Launch Nav')
            self._btn_launch_nav.setStyleSheet(
                'background-color: #1ABC9C; color: white; font-weight: bold;')

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------
    def _load_map(self):
        maps_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__)))),
            'maps')
        path, _ = QFileDialog.getOpenFileName(
            self, 'Select 2D Map YAML', maps_dir, 'YAML (*.yaml *.yml)')
        if path:
            if self._map_widget.load_map(path):
                name = os.path.basename(path)
                self._lbl_map.setText(name)
                self._lbl_map.setStyleSheet('color: #2ecc71;')
                self.statusBar().showMessage(f'Map loaded: {name}')
                self._log_event(f'[UI] Manual map load: {path}')
            else:
                QMessageBox.warning(self, 'Error', f'Failed to load map:\n{path}')
                self._log_event(f'[UI] Map load failed: {path}')

    def _refresh_pose(self):
        self._engine.request_pose_refresh()
        self._log_event('[UI] Refresh Pose requested')
        self.statusBar().showMessage('Refreshing pose from TF...')

    def _center_robot(self):
        if self._map_widget.center_on_robot():
            self._log_event('[UI] Centered view on robot')
            self.statusBar().showMessage('Centered view on robot')
        else:
            self._log_event('[UI] Center robot requested, but no robot pose is available')
            self.statusBar().showMessage('No robot pose available to center on')

    def _toggle_pose_estimate(self):
        if self._btn_pose_estimate.isChecked():
            self._map_widget.set_mode('pose_estimate')
            self.statusBar().showMessage(
                '2D Pose Estimate: click on map to set position, drag to set orientation')
            self._log_event('[UI] 2D Pose Estimate mode activated')
        else:
            self._map_widget.set_mode('waypoint')
            self.statusBar().showMessage('Waypoint mode')

    @pyqtSlot(float, float, float)
    def _on_pose_estimate(self, x, y, yaw):
        self._btn_pose_estimate.setChecked(False)
        self._map_widget.set_pose_estimate_marker(x, y, yaw, 'pending')
        self._engine.publish_initial_pose(x, y, yaw)
        self._log_event(
            f'[UI] 2D Pose Estimate sent: ({x:.3f}, {y:.3f}, '
            f'yaw={math.degrees(yaw):.1f}°)')
        self.statusBar().showMessage(
            f'Initial pose published — verifying...')

    def _auto_load_map(self):
        """Auto-load the best map YAML from maps/ directory (recursive).

        Priority:
          1. Files named 'map.yaml' or 'map.yml' — pick the most recent
          2. Otherwise fall back to the most recently modified .yaml/.yml
        """
        maps_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__)))),
            'maps')
        if not os.path.isdir(maps_dir):
            return

        # Recursive scan
        yamls = []
        for root, _dirs, files in os.walk(maps_dir):
            for f in files:
                if f.endswith(('.yaml', '.yml')):
                    yamls.append(os.path.join(root, f))
        if not yamls:
            return

        # Prefer files named exactly "map.yaml" / "map.yml"
        preferred = [y for y in yamls
                     if os.path.basename(y).lower() in ('map.yaml', 'map.yml')]
        candidates = preferred if preferred else yamls

        # Among candidates, pick most recently modified
        best = max(candidates, key=os.path.getmtime)
        if self._map_widget.load_map(best):
            # Show relative path from maps/ for clarity
            rel = os.path.relpath(best, maps_dir)
            self._lbl_map.setText(rel)
            self._lbl_map.setStyleSheet('color: #2ecc71;')
            self.statusBar().showMessage(f'Auto-loaded map: {rel}')
            self._log_event(f'[UI] Auto-loaded map: {best}')

    # ------------------------------------------------------------------
    # Waypoint table
    # ------------------------------------------------------------------
    def _refresh_wp_table(self):
        wps = self._map_widget.waypoints
        self._wp_table.setRowCount(len(wps))
        for i, wp in enumerate(wps):
            enabled_item = self._wp_table.item(i, 0)
            checked = Qt.Checked
            if enabled_item is not None:
                checked = enabled_item.checkState()
            enabled_item = QTableWidgetItem()
            enabled_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            enabled_item.setCheckState(checked)
            enabled_item.setTextAlignment(Qt.AlignCenter)
            self._wp_table.setItem(i, 0, enabled_item)

            self._wp_table.setItem(i, 1, QTableWidgetItem(wp.label))
            self._wp_table.setItem(i, 2, QTableWidgetItem(f'{wp.x:.3f}'))
            self._wp_table.setItem(i, 3, QTableWidgetItem(f'{wp.y:.3f}'))
            self._wp_table.setItem(i, 4, QTableWidgetItem(f'{math.degrees(wp.yaw):.1f}'))

    def _clear_waypoints(self):
        self._map_widget.clear_waypoints()
        self._log_event('[UI] Cleared all waypoints')

    def _enabled_waypoints(self) -> List[Waypoint]:
        enabled = []
        for i, wp in enumerate(self._map_widget.waypoints):
            item = self._wp_table.item(i, 0)
            if item is None or item.checkState() == Qt.Checked:
                enabled.append(wp)
        return enabled

    # ------------------------------------------------------------------
    # Test execution
    # ------------------------------------------------------------------
    def _start_test(self):
        wps = self._enabled_waypoints()
        if len(wps) < 1:
            QMessageBox.information(self, 'Info',
                                    'Add at least 1 waypoint on the map.')
            return

        goals = [Pose2D(wp.x, wp.y, wp.yaw) for wp in wps]
        cycles = self._spin_cycles.value()
        if self._chk_loop.isChecked():
            cycles = 9999  # effectively infinite

        self._map_widget.clear_trajectories()
        self._metrics_table.setRowCount(0)
        self._event_log.clear()
        self._engine.records.clear()
        self._replay_run_index = -1

        self._btn_start.setEnabled(False)
        self._btn_stop.setEnabled(True)
        labels = ','.join(wp.label for wp in wps)
        mode = 'infinite loop' if self._chk_loop.isChecked() else f'{cycles} cycle(s)'
        self._log_event(f'[UI] Start patrol: waypoints=[{labels}] mode={mode}')
        self.statusBar().showMessage(f'Starting patrol [{labels}] — {mode}')
        self._engine.start_test(goals, num_cycles=cycles)

    def _stop_test(self):
        self._engine.request_stop_test()
        self._btn_stop.setEnabled(False)
        self._log_event('[UI] Stop requested')
        self.statusBar().showMessage('Stopping test...')

    def _clear_trajectories(self):
        self._map_widget.clear_trajectories()
        self._metrics_table.setRowCount(0)
        self._log_event('[UI] Cleared trajectory overlays and metrics table')

    # ------------------------------------------------------------------
    # Engine signal handlers
    # ------------------------------------------------------------------
    @pyqtSlot()
    def _on_engine_ready(self):
        self.statusBar().showMessage('Engine ready — monitoring')
        self._log_event('Engine started — ROS2 connected')

    @pyqtSlot()
    def _on_engine_stopped(self):
        self._btn_start.setEnabled(False)
        self._btn_stop.setEnabled(False)
        self.statusBar().showMessage('Engine stopped')
        self._log_event('Engine stopped')

    @pyqtSlot()
    def _on_test_batch_finished(self):
        self._btn_start.setEnabled(True)
        self._btn_stop.setEnabled(False)
        # Auto-save results
        path = self._save_results(auto=True)
        if path:
            self._log_event(f'Test batch finished — saved to {os.path.basename(path)}')
            self.statusBar().showMessage(f'Test complete — saved {os.path.basename(path)}')
        else:
            self._log_event('Test batch finished')
            self.statusBar().showMessage('Test complete — monitoring')

    @pyqtSlot(float, float, float)
    def _on_odom(self, x, y, yaw):
        self._map_widget.set_robot_pose(x, y, yaw)
        if self._engine.is_testing():
            self._map_widget.append_actual_pose(x, y)

    @pyqtSlot(list)
    def _on_global_plan(self, pts):
        self._map_widget.set_global_plan(pts)

    @pyqtSlot(list)
    def _on_local_plan(self, pts):
        self._map_widget.set_local_plan(pts)

    @pyqtSlot(list)
    def _on_theoretical(self, pts):
        self._map_widget.set_theoretical_path(pts)

    @pyqtSlot(int, object)
    def _on_run_started(self, idx, record: RunRecord):
        self._current_run_index = idx
        goal = record.goal
        self._log_event(f'Run {idx + 1} started → ({goal.x:.2f}, {goal.y:.2f})')
        self._map_widget.set_actual_trajectory([])

    @pyqtSlot(int, object)
    def _on_run_finished(self, idx, record: RunRecord):
        self._log_event(
            f'Run {idx + 1} {record.status} — '
            f'{record.duration:.1f}s, actual={record.actual_length:.2f}m, '
            f'planned={record.theoretical_length:.2f}m, '
            f'avg_dev={record.mean_lateral_deviation:.3f}m')

        # Add to metrics table
        row = self._metrics_table.rowCount()
        self._metrics_table.insertRow(row)
        planned = record.theoretical_length
        actual = record.actual_length
        ratio = actual / planned if planned > 0.01 else 0.0

        vals = [
            str(idx + 1),
            record.status,
            f'{record.duration:.1f}',
            f'{planned:.2f}',
            f'{actual:.2f}',
            f'{record.mean_lateral_deviation:.3f}',
            f'{record.max_lateral_deviation:.3f}',
            f'{ratio:.2f}',
            str(record.recovery_count),
            str(record.stall_count),
        ]
        for col, v in enumerate(vals):
            item = QTableWidgetItem(v)
            item.setTextAlignment(Qt.AlignCenter)
            # Color code status
            if col == 1:
                if record.status == 'succeeded':
                    item.setForeground(QColor(46, 204, 113))
                elif record.status == 'failed':
                    item.setForeground(QColor(231, 76, 60))
                else:
                    item.setForeground(QColor(241, 196, 15))
            self._metrics_table.setItem(row, col, item)

    @pyqtSlot(float, str)
    def _on_event(self, ts, msg):
        self._log_event(msg)

    @pyqtSlot(bool, float, float, float)
    def _on_pose_refresh_result(self, ok, x, y, yaw):
        if ok:
            self._map_widget.set_robot_pose(x, y, yaw)
            self._log_event(f'[UI] Refresh Pose succeeded: ({x:.2f}, {y:.2f})')
            self.statusBar().showMessage('Pose refresh succeeded')
        else:
            self._log_event('[UI] Refresh Pose failed: no map pose available yet')
            self.statusBar().showMessage('No map pose available yet')

    @pyqtSlot(bool, float, float, float, float, float, float, float)
    def _on_initial_pose_verified(self, ok, pub_x, pub_y, pub_yaw,
                                   act_x, act_y, act_yaw, offset):
        if offset < 0:
            # TF unavailable
            self._map_widget.set_pose_estimate_marker(pub_x, pub_y, pub_yaw, 'warn')
            self._log_event('[UI] 2D Pose Estimate: TF verification failed — no pose available')
            self.statusBar().showMessage('Pose estimate: TF not available for verification')
        elif ok:
            self._map_widget.set_pose_estimate_marker(pub_x, pub_y, pub_yaw, 'ok')
            self._log_event(
                f'[UI] 2D Pose Estimate OK — TF: ({act_x:.3f}, {act_y:.3f}, '
                f'yaw={math.degrees(act_yaw):.1f}°) offset={offset:.3f}m')
            self.statusBar().showMessage(
                f'Pose estimate OK (offset={offset:.3f}m)')
        else:
            self._map_widget.set_pose_estimate_marker(pub_x, pub_y, pub_yaw, 'warn')
            self._log_event(
                f'[UI] 2D Pose Estimate OFFSET — TF: ({act_x:.3f}, {act_y:.3f}, '
                f'yaw={math.degrees(act_yaw):.1f}°) offset={offset:.3f}m')
            self.statusBar().showMessage(
                f'Pose estimate: large offset={offset:.3f}m — check localization')
        # Auto-clear marker after 5 seconds
        QTimer.singleShot(5000, self._map_widget.clear_pose_estimate_marker)

    def _log_event(self, msg: str):
        t = time.strftime('%H:%M:%S')
        self._event_log.append(f'<span style="color:#888">[{t}]</span> {msg}')
        sb = self._event_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ------------------------------------------------------------------
    # Run replay
    # ------------------------------------------------------------------
    @pyqtSlot(int, int)
    def _on_run_selected(self, row, _col):
        """Show a completed run's trajectories on the map."""
        records = self._engine.records
        if row < 0 or row >= len(records):
            return
        record = records[row]
        self._replay_run_index = row
        self._map_widget.set_theoretical_path(
            [(p.x, p.y) for p in record.theoretical_path])
        self._map_widget.set_global_plan(
            [(p.x, p.y) for p in record.global_plan])
        self._map_widget.set_local_plan(
            [(p.x, p.y) for p in record.local_plan])
        self._map_widget.set_actual_trajectory(
            [(p.x, p.y) for p in record.actual_trajectory])
        self._log_event(f'[UI] Replay selected: run={row + 1}')
        self.statusBar().showMessage(
            f'Replay: Run {row + 1} — {record.status}, '
            f'{record.duration:.1f}s, dev={record.mean_lateral_deviation:.3f}m')

    # ------------------------------------------------------------------
    # Save / export
    # ------------------------------------------------------------------
    def _save_results(self, auto=False):
        """Save all run records to a timestamped JSON file.

        Returns the saved file path, or None on failure / no data.
        """
        records = self._engine.records
        if not records:
            if not auto:
                QMessageBox.information(self, 'Info', 'No test results to save.')
            return None

        # Build output directory
        results_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__)))),
            'test_results')
        os.makedirs(results_dir, exist_ok=True)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'nav_test_{stamp}.json'
        filepath = os.path.join(results_dir, filename)

        # Waypoints
        wps = self._map_widget.waypoints
        wp_list = [{'label': wp.label, 'x': wp.x, 'y': wp.y,
                     'yaw': wp.yaw} for wp in wps]

        data = {
            'timestamp': stamp,
            'num_cycles': self._spin_cycles.value(),
            'waypoints': wp_list,
            'runs': [r.to_dict() for r in records],
            'summary': {
                'total_runs': len(records),
                'succeeded': sum(1 for r in records if r.status == 'succeeded'),
                'failed': sum(1 for r in records if r.status == 'failed'),
                'canceled': sum(1 for r in records if r.status == 'canceled'),
                'total_distance': sum(r.actual_length for r in records),
                'total_time': sum(r.duration for r in records),
                'avg_deviation': (
                    sum(r.mean_lateral_deviation for r in records) / len(records)
                    if records else 0.0),
                'total_recoveries': sum(r.recovery_count for r in records),
                'total_aborts': sum(r.abort_count for r in records),
                'total_stalls': sum(r.stall_count for r in records),
            },
        }

        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self._log_event(f'[UI] Results saved: {filepath}')
            if not auto:
                self.statusBar().showMessage(f'Saved: {filepath}')
            return filepath
        except Exception as e:
            self._log_event(f'Save failed: {e}')
            if not auto:
                QMessageBox.warning(self, 'Save Error', str(e))
            return None

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def closeEvent(self, event):
        # Stop nav script process group
        if self._nav_process and self._nav_process.poll() is None:
            try:
                os.killpg(os.getpgid(self._nav_process.pid), signal.SIGTERM)
            except Exception:
                self._nav_process.terminate()
        if self._engine.isRunning():
            self._engine.request_shutdown()
            self._engine.wait(5000)
        super().closeEvent(event)
