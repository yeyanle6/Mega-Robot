#!/usr/bin/env python3
"""
Process manager for controlling ROS2 components.
"""

import subprocess
import os
import signal
import time
import psutil
from PyQt5.QtCore import QObject, pyqtSignal
from .log_handler import LogReaderThread


class ProcessManager(QObject):
    """Manages subprocess lifecycle for ROS2 components."""

    # Signals
    log_received = pyqtSignal(str, str, str)  # (component_id, level, message)
    status_changed = pyqtSignal(str, str)     # (component_id, status)

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.processes = {}      # component_id -> subprocess.Popen
        self.log_threads = {}    # component_id -> LogReaderThread

    def start_component(self, component_id):
        """
        Start a ROS2 component.

        Args:
            component_id: Component identifier from config

        Returns:
            bool: True if started successfully, False otherwise
        """
        if component_id not in self.config['components']:
            self.log_received.emit(component_id, 'ERROR', f'Unknown component: {component_id}')
            return False

        if self.is_running(component_id):
            self.log_received.emit(component_id, 'WARN', f'Component already running: {component_id}')
            return False

        component = self.config['components'][component_id]

        # Check dependencies
        for dep in component.get('dependencies', []):
            if not self.is_running(dep):
                dep_name = self.config['components'][dep]['name']
                self.log_received.emit(
                    component_id,
                    'ERROR',
                    f'Dependency not running: {dep_name}. Please start it first.'
                )
                return False

        # Get command
        command = component['command']

        # Handle parameters (e.g., map path for navigation)
        if 'params' in component:
            for param_key, param_value in component['params'].items():
                placeholder = '{' + param_key + '}'
                if placeholder in command:
                    if not param_value:
                        self.log_received.emit(
                            component_id,
                            'ERROR',
                            f'Missing parameter: {param_key}'
                        )
                        return False
                    command = command.replace(placeholder, param_value)

        try:
            self.log_received.emit(component_id, 'INFO', f'Starting: {command}')

            # Start process with new process group
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                universal_newlines=True,
                preexec_fn=os.setsid  # Create new process group
            )

            self.processes[component_id] = process

            # Start log reader thread
            log_thread = LogReaderThread(process, component_id)
            log_thread.log_received.connect(
                lambda level, msg: self.log_received.emit(component_id, level, msg)
            )
            log_thread.start()
            self.log_threads[component_id] = log_thread

            self.status_changed.emit(component_id, 'running')
            self.log_received.emit(component_id, 'INFO', f'Started successfully (PID: {process.pid})')

            return True

        except Exception as e:
            self.log_received.emit(component_id, 'ERROR', f'Failed to start: {str(e)}')
            self.status_changed.emit(component_id, 'stopped')
            return False

    def stop_component(self, component_id):
        """
        Stop a ROS2 component.

        Args:
            component_id: Component identifier

        Returns:
            bool: True if stopped successfully, False otherwise
        """
        if not self.is_running(component_id):
            self.log_received.emit(component_id, 'WARN', 'Component not running')
            return False

        try:
            process = self.processes[component_id]
            pgid = os.getpgid(process.pid)

            self.log_received.emit(component_id, 'INFO', f'Stopping process (PID: {process.pid})...')

            # Send SIGTERM to process group
            os.killpg(pgid, signal.SIGTERM)

            # Wait for graceful shutdown (up to 10 seconds)
            for i in range(10):
                if process.poll() is not None:
                    self.log_received.emit(component_id, 'INFO', 'Stopped successfully')
                    break
                time.sleep(1)
            else:
                # Force kill if still running
                self.log_received.emit(component_id, 'WARN', 'Forcing kill...')
                os.killpg(pgid, signal.SIGKILL)
                process.wait(timeout=2)
                self.log_received.emit(component_id, 'INFO', 'Force killed')

            # Stop log thread
            if component_id in self.log_threads:
                self.log_threads[component_id].stop()
                self.log_threads[component_id].wait(2000)
                del self.log_threads[component_id]

            del self.processes[component_id]
            self.status_changed.emit(component_id, 'stopped')

            return True

        except Exception as e:
            self.log_received.emit(component_id, 'ERROR', f'Failed to stop: {str(e)}')
            return False

    def is_running(self, component_id):
        """
        Check if a component is running.

        Args:
            component_id: Component identifier

        Returns:
            bool: True if running, False otherwise
        """
        if component_id not in self.processes:
            return False

        process = self.processes[component_id]
        return process.poll() is None

    def stop_all(self):
        """Stop all running components."""
        for component_id in list(self.processes.keys()):
            self.stop_component(component_id)

    def get_process_pid(self, component_id):
        """Get PID of a running component."""
        if component_id in self.processes:
            return self.processes[component_id].pid
        return None

    def monitor_health(self):
        """
        Monitor health of all running processes.
        Should be called periodically.
        """
        for component_id in list(self.processes.keys()):
            process = self.processes[component_id]

            # Check if process died unexpectedly
            if process.poll() is not None:
                return_code = process.returncode
                self.log_received.emit(
                    component_id,
                    'ERROR',
                    f'Process terminated unexpectedly (exit code: {return_code})'
                )

                # Clean up
                if component_id in self.log_threads:
                    self.log_threads[component_id].stop()
                    self.log_threads[component_id].wait(2000)
                    del self.log_threads[component_id]

                del self.processes[component_id]
                self.status_changed.emit(component_id, 'stopped')

    def set_param(self, component_id, param_key, param_value):
        """
        Set a parameter for a component.

        Args:
            component_id: Component identifier
            param_key: Parameter key
            param_value: Parameter value
        """
        if component_id in self.config['components']:
            if 'params' not in self.config['components'][component_id]:
                self.config['components'][component_id]['params'] = {}
            self.config['components'][component_id]['params'][param_key] = param_value

    def kill_all_ros_processes(self):
        """Nuclear option: kill all ROS2 processes."""
        try:
            self.log_received.emit('system', 'INFO', 'Killing all ROS2 processes...')

            # First stop all managed processes
            self.stop_all()

            # Then kill any remaining ros2 processes
            killed_count = 0
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    if 'ros2' in cmdline or 'ros' in proc.info['name']:
                        proc.kill()
                        killed_count += 1
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            self.log_received.emit('system', 'INFO', f'Killed {killed_count} ROS2 processes')
            return True

        except Exception as e:
            self.log_received.emit('system', 'ERROR', f'Failed to kill processes: {str(e)}')
            return False
