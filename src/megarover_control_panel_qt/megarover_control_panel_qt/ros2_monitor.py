#!/usr/bin/env python3
"""
ROS2 monitor for checking topics and nodes.
"""

from PyQt5.QtCore import QThread, pyqtSignal
import time


class ROS2Monitor(QThread):
    """Monitor ROS2 topics and nodes in a separate thread."""

    # Signals
    topic_status_changed = pyqtSignal(str, bool, float)  # (topic, is_active, hz)
    node_status_changed = pyqtSignal(str, bool)          # (node, is_active)

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.running = True
        self.node = None

    def run(self):
        """Main thread loop - MUST initialize rclpy here, not in __init__."""
        try:
            import rclpy
            from rclpy.node import Node

            # Initialize rclpy in this thread
            rclpy.init()
            self.node = rclpy.create_node('control_panel_monitor')

            # Main monitoring loop (1 Hz)
            while self.running:
                try:
                    # Check topics
                    self._check_topics()

                    # Check nodes
                    self._check_nodes()

                    # Sleep for 1 second
                    time.sleep(1.0)

                except Exception as e:
                    print(f'ROS2Monitor loop error: {e}')
                    time.sleep(1.0)

        except Exception as e:
            print(f'ROS2Monitor initialization error: {e}')

        finally:
            # Clean up
            if self.node:
                self.node.destroy_node()

            try:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except:
                pass

    def _check_topics(self):
        """Check if expected topics are being published."""
        if not self.node:
            return

        try:
            # Get all available topics
            topic_list = self.node.get_topic_names_and_types()
            topic_names = [name for name, _ in topic_list]

            # Check each component's topics
            for component_id, component in self.config['components'].items():
                topics_to_check = component.get('check_topics', [])

                for topic_name in topics_to_check:
                    is_active = topic_name in topic_names

                    # TODO: Calculate actual message rate
                    # For now, just report 0 Hz if inactive, 1.0 Hz if active
                    hz = 1.0 if is_active else 0.0

                    self.topic_status_changed.emit(topic_name, is_active, hz)

        except Exception as e:
            print(f'Error checking topics: {e}')

    def _check_nodes(self):
        """Check if expected nodes are running."""
        if not self.node:
            return

        try:
            # Get all running nodes
            node_names = self.node.get_node_names()

            # Check each component's nodes
            for component_id, component in self.config['components'].items():
                nodes_to_check = component.get('check_nodes', [])

                for node_name in nodes_to_check:
                    is_active = node_name in node_names
                    self.node_status_changed.emit(node_name, is_active)

        except Exception as e:
            print(f'Error checking nodes: {e}')

    def stop(self):
        """Stop the monitoring thread."""
        self.running = False


class ComponentStatusAggregator:
    """
    Aggregates status from process manager and ROS2 monitor
    to determine overall component health.
    """

    def __init__(self):
        self.process_status = {}  # component_id -> 'running' | 'stopped'
        self.topic_status = {}    # topic_name -> bool
        self.node_status = {}     # node_name -> bool

    def update_process_status(self, component_id, status):
        """Update process status."""
        self.process_status[component_id] = status

    def update_topic_status(self, topic_name, is_active):
        """Update topic status."""
        self.topic_status[topic_name] = is_active

    def update_node_status(self, node_name, is_active):
        """Update node status."""
        self.node_status[node_name] = is_active

    def get_component_status(self, component_id, component_config):
        """
        Get overall status for a component.

        Returns:
            str: 'running' (green), 'partial' (yellow), 'stopped' (gray), 'error' (red)
        """
        # Check if process is running
        if self.process_status.get(component_id) != 'running':
            return 'stopped'

        # Process is running, check ROS2 health
        expected_topics = component_config.get('check_topics', [])
        expected_nodes = component_config.get('check_nodes', [])

        # If no ROS2 checks defined, just return process status
        if not expected_topics and not expected_nodes:
            return 'running'

        # Check topics
        topics_ok = True
        for topic in expected_topics:
            if not self.topic_status.get(topic, False):
                topics_ok = False
                break

        # Check nodes
        nodes_ok = True
        for node in expected_nodes:
            if not self.node_status.get(node, False):
                nodes_ok = False
                break

        # Determine overall status
        if topics_ok and nodes_ok:
            return 'running'  # All good
        elif expected_topics and not topics_ok:
            return 'partial'  # Process running but topics not publishing
        elif expected_nodes and not nodes_ok:
            return 'partial'  # Process running but nodes not found
        else:
            return 'running'  # Default to running if checks pass
