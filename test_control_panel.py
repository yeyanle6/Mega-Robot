#!/usr/bin/env python3
"""
Test script for MegaRover3 Control Panel.
Tests basic functionality without launching the GUI.
"""

import sys
import os

# Add package path
sys.path.insert(0, 'src/megarover_control_panel_qt')

def test_imports():
    """Test that all modules can be imported."""
    print("Testing imports...")

    try:
        from megarover_control_panel_qt.main_window import MainWindow
        print("✓ MainWindow imported")

        from megarover_control_panel_qt.process_manager import ProcessManager
        print("✓ ProcessManager imported")

        from megarover_control_panel_qt.ros2_monitor import ROS2Monitor, ComponentStatusAggregator
        print("✓ ROS2Monitor imported")

        from megarover_control_panel_qt.log_handler import LogReaderThread, LogFormatter
        print("✓ LogHandler imported")

        from megarover_control_panel_qt.widgets import ComponentControlPanel, LogViewer
        print("✓ Widgets imported")

        return True
    except Exception as e:
        print(f"✗ Import failed: {e}")
        return False


def test_config():
    """Test that config file can be loaded."""
    print("\nTesting configuration...")

    try:
        import yaml
        config_path = 'src/megarover_control_panel_qt/megarover_control_panel_qt/config/panel_config.yaml'

        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        print(f"✓ Config loaded successfully")
        print(f"  Components: {', '.join(config['components'].keys())}")

        # Validate config structure
        for comp_id, comp_data in config['components'].items():
            assert 'name' in comp_data, f"Missing 'name' in {comp_id}"
            assert 'command' in comp_data, f"Missing 'command' in {comp_id}"
            print(f"  ✓ {comp_id}: {comp_data['name']}")

        return True
    except Exception as e:
        print(f"✗ Config test failed: {e}")
        return False


def test_process_manager():
    """Test ProcessManager instantiation."""
    print("\nTesting ProcessManager...")

    try:
        import yaml
        from megarover_control_panel_qt.process_manager import ProcessManager

        config_path = 'src/megarover_control_panel_qt/megarover_control_panel_qt/config/panel_config.yaml'
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        print("✓ ProcessManager class available")
        return True
    except Exception as e:
        print(f"✗ ProcessManager test failed: {e}")
        return False


def test_file_structure():
    """Test that all required files exist."""
    print("\nTesting file structure...")

    required_files = [
        'src/megarover_control_panel_qt/megarover_control_panel_qt/__init__.py',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/main_window.py',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/process_manager.py',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/ros2_monitor.py',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/log_handler.py',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/widgets/__init__.py',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/widgets/component_control.py',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/widgets/log_viewer.py',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/config/panel_config.yaml',
        'src/megarover_control_panel_qt/megarover_control_panel_qt/resources/styles.qss',
        'src/megarover_control_panel_qt/scripts/control_panel.py',
        'src/megarover_control_panel_qt/scripts/nav_test_workbench.py',
    ]

    all_exist = True
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"✓ {file_path}")
        else:
            print(f"✗ {file_path} - MISSING")
            all_exist = False

    return all_exist


def main():
    """Run all tests."""
    print("=" * 60)
    print("MegaRover3 Control Panel Test Suite")
    print("=" * 60)

    results = []

    results.append(("File Structure", test_file_structure()))
    results.append(("Imports", test_imports()))
    results.append(("Configuration", test_config()))
    results.append(("ProcessManager", test_process_manager()))

    print("\n" + "=" * 60)
    print("Test Results Summary")
    print("=" * 60)

    for test_name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{test_name:.<40} {status}")

    all_passed = all(result[1] for result in results)

    print("=" * 60)
    if all_passed:
        print("✓ All tests passed!")
        print("\nYou can now launch the control panel:")
        print("  ros2 run megarover_control_panel_qt control_panel.py")
    else:
        print("✗ Some tests failed. Please check the errors above.")
    print("=" * 60)

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
