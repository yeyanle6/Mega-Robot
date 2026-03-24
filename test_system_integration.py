#!/usr/bin/env python3
"""
MegaRover3 Control Panel Integration Test Suite
测试所有功能模块是否正常工作
"""

import sys
import os
import yaml
import subprocess
from pathlib import Path

# 测试计数器
tests_passed = 0
tests_failed = 0
test_results = []

def test_result(name, passed, message=""):
    """记录测试结果"""
    global tests_passed, tests_failed, test_results
    if passed:
        tests_passed += 1
        status = "✓ PASS"
        color = "\033[92m"
    else:
        tests_failed += 1
        status = "✗ FAIL"
        color = "\033[91m"

    reset = "\033[0m"
    result = f"{color}{status}{reset} - {name}"
    if message:
        result += f"\n      {message}"

    test_results.append((name, passed, message))
    print(result)
    return passed


print("=" * 70)
print("MegaRover3 Control Panel - Integration Test Suite")
print("=" * 70)
print()

# ============================================================
# 测试 1: 文件结构完整性
# ============================================================
print("【测试组 1】文件结构完整性")
print("-" * 70)

required_files = [
    'src/megarover3_ros2/megarover3_navigation/gui/__init__.py',
    'src/megarover3_ros2/megarover3_navigation/gui/main_window.py',
    'src/megarover3_ros2/megarover3_navigation/gui/process_manager.py',
    'src/megarover3_ros2/megarover3_navigation/gui/ros2_monitor.py',
    'src/megarover3_ros2/megarover3_navigation/gui/log_handler.py',
    'src/megarover3_ros2/megarover3_navigation/gui/widgets/__init__.py',
    'src/megarover3_ros2/megarover3_navigation/gui/widgets/component_control.py',
    'src/megarover3_ros2/megarover3_navigation/gui/widgets/log_viewer.py',
    'src/megarover3_ros2/megarover3_navigation/gui/config/panel_config.yaml',
    'src/megarover3_ros2/megarover3_navigation/gui/resources/styles.qss',
    'src/megarover3_ros2/megarover3_navigation/scripts/control_panel.py',
    'src/megarover3_ros2/megarover3_navigation/launch/fastlio2_navigation.launch.py',
    'src/megarover3_ros2/megarover3_navigation/launch/fastlio2_pgo_navigation.launch.py',
]

for file_path in required_files:
    exists = os.path.exists(file_path)
    test_result(f"文件存在: {os.path.basename(file_path)}", exists,
                file_path if exists else f"缺失: {file_path}")

print()

# ============================================================
# 测试 2: Python 模块导入
# ============================================================
print("【测试组 2】Python 模块导入")
print("-" * 70)

sys.path.insert(0, 'src/megarover3_ros2/megarover3_navigation')

try:
    from PyQt5.QtWidgets import QApplication
    test_result("PyQt5 导入", True)
except Exception as e:
    test_result("PyQt5 导入", False, str(e))

try:
    import yaml
    test_result("PyYAML 导入", True)
except Exception as e:
    test_result("PyYAML 导入", False, str(e))

try:
    import psutil
    test_result("psutil 导入", True)
except Exception as e:
    test_result("psutil 导入", False, str(e))

try:
    from gui.main_window import MainWindow
    test_result("MainWindow 导入", True)
except Exception as e:
    test_result("MainWindow 导入", False, str(e))

try:
    from gui.process_manager import ProcessManager
    test_result("ProcessManager 导入", True)
except Exception as e:
    test_result("ProcessManager 导入", False, str(e))

try:
    from gui.ros2_monitor import ROS2Monitor, ComponentStatusAggregator
    test_result("ROS2Monitor 导入", True)
except Exception as e:
    test_result("ROS2Monitor 导入", False, str(e))

try:
    from gui.log_handler import LogReaderThread, LogFormatter
    test_result("LogHandler 导入", True)
except Exception as e:
    test_result("LogHandler 导入", False, str(e))

try:
    from gui.widgets import ComponentControlPanel, LogViewer
    test_result("Widgets 导入", True)
except Exception as e:
    test_result("Widgets 导入", False, str(e))

print()

# ============================================================
# 测试 3: 配置文件解析
# ============================================================
print("【测试组 3】配置文件解析")
print("-" * 70)

config_path = 'src/megarover3_ros2/megarover3_navigation/gui/config/panel_config.yaml'

try:
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    test_result("配置文件加载", True)

    # 验证配置结构
    if 'components' in config:
        test_result("配置文件包含 components", True)

        expected_components = ['chassis', 'lidar', 'd455_front', 'slam', 'slam_pgo',
                              'navigation', 'navigation_pgo', 'teleop']

        for comp_id in expected_components:
            if comp_id in config['components']:
                comp = config['components'][comp_id]
                has_name = 'name' in comp
                has_command = 'command' in comp
                has_deps = 'dependencies' in comp

                all_ok = has_name and has_command and has_deps
                test_result(f"组件 {comp_id} 配置完整", all_ok,
                           f"name={has_name}, command={has_command}, dependencies={has_deps}")
            else:
                test_result(f"组件 {comp_id} 存在", False, "缺失")

        actual_count = len(config['components'])
        expected_count = len(expected_components)
        test_result(f"组件数量正确 ({actual_count}/{expected_count})",
                   actual_count == expected_count)
    else:
        test_result("配置文件包含 components", False)

except Exception as e:
    test_result("配置文件加载", False, str(e))

print()

# ============================================================
# 测试 4: 安装文件完整性
# ============================================================
print("【测试组 4】安装文件完整性")
print("-" * 70)

install_files = [
    'install/megarover3_navigation/lib/megarover3_navigation/control_panel.py',
    'install/megarover3_navigation/share/megarover3_navigation/gui',
    'install/megarover3_navigation/share/megarover3_navigation/launch/fastlio2_navigation.launch.py',
    'install/megarover3_navigation/share/megarover3_navigation/launch/fastlio2_pgo_navigation.launch.py',
]

for file_path in install_files:
    exists = os.path.exists(file_path)
    test_result(f"安装: {os.path.basename(file_path)}", exists,
               file_path if exists else f"未安装: {file_path}")

print()

# ============================================================
# 测试 5: 快捷脚本
# ============================================================
print("【测试组 5】快捷脚本")
print("-" * 70)

panel_script = '/home/ros/megarover3_panel'
exists = os.path.exists(panel_script)
test_result("快捷脚本存在", exists)

if exists:
    is_executable = os.access(panel_script, os.X_OK)
    test_result("快捷脚本可执行", is_executable)

print()

# ============================================================
# 测试 6: ROS2 包依赖
# ============================================================
print("【测试组 6】ROS2 包依赖")
print("-" * 70)

try:
    result = subprocess.run(['ros2', 'pkg', 'list'],
                          capture_output=True, text=True, timeout=5)
    packages = result.stdout

    required_packages = [
        'megarover3_navigation',
        'realsense2_camera',
        'livox_ros_driver2',
        'fastlio2',
        'pgo',
        'localizer',
    ]

    for pkg in required_packages:
        found = pkg in packages
        test_result(f"ROS2 包: {pkg}", found)

except Exception as e:
    test_result("ROS2 包检查", False, str(e))

print()

# ============================================================
# 测试 7: Launch 文件语法
# ============================================================
print("【测试组 7】Launch 文件语法")
print("-" * 70)

launch_files = [
    'src/megarover3_ros2/megarover3_navigation/launch/fastlio2_navigation.launch.py',
    'src/megarover3_ros2/megarover3_navigation/launch/fastlio2_pgo_navigation.launch.py',
]

for launch_file in launch_files:
    try:
        with open(launch_file, 'r') as f:
            compile(f.read(), launch_file, 'exec')
        test_result(f"Launch 语法: {os.path.basename(launch_file)}", True)
    except SyntaxError as e:
        test_result(f"Launch 语法: {os.path.basename(launch_file)}", False, str(e))
    except Exception as e:
        test_result(f"Launch 语法: {os.path.basename(launch_file)}", False, str(e))

print()

# ============================================================
# 测试 8: 文档完整性
# ============================================================
print("【测试组 8】文档完整性")
print("-" * 70)

doc_files = [
    'PGO_MAPPING_GUIDE.md',
    'CONTROL_PANEL_README.md',
    'QUICK_START.md',
    'IMPLEMENTATION_SUMMARY.md',
]

for doc in doc_files:
    exists = os.path.exists(doc)
    test_result(f"文档: {doc}", exists)

print()

# ============================================================
# 测试 9: 地图保存脚本
# ============================================================
print("【测试组 9】地图保存脚本")
print("-" * 70)

map_saver = 'save_map_now.py'
exists = os.path.exists(map_saver)
test_result("地图保存脚本存在", exists)

if exists:
    is_executable = os.access(map_saver, os.X_OK)
    test_result("地图保存脚本可执行", is_executable)

    try:
        with open(map_saver, 'r') as f:
            compile(f.read(), map_saver, 'exec')
        test_result("地图保存脚本语法", True)
    except Exception as e:
        test_result("地图保存脚本语法", False, str(e))

print()

# ============================================================
# 测试 10: 相机库版本
# ============================================================
print("【测试组 10】相机库版本")
print("-" * 70)

try:
    result = subprocess.run(['ldconfig', '-p'],
                          capture_output=True, text=True, timeout=5)
    output = result.stdout

    # 检查是否还有老版本库
    has_old_version = '/usr/local/lib/librealsense2.so' in output
    test_result("老版本库已删除", not has_old_version,
               "检测到 /usr/local/lib/librealsense2.so" if has_old_version else "")

except Exception as e:
    test_result("库版本检查", False, str(e))

print()

# ============================================================
# 测试总结
# ============================================================
print("=" * 70)
print("测试总结")
print("=" * 70)

total_tests = tests_passed + tests_failed
pass_rate = (tests_passed / total_tests * 100) if total_tests > 0 else 0

print(f"\n总测试数: {total_tests}")
print(f"✓ 通过: {tests_passed}")
print(f"✗ 失败: {tests_failed}")
print(f"通过率: {pass_rate:.1f}%")

if tests_failed == 0:
    print("\n" + "=" * 70)
    print("🎉 所有测试通过！系统已准备就绪！")
    print("=" * 70)
    print("\n可以启动控制面板:")
    print("  ~/megarover3_panel")
else:
    print("\n" + "=" * 70)
    print("⚠️  有测试失败，请检查上述错误信息")
    print("=" * 70)
    print("\n失败的测试:")
    for name, passed, message in test_results:
        if not passed:
            print(f"  - {name}")
            if message:
                print(f"    {message}")

print("\n" + "=" * 70)

sys.exit(0 if tests_failed == 0 else 1)
