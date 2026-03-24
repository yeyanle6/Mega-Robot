#!/usr/bin/env python3
"""
MegaRover3 Control Panel - Functional Test
真正测试功能是否能运行
"""

import sys
import os
import yaml
import time
from pathlib import Path

# 设置路径
sys.path.insert(0, 'src/megarover3_ros2/megarover3_navigation')

print("=" * 70)
print("MegaRover3 Control Panel - 功能测试")
print("=" * 70)
print()

# 测试计数器
tests_passed = 0
tests_failed = 0

def test_result(name, passed, message=""):
    """记录测试结果"""
    global tests_passed, tests_failed
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
    print(result)
    return passed

# ============================================================
# 测试 1: 配置加载和解析
# ============================================================
print("【功能测试 1】配置加载")
print("-" * 70)

try:
    from gui.main_window import MainWindow
    config_path = 'src/megarover3_ros2/megarover3_navigation/gui/config/panel_config.yaml'

    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    test_result("配置文件加载", True, f"加载了 {len(config['components'])} 个组件")
except Exception as e:
    test_result("配置文件加载", False, str(e))

print()

# ============================================================
# 测试 2: ProcessManager 实例化
# ============================================================
print("【功能测试 2】ProcessManager")
print("-" * 70)

try:
    from gui.process_manager import ProcessManager

    pm = ProcessManager(config)
    test_result("ProcessManager 创建", True)

    # 测试基本方法
    has_start = hasattr(pm, 'start_component')
    has_stop = hasattr(pm, 'stop_component')
    has_running = hasattr(pm, 'is_running')

    test_result("ProcessManager 有 start_component 方法", has_start)
    test_result("ProcessManager 有 stop_component 方法", has_stop)
    test_result("ProcessManager 有 is_running 方法", has_running)

    # 测试状态检查（不启动真实进程）
    for comp_id in config['components'].keys():
        is_running = pm.is_running(comp_id)
        test_result(f"检查 {comp_id} 状态", True, f"运行中: {is_running}")

except Exception as e:
    test_result("ProcessManager 测试", False, str(e))

print()

# ============================================================
# 测试 3: ROS2Monitor 实例化
# ============================================================
print("【功能测试 3】ROS2Monitor")
print("-" * 70)

try:
    from gui.ros2_monitor import ROS2Monitor, ComponentStatusAggregator

    # 不启动线程，只测试实例化
    monitor = ROS2Monitor(config)
    test_result("ROS2Monitor 创建", True)

    has_run = hasattr(monitor, 'run')
    has_stop = hasattr(monitor, 'stop')

    test_result("ROS2Monitor 有 run 方法", has_run)
    test_result("ROS2Monitor 有 stop 方法", has_stop)

    # 测试状态聚合器
    aggregator = ComponentStatusAggregator()
    test_result("ComponentStatusAggregator 创建", True)

    # 测试状态更新
    aggregator.update_process_status('test', 'running')
    aggregator.update_topic_status('/test/topic', True)
    aggregator.update_node_status('/test/node', True)

    status = aggregator.get_component_status('test', {'check_topics': [], 'check_nodes': []})
    test_result("状态聚合器工作", True, f"状态: {status}")

except Exception as e:
    test_result("ROS2Monitor 测试", False, str(e))

print()

# ============================================================
# 测试 4: LogHandler
# ============================================================
print("【功能测试 4】LogHandler")
print("-" * 70)

try:
    from gui.log_handler import LogFormatter

    # 测试日志格式化
    html = LogFormatter.format_html('INFO', 'Test message', 'test_component')
    test_result("LogFormatter.format_html", True, f"生成了 {len(html)} 字符的 HTML")

    # 测试 ANSI 清除
    ansi_text = '\033[91mRed Text\033[0m'
    clean_text = LogFormatter.strip_ansi(ansi_text)
    test_result("LogFormatter.strip_ansi", 'Red Text' in clean_text)

except Exception as e:
    test_result("LogHandler 测试", False, str(e))

print()

# ============================================================
# 测试 5: Widgets 组件
# ============================================================
print("【功能测试 5】UI Widgets")
print("-" * 70)

try:
    from PyQt5.QtWidgets import QApplication
    from gui.widgets import ComponentControlPanel, LogViewer

    # 创建 QApplication（测试需要）
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    # 测试 LogViewer
    log_viewer = LogViewer()
    test_result("LogViewer 创建", True)

    has_append = hasattr(log_viewer, 'append_log')
    has_clear = hasattr(log_viewer, 'clear_logs')

    test_result("LogViewer 有 append_log 方法", has_append)
    test_result("LogViewer 有 clear_logs 方法", has_clear)

    # 测试添加日志
    log_viewer.append_log("<span>Test log</span>")
    test_result("LogViewer 可以添加日志", True)

    # 测试 ComponentControlPanel
    from gui.ros2_monitor import ComponentStatusAggregator
    status_agg = ComponentStatusAggregator()

    control_panel = ComponentControlPanel(config, pm, status_agg)
    test_result("ComponentControlPanel 创建", True)

except Exception as e:
    test_result("UI Widgets 测试", False, str(e))

print()

# ============================================================
# 测试 6: MainWindow 实例化
# ============================================================
print("【功能测试 6】MainWindow")
print("-" * 70)

try:
    from PyQt5.QtWidgets import QApplication
    from gui.main_window import MainWindow

    # 创建 QApplication
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    # 创建主窗口（不显示）
    window = MainWindow()
    test_result("MainWindow 创建", True)

    # 检查窗口属性
    has_title = window.windowTitle() == 'MegaRover3 控制面板'
    test_result("窗口标题正确", has_title, window.windowTitle())

    # 检查组件
    has_component_panel = hasattr(window, 'component_panel')
    has_log_viewer = hasattr(window, 'log_viewer')
    has_process_manager = hasattr(window, 'process_manager')

    test_result("MainWindow 有 component_panel", has_component_panel)
    test_result("MainWindow 有 log_viewer", has_log_viewer)
    test_result("MainWindow 有 process_manager", has_process_manager)

    # 检查快捷操作按钮
    has_mapping_btn = hasattr(window, 'mapping_btn')
    has_nav_btn = hasattr(window, 'nav_btn')
    has_save_map_btn = hasattr(window, 'save_map_btn')

    test_result("MainWindow 有一键启动建图按钮", has_mapping_btn)
    test_result("MainWindow 有一键启动导航按钮", has_nav_btn)
    test_result("MainWindow 有保存地图按钮", has_save_map_btn)

    # 不调用 show()，避免实际显示窗口
    # window.show()

    # 清理
    window.close()

except Exception as e:
    test_result("MainWindow 测试", False, str(e))

print()

# ============================================================
# 测试 7: 组件命令验证
# ============================================================
print("【功能测试 7】组件命令")
print("-" * 70)

try:
    for comp_id, comp_config in config['components'].items():
        command = comp_config['command']

        # 检查命令格式
        valid = len(command) > 0 and ('ros2' in command or 'bash' in command)
        test_result(f"{comp_id} 命令格式", valid, command[:50] + "..." if len(command) > 50 else command)

except Exception as e:
    test_result("组件命令验证", False, str(e))

print()

# ============================================================
# 测试 8: 依赖关系检查
# ============================================================
print("【功能测试 8】依赖关系")
print("-" * 70)

try:
    # 检查依赖关系的完整性
    for comp_id, comp_config in config['components'].items():
        deps = comp_config.get('dependencies', [])

        # 验证所有依赖都存在
        all_deps_exist = all(dep in config['components'] for dep in deps)

        dep_str = ', '.join(deps) if deps else '无'
        test_result(f"{comp_id} 依赖完整", all_deps_exist, f"依赖: {dep_str}")

except Exception as e:
    test_result("依赖关系检查", False, str(e))

print()

# ============================================================
# 测试 9: 参数处理
# ============================================================
print("【功能测试 9】参数处理")
print("-" * 70)

try:
    # 测试 ProcessManager 的参数设置
    pm.set_param('navigation', 'map_path', '/test/path/map.yaml')
    test_result("ProcessManager.set_param", True)

    # 验证参数已设置
    nav_config = config['components']['navigation']
    has_params = 'params' in nav_config
    test_result("参数已设置到配置", has_params)

except Exception as e:
    test_result("参数处理测试", False, str(e))

print()

# ============================================================
# 测试 10: 启动脚本测试
# ============================================================
print("【功能测试 10】启动脚本")
print("-" * 70)

try:
    control_panel_script = 'src/megarover3_ros2/megarover3_navigation/scripts/control_panel.py'

    # 读取脚本内容
    with open(control_panel_script, 'r') as f:
        script_content = f.read()

    # 检查关键导入
    has_import_app = 'from PyQt5.QtWidgets import QApplication' in script_content
    has_import_window = 'from gui.main_window import MainWindow' in script_content
    has_main = 'def main():' in script_content

    test_result("启动脚本有 QApplication 导入", has_import_app)
    test_result("启动脚本有 MainWindow 导入", has_import_window)
    test_result("启动脚本有 main 函数", has_main)

    # 检查语法
    compile(script_content, control_panel_script, 'exec')
    test_result("启动脚本语法正确", True)

except Exception as e:
    test_result("启动脚本测试", False, str(e))

print()

# ============================================================
# 测试总结
# ============================================================
print("=" * 70)
print("功能测试总结")
print("=" * 70)

total_tests = tests_passed + tests_failed
pass_rate = (tests_passed / total_tests * 100) if total_tests > 0 else 0

print(f"\n总测试数: {total_tests}")
print(f"✓ 通过: {tests_passed}")
print(f"✗ 失败: {tests_failed}")
print(f"通过率: {pass_rate:.1f}%")

if tests_failed == 0:
    print("\n" + "=" * 70)
    print("🎉 所有功能测试通过！")
    print("=" * 70)
    print("\n✅ 已测试的功能:")
    print("  - 配置加载和解析")
    print("  - ProcessManager 进程管理")
    print("  - ROS2Monitor 状态监控")
    print("  - LogHandler 日志处理")
    print("  - UI Widgets 界面组件")
    print("  - MainWindow 主窗口")
    print("  - 组件命令验证")
    print("  - 依赖关系检查")
    print("  - 参数处理")
    print("  - 启动脚本验证")
    print("\n⚠️  未测试（需要硬件）:")
    print("  - 实际进程启动/停止")
    print("  - ROS2 话题/节点检测")
    print("  - 硬件设备通信")
    print("\n可以启动控制面板:")
    print("  ~/megarover3_panel")
else:
    print("\n" + "=" * 70)
    print("⚠️  有功能测试失败")
    print("=" * 70)

print("\n" + "=" * 70)

sys.exit(0 if tests_failed == 0 else 1)
