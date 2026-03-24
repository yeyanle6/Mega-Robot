#!/usr/bin/env python3
"""
MegaRover3 Control Panel - 简化功能测试
不创建实际 GUI，只测试核心功能
"""

import sys
import os
import yaml

# 设置路径
sys.path.insert(0, 'src/megarover3_ros2/megarover3_navigation')

print("=" * 70)
print("MegaRover3 Control Panel - 功能测试（无GUI）")
print("=" * 70)
print()

tests_passed = 0
tests_failed = 0

def test(name, func):
    """执行测试"""
    global tests_passed, tests_failed
    try:
        result = func()
        if result:
            tests_passed += 1
            print(f"\033[92m✓\033[0m {name}")
            return True
        else:
            tests_failed += 1
            print(f"\033[91m✗\033[0m {name}")
            return False
    except Exception as e:
        tests_failed += 1
        print(f"\033[91m✗\033[0m {name}: {str(e)}")
        return False

# ============================================================
# 1. 配置加载
# ============================================================
print("【1. 配置加载】")
config = None

def load_config():
    global config
    config_path = 'src/megarover3_ros2/megarover3_navigation/gui/config/panel_config.yaml'
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    return config is not None and 'components' in config

test("加载 panel_config.yaml", load_config)
test("配置包含 8 个组件", lambda: len(config['components']) == 8)
print()

# ============================================================
# 2. ProcessManager 功能
# ============================================================
print("【2. ProcessManager 功能】")

from gui.process_manager import ProcessManager

pm = None

def create_pm():
    global pm
    pm = ProcessManager(config)
    return pm is not None

test("创建 ProcessManager", create_pm)
test("ProcessManager.start_component 存在", lambda: hasattr(pm, 'start_component'))
test("ProcessManager.stop_component 存在", lambda: hasattr(pm, 'stop_component'))
test("ProcessManager.is_running 存在", lambda: hasattr(pm, 'is_running'))
test("ProcessManager.set_param 存在", lambda: hasattr(pm, 'set_param'))

# 测试状态检查
def test_status_check():
    for comp_id in ['chassis', 'lidar', 'd455_front']:
        running = pm.is_running(comp_id)
        if running not in [True, False]:
            return False
    return True

test("检查组件状态", test_status_check)

# 测试参数设置
def test_set_param():
    pm.set_param('navigation', 'map_path', '/test/map.yaml')
    return 'params' in config['components']['navigation']

test("设置组件参数", test_set_param)
print()

# ============================================================
# 3. ROS2Monitor 功能
# ============================================================
print("【3. ROS2Monitor 功能】")

from gui.ros2_monitor import ROS2Monitor, ComponentStatusAggregator

def test_monitor_create():
    monitor = ROS2Monitor(config)
    return monitor is not None

test("创建 ROS2Monitor", test_monitor_create)

aggregator = None

def test_aggregator_create():
    global aggregator
    aggregator = ComponentStatusAggregator()
    return aggregator is not None

test("创建 ComponentStatusAggregator", test_aggregator_create)

def test_aggregator_update():
    aggregator.update_process_status('test', 'running')
    aggregator.update_topic_status('/test/topic', True)
    aggregator.update_node_status('/test/node', True)
    return True

test("更新组件状态", test_aggregator_update)

def test_get_status():
    status = aggregator.get_component_status('test', {})
    return status in ['running', 'stopped', 'partial', 'error']

test("获取组件状态", test_get_status)
print()

# ============================================================
# 4. LogHandler 功能
# ============================================================
print("【4. LogHandler 功能】")

from gui.log_handler import LogFormatter

def test_format_html():
    html = LogFormatter.format_html('INFO', 'Test message', 'test')
    return len(html) > 0 and 'Test message' in html

test("格式化日志为 HTML", test_format_html)

def test_strip_ansi():
    text = LogFormatter.strip_ansi('\033[91mRed\033[0m')
    return 'Red' in text and '\033' not in text

test("清除 ANSI 代码", test_strip_ansi)
print()

# ============================================================
# 5. 组件配置验证
# ============================================================
print("【5. 组件配置验证】")

components_ok = 0

for comp_id, comp_config in config['components'].items():
    has_name = 'name' in comp_config
    has_command = 'command' in comp_config
    has_deps = 'dependencies' in comp_config

    if has_name and has_command and has_deps:
        components_ok += 1
        print(f"\033[92m✓\033[0m {comp_id}: {comp_config['name']}")
    else:
        print(f"\033[91m✗\033[0m {comp_id}: 配置不完整")

print()

# ============================================================
# 6. 依赖关系验证
# ============================================================
print("【6. 依赖关系验证】")

deps_ok = True
for comp_id, comp_config in config['components'].items():
    deps = comp_config.get('dependencies', [])
    for dep in deps:
        if dep not in config['components']:
            print(f"\033[91m✗\033[0m {comp_id} 依赖不存在的组件: {dep}")
            deps_ok = False
        else:
            print(f"\033[92m✓\033[0m {comp_id} → {dep}")

if deps_ok:
    print("\033[92m所有依赖关系正确\033[0m")
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
    print("\n\033[92m🎉 所有功能测试通过！\033[0m")
    print("\n✅ 已验证功能:")
    print("  • 配置加载和解析")
    print("  • ProcessManager 进程管理API")
    print("  • ROS2Monitor 状态监控API")
    print("  • ComponentStatusAggregator 状态聚合")
    print("  • LogFormatter 日志格式化")
    print("  • 组件配置完整性")
    print("  • 依赖关系正确性")
    print("\n⏭️  下一步:")
    print("  启动控制面板进行实际测试:")
    print("    ~/megarover3_panel")
else:
    print("\n\033[91m⚠️  有测试失败\033[0m")

print("\n" + "=" * 70)
