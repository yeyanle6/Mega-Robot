#!/usr/bin/env python3
"""
模块化系统测试脚本
测试传感器检测和模式选择逻辑
"""

import subprocess
import yaml
import os

def check_hardware():
    """检查硬件设备"""
    print("=" * 50)
    print("硬件检测")
    print("=" * 50)

    # 检查Livox
    try:
        usb_devices = subprocess.check_output(['ls', '/dev/'], text=True)
        has_livox = 'ttyUSB' in usb_devices
        print(f"Livox MID360: {'✓ 检测到' if has_livox else '✗ 未检测到'}")
        if has_livox:
            print("  USB设备:", [d for d in usb_devices.split() if 'ttyUSB' in d])
    except:
        print("Livox MID360: ✗ 检测失败")
        has_livox = False

    # 检查RealSense
    try:
        lsusb = subprocess.check_output(['lsusb'], text=True)
        has_realsense = 'Intel' in lsusb and 'RealSense' in lsusb
        print(f"RealSense D455: {'✓ 检测到' if has_realsense else '✗ 未检测到'}")
        if has_realsense:
            for line in lsusb.split('\n'):
                if 'Intel' in line and 'RealSense' in line:
                    print(f"  设备信息: {line.strip()}")
    except:
        print("RealSense D455: ✗ 检测失败")
        has_realsense = False

    # 检查用户权限
    try:
        groups = subprocess.check_output(['groups'], text=True)
        in_dialout = 'dialout' in groups
        print(f"用户权限 (dialout): {'✓' if in_dialout else '✗ 需要添加到dialout组'}")
    except:
        pass

    return has_livox, has_realsense

def determine_mode(has_livox, has_realsense):
    """确定SLAM模式"""
    print("\n" + "=" * 50)
    print("模式选择")
    print("=" * 50)

    if has_livox and has_realsense:
        mode = 'fusion'
        config = 'rtabmap_fusion.yaml'
        desc = "传感器融合模式 (MID360 + D455i)"
    elif has_livox:
        mode = 'lidar_only'
        config = 'rtabmap_lidar_only.yaml'
        desc = "仅激光雷达模式 (MID360)"
    elif has_realsense:
        mode = 'rgbd_only'
        config = 'rtabmap_rgbd_only.yaml'
        desc = "仅RGB-D模式 (D455i)"
    else:
        mode = 'odometry_only'
        config = 'rtabmap_odom_only.yaml'
        desc = "纯里程计模式 (降级模式)"

    print(f"选择的模式: {mode}")
    print(f"描述: {desc}")
    print(f"配置文件: {config}")

    return mode, config

def check_config_files():
    """检查配置文件"""
    print("\n" + "=" * 50)
    print("配置文件检查")
    print("=" * 50)

    config_files = [
        'rtabmap_fusion.yaml',
        'rtabmap_lidar_only.yaml',
        'rtabmap_rgbd_only.yaml',
        'rtabmap_odom_only.yaml'
    ]

    all_exist = True
    for config in config_files:
        exists = os.path.exists(config)
        print(f"{config}: {'✓ 存在' if exists else '✗ 缺失'}")
        if not exists:
            all_exist = False

    return all_exist

def check_scripts():
    """检查脚本文件"""
    print("\n" + "=" * 50)
    print("脚本文件检查")
    print("=" * 50)

    scripts = [
        'modular_rtabmap.launch.py',
        'sensor_detector.py',
        'health_monitor.py',
        'start_modular_slam.sh',
        'monitor_rtabmap.sh',
        'diagnose_rtabmap.sh'
    ]

    all_exist = True
    for script in scripts:
        exists = os.path.exists(script)
        executable = os.access(script, os.X_OK) if exists else False
        status = '✓ 可执行' if executable else ('存在但不可执行' if exists else '✗ 缺失')
        print(f"{script:30} {status}")
        if not exists:
            all_exist = False

    return all_exist

def test_config_loading(config_file):
    """测试配置文件加载"""
    print("\n" + "=" * 50)
    print(f"测试配置加载: {config_file}")
    print("=" * 50)

    if not os.path.exists(config_file):
        print("配置文件不存在")
        return False

    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        # 检查关键参数
        if 'rtabmap' in config and 'ros__parameters' in config['rtabmap']:
            params = config['rtabmap']['ros__parameters']
            print("关键参数:")
            key_params = [
                'frame_id',
                'subscribe_depth',
                'subscribe_rgb',
                'subscribe_scan_cloud',
                'Reg/Strategy'
            ]
            for param in key_params:
                if param in params:
                    print(f"  {param}: {params[param]}")
            return True
        else:
            print("配置格式错误")
            return False

    except Exception as e:
        print(f"加载失败: {e}")
        return False

def check_ros_packages():
    """检查ROS包"""
    print("\n" + "=" * 50)
    print("ROS包检查")
    print("=" * 50)

    required_packages = [
        'rtabmap_slam',
        'rtabmap_ros',
        'pointcloud_to_laserscan',
        'tf2_ros',
        'robot_state_publisher'
    ]

    try:
        installed = subprocess.check_output(['ros2', 'pkg', 'list'], text=True).split('\n')

        all_installed = True
        for pkg in required_packages:
            is_installed = pkg in installed
            print(f"{pkg:30} {'✓ 已安装' if is_installed else '✗ 未安装'}")
            if not is_installed:
                all_installed = False

        return all_installed
    except:
        print("无法检查ROS包（ROS2未正确设置？）")
        return False

def main():
    print("\n")
    print("╔" + "=" * 48 + "╗")
    print("║" + "  模块化RTABMAP系统测试  ".center(48) + "║")
    print("╚" + "=" * 48 + "╝")

    # 硬件检测
    has_livox, has_realsense = check_hardware()

    # 模式选择
    mode, config = determine_mode(has_livox, has_realsense)

    # 配置文件检查
    configs_ok = check_config_files()

    # 脚本文件检查
    scripts_ok = check_scripts()

    # 测试选中的配置文件
    config_ok = test_config_loading(config)

    # ROS包检查
    ros_ok = check_ros_packages()

    # 总结
    print("\n" + "=" * 50)
    print("测试总结")
    print("=" * 50)

    status = {
        "硬件检测": has_livox or has_realsense,
        "配置文件": configs_ok,
        "脚本文件": scripts_ok,
        "配置加载": config_ok,
        "ROS包": ros_ok
    }

    all_ok = True
    for item, ok in status.items():
        print(f"{item:15} {'✓ 通过' if ok else '✗ 失败'}")
        if not ok:
            all_ok = False

    print("\n" + "=" * 50)
    if all_ok:
        print("✓ 系统准备就绪！")
        print(f"  建议运行: ./start_modular_slam.sh --mode {mode}")
    else:
        print("✗ 系统存在问题，请先解决上述问题")
        if not status["ROS包"]:
            print("\n建议安装缺失的包:")
            print("  sudo apt update")
            print("  sudo apt install ros-$ROS_DISTRO-rtabmap-ros")

    print("=" * 50)

if __name__ == '__main__':
    main()