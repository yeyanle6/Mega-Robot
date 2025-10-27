#!/usr/bin/env python3
"""
系统验证脚本 - 检查ROS2包集成是否正确
"""

import os
import sys
import subprocess
import yaml
from termcolor import colored
import time

class SystemValidator:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.successes = []

    def print_header(self, text):
        print("\n" + "="*50)
        print(text.center(50))
        print("="*50)

    def check_result(self, condition, success_msg, error_msg, is_critical=True):
        if condition:
            self.successes.append(success_msg)
            print(colored(f"✓ {success_msg}", "green"))
            return True
        else:
            if is_critical:
                self.errors.append(error_msg)
                print(colored(f"✗ {error_msg}", "red"))
            else:
                self.warnings.append(error_msg)
                print(colored(f"⚠ {error_msg}", "yellow"))
            return False

    def check_ros_environment(self):
        """检查ROS2环境"""
        self.print_header("ROS2环境检查")

        # 检查ROS_DISTRO
        ros_distro = os.environ.get('ROS_DISTRO')
        self.check_result(
            ros_distro is not None,
            f"ROS2环境已设置: {ros_distro}",
            "ROS2环境未设置"
        )

        # 检查包是否可以找到
        try:
            result = subprocess.run(
                ['ros2', 'pkg', 'prefix', 'megarover_navigation'],
                capture_output=True, text=True, timeout=5
            )
            pkg_found = result.returncode == 0
            self.check_result(
                pkg_found,
                "megarover_navigation包已安装",
                "megarover_navigation包未找到"
            )
        except:
            self.check_result(False, "", "无法执行ros2命令")

    def check_dependencies(self):
        """检查依赖包"""
        self.print_header("依赖包检查")

        required_packages = [
            'nav2_bringup',
            'nav2_bt_navigator',
            'nav2_controller',
            'nav2_planner',
            'rtabmap_ros',
            'rtabmap_slam',
            'pointcloud_to_laserscan',
            'megarover3_bringup',
            'livox_ros_driver2',
            'realsense2_camera'
        ]

        try:
            result = subprocess.run(
                ['ros2', 'pkg', 'list'],
                capture_output=True, text=True, timeout=5
            )
            installed_packages = result.stdout.split('\n') if result.returncode == 0 else []

            for pkg in required_packages:
                is_installed = pkg in installed_packages
                self.check_result(
                    is_installed,
                    f"{pkg} 已安装",
                    f"{pkg} 未安装",
                    is_critical=(pkg not in ['livox_ros_driver2', 'realsense2_camera'])
                )
        except Exception as e:
            self.errors.append(f"检查依赖包失败: {e}")

    def check_file_structure(self):
        """检查文件结构"""
        self.print_header("文件结构检查")

        # 获取包路径
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('megarover_navigation')
        except:
            pkg_dir = None
            self.errors.append("无法获取包路径")
            return

        # 检查关键文件
        required_files = {
            'launch': [
                'navigation.launch.py',
                'modular_rtabmap.launch.py',
                'megarover_nav2_slam.launch.py'
            ],
            'config': [
                'nav2_params.yaml',
                'rtabmap_fusion.yaml',
                'rtabmap_lidar_only.yaml',
                'rtabmap_rgbd_only.yaml',
                'rtabmap_odom_only.yaml'
            ],
            'rviz': [
                'nav2_rviz_config.rviz'
            ]
        }

        for directory, files in required_files.items():
            dir_path = os.path.join(pkg_dir, directory)
            for file in files:
                file_path = os.path.join(dir_path, file)
                self.check_result(
                    os.path.exists(file_path),
                    f"{directory}/{file} 存在",
                    f"{directory}/{file} 缺失"
                )

    def check_nodes(self):
        """检查节点可执行性"""
        self.print_header("节点检查")

        nodes = [
            ('megarover_navigation', 'sensor_detector'),
            ('megarover_navigation', 'health_monitor')
        ]

        for pkg, executable in nodes:
            try:
                result = subprocess.run(
                    ['ros2', 'run', pkg, executable, '--help'],
                    capture_output=True, text=True, timeout=2
                )
                # 即使返回非0（因为缺少参数），只要命令能运行就算成功
                node_exists = 'usage:' in result.stdout.lower() or result.returncode != 127
                self.check_result(
                    node_exists,
                    f"{pkg}::{executable} 可执行",
                    f"{pkg}::{executable} 无法执行",
                    is_critical=False
                )
            except subprocess.TimeoutExpired:
                # 节点启动了但没有--help，这是正常的
                self.successes.append(f"{pkg}::{executable} 可执行")
                print(colored(f"✓ {pkg}::{executable} 可执行", "green"))
            except Exception as e:
                self.warnings.append(f"{pkg}::{executable} 检查失败: {e}")

    def check_launch_files(self):
        """检查Launch文件语法"""
        self.print_header("Launch文件检查")

        launch_files = [
            'navigation.launch.py',
            'modular_rtabmap.launch.py',
            'megarover_nav2_slam.launch.py'
        ]

        for launch_file in launch_files:
            try:
                # 尝试导入launch文件来检查语法
                result = subprocess.run(
                    ['python3', '-c',
                     f"from ament_index_python.packages import get_package_share_directory; "
                     f"import sys; import os; "
                     f"sys.path.insert(0, os.path.join(get_package_share_directory('megarover_navigation'), 'launch')); "
                     f"import {launch_file[:-3]}"],
                    capture_output=True, text=True, timeout=5
                )

                syntax_ok = result.returncode == 0
                self.check_result(
                    syntax_ok,
                    f"{launch_file} 语法正确",
                    f"{launch_file} 语法错误: {result.stderr[:100]}"
                )
            except Exception as e:
                self.warnings.append(f"{launch_file} 检查失败: {e}")

    def check_config_files(self):
        """检查配置文件格式"""
        self.print_header("配置文件检查")

        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('megarover_navigation')
        except:
            return

        config_files = [
            'nav2_params.yaml',
            'rtabmap_fusion.yaml',
            'rtabmap_lidar_only.yaml',
            'rtabmap_rgbd_only.yaml',
            'rtabmap_odom_only.yaml'
        ]

        for config_file in config_files:
            config_path = os.path.join(pkg_dir, 'config', config_file)
            if os.path.exists(config_path):
                try:
                    with open(config_path, 'r') as f:
                        yaml.safe_load(f)
                    self.check_result(
                        True,
                        f"{config_file} 格式正确",
                        ""
                    )
                except yaml.YAMLError as e:
                    self.check_result(
                        False,
                        "",
                        f"{config_file} YAML格式错误: {str(e)[:50]}"
                    )

    def check_hardware(self):
        """检查硬件设备"""
        self.print_header("硬件设备检查")

        # 检查Livox
        try:
            usb_devices = subprocess.check_output(['ls', '/dev/'], text=True)
            has_livox = 'ttyUSB' in usb_devices
            self.check_result(
                has_livox,
                "检测到USB设备（可能是Livox）",
                "未检测到Livox设备",
                is_critical=False
            )
        except:
            pass

        # 检查RealSense
        try:
            lsusb = subprocess.check_output(['lsusb'], text=True)
            has_realsense = 'Intel' in lsusb and 'RealSense' in lsusb
            self.check_result(
                has_realsense,
                "检测到RealSense相机",
                "未检测到RealSense相机",
                is_critical=False
            )
        except:
            pass

        # 检查用户权限
        try:
            groups_output = subprocess.check_output(['groups'], text=True)
            in_dialout = 'dialout' in groups_output
            self.check_result(
                in_dialout,
                "用户在dialout组",
                "用户不在dialout组（可能无法访问串口设备）",
                is_critical=False
            )
        except:
            pass

    def print_summary(self):
        """打印总结"""
        self.print_header("验证总结")

        print(f"\n成功: {len(self.successes)} 项")
        print(f"警告: {len(self.warnings)} 项")
        print(f"错误: {len(self.errors)} 项")

        if self.errors:
            print(colored("\n错误列表:", "red"))
            for error in self.errors:
                print(colored(f"  • {error}", "red"))

        if self.warnings:
            print(colored("\n警告列表:", "yellow"))
            for warning in self.warnings:
                print(colored(f"  • {warning}", "yellow"))

        if not self.errors:
            print(colored("\n✓ 系统验证通过！", "green", attrs=['bold']))
            print("\n可以运行以下命令启动系统:")
            print("  ros2 launch megarover_navigation navigation.launch.py")
        else:
            print(colored("\n✗ 系统存在错误，请先解决上述问题", "red", attrs=['bold']))

        return len(self.errors) == 0

def main():
    # 尝试导入termcolor，如果失败则定义简单的colored函数
    global colored
    try:
        from termcolor import colored
    except ImportError:
        def colored(text, color, attrs=None):
            return text
        print("提示: 安装termcolor以获得彩色输出: pip3 install termcolor")

    validator = SystemValidator()

    print("="*50)
    print("MegaRover Navigation 系统验证".center(50))
    print("="*50)

    # 运行所有检查
    validator.check_ros_environment()
    validator.check_dependencies()
    validator.check_file_structure()
    validator.check_nodes()
    validator.check_launch_files()
    validator.check_config_files()
    validator.check_hardware()

    # 打印总结
    success = validator.print_summary()

    # 返回状态码
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()