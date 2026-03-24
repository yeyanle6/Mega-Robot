#!/usr/bin/env python3
"""
Asynchronous Map Saver Thread with Progress Tracking
"""

import subprocess
import shutil
import time
import os
from PyQt5.QtCore import QThread, pyqtSignal


class MapSaverThread(QThread):
    """异步地图保存线程"""

    # 信号定义
    progress_updated = pyqtSignal(int, str)  # (进度百分比, 状态消息)
    save_completed = pyqtSignal(bool, dict)  # (成功/失败, 结果详情)
    error_occurred = pyqtSignal(str, str)    # (错误类型, 错误消息)

    def __init__(self, map_path, process_manager=None):
        super().__init__()
        self.map_path = map_path
        self.map_dir = os.path.dirname(map_path)
        self.map_basename = os.path.basename(map_path)
        self.process = None
        self._is_cancelled = False
        self.process_manager = process_manager  # 用于停止 SLAM
        self.slam_component = None  # 记录哪个 SLAM 正在运行

    def run(self):
        """执行异步保存（正确顺序：PGO 3D先→2D→停止SLAM）"""
        try:
            # 步骤 1: 检查前置条件 (10%)
            self.progress_updated.emit(10, "检查系统状态...")
            time.sleep(0.5)

            if not self._check_prerequisites():
                return

            # 步骤 2: 通过 PGO 服务保存 3D 地图（SLAM 必须运行中）(20% -> 50%)
            self.progress_updated.emit(20, "调用 PGO 服务保存 3D 地图...")
            time.sleep(0.5)

            if not self._save_3d_via_pgo():
                # 3D 保存失败不阻断流程，继续保存 2D
                self.progress_updated.emit(50, "⚠ 3D 地图保存失败，继续保存 2D 地图")
            else:
                self.progress_updated.emit(50, "✓ 3D 地图保存成功")

            time.sleep(0.5)

            # 步骤 3: 保存 2D 地图（SLAM 必须运行中）(50% -> 70%)
            self.progress_updated.emit(50, "启动 2D 地图保存...")
            time.sleep(0.5)

            if not self._start_save_process():
                return

            self.progress_updated.emit(55, "订阅 /map 话题，等待 2D 地图数据...")

            if not self._wait_for_2d_map():
                return

            self.progress_updated.emit(70, "✓ 2D 地图保存成功")
            time.sleep(0.5)

            # 步骤 4: 停止 SLAM（可选）(70% -> 80%)
            self.progress_updated.emit(75, "停止 SLAM...")

            if not self._stop_slam():
                self.progress_updated.emit(80, "⚠ SLAM 未停止（可手动停止）")
            else:
                self.progress_updated.emit(80, "✓ SLAM 已停止")

            time.sleep(0.5)

            # 步骤 5: 验证保存结果 (90% -> 100%)
            self.progress_updated.emit(90, "验证保存结果...")
            time.sleep(0.5)

            result = self._verify_saved_files()

            self.progress_updated.emit(100, "地图保存完成！")
            self.save_completed.emit(True, result)

        except Exception as e:
            self.error_occurred.emit("未知错误", str(e))
            self.save_completed.emit(False, {})

    def _check_prerequisites(self):
        """检查前置条件"""
        # 检查 save_map.py 脚本是否存在
        script_path = '/home/ros/Code/Demo8/save_map.py'
        if not os.path.exists(script_path):
            self.error_occurred.emit(
                "脚本缺失",
                f"找不到保存脚本:\n{script_path}\n\n请确保已正确安装控制面板。"
            )
            return False

        # 检查目标目录是否存在，不存在则创建
        if self.map_dir and not os.path.exists(self.map_dir):
            try:
                os.makedirs(self.map_dir)
            except Exception as e:
                self.error_occurred.emit(
                    "目录创建失败",
                    f"无法创建保存目录:\n{self.map_dir}\n\n错误: {str(e)}"
                )
                return False

        # 检查 SLAM PGO 是否运行（必须是 PGO 模式）
        slam_running, slam_type = self._check_slam_running()
        if not slam_running:
            self.error_occurred.emit(
                "SLAM PGO 未运行",
                "保存地图需要 SLAM (PGO 模式) 正在运行。\n\n"
                "原因：\n"
                "• 3D 地图需要调用 PGO 服务 (/pgo/save_maps) 保存回环优化结果\n"
                "• 2D 地图需要订阅 /map 话题，该话题由 SLAM 发布\n\n"
                "请先启动 SLAM (建图-PGO优化) 模式。\n"
                "普通 SLAM 模式不支持 PGO 优化保存。"
            )
            return False

        # 记录运行中的 SLAM 组件
        self.slam_component = slam_type

        # 检查是否有 ROS2 话题 /map
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if '/map' not in result.stdout:
                self.error_occurred.emit(
                    "地图话题未发布",
                    "SLAM 正在运行，但未检测到 /map 话题。\n\n"
                    "可能原因：\n"
                    "• OctoMap 服务器未启动\n"
                    "• SLAM 刚启动，地图尚未生成\n\n"
                    "建议：\n"
                    "• 等待几秒后重试\n"
                    "• 检查 SLAM 组件是否正常运行\n\n"
                    "当前地图相关话题:\n" +
                    '\n'.join([t for t in result.stdout.split('\n') if 'map' in t.lower()][:5])
                )
                return False
        except subprocess.TimeoutExpired:
            self.error_occurred.emit(
                "ROS2 检查超时",
                "检查 ROS2 话题时超时，请确保 ROS2 环境正常。"
            )
            return False
        except Exception as e:
            self.error_occurred.emit(
                "ROS2 检查失败",
                f"无法检查 ROS2 话题:\n{str(e)}"
            )
            return False

        return True

    def _check_slam_running(self):
        """检查 SLAM PGO 是否正在运行，返回 (是否运行, SLAM类型)"""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )

            # 只认 PGO 模式（检测 /pgo/pgo_node）
            if '/pgo/pgo_node' in result.stdout:
                return (True, 'slam_pgo')

            return (False, None)

        except Exception:
            return (False, None)

    def _save_3d_via_pgo(self):
        """通过 PGO 服务保存回环优化后的 3D 地图（SLAM 必须运行中）"""
        # 1. 确保目标目录存在（PGO 的 file_path 要求是已存在的目录）
        pgo_save_dir = self.map_path + '_pgo'
        try:
            os.makedirs(pgo_save_dir, exist_ok=True)
        except Exception as e:
            self.error_occurred.emit(
                "PGO 目录创建失败",
                f"无法创建 PGO 保存目录:\n{pgo_save_dir}\n\n错误: {str(e)}"
            )
            return False

        self.progress_updated.emit(25, "调用 /pgo/save_maps 服务...")

        # 2. 用 subprocess 调用 ros2 service call
        cmd = [
            'ros2', 'service', 'call',
            '/pgo/save_maps', 'interface/srv/SaveMaps',
            '{file_path: "' + pgo_save_dir + '", save_patches: true}'
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60
            )
        except subprocess.TimeoutExpired:
            self.error_occurred.emit(
                "PGO 保存超时",
                "调用 /pgo/save_maps 服务超时（60秒）。\n\n"
                "可能原因:\n"
                "• 点云数据量过大\n"
                "• PGO 节点处理缓慢\n\n"
                "建议:\n"
                "• 重试保存操作\n"
                "• 检查 PGO 节点日志"
            )
            return False
        except Exception as e:
            self.error_occurred.emit(
                "PGO 服务调用失败",
                f"调用 /pgo/save_maps 服务时出错:\n{str(e)}"
            )
            return False

        self.progress_updated.emit(35, "等待 PGO 保存完成...")

        # 3. 检查结果（从输出中解析 success 字段）
        if 'success=True' not in result.stdout and 'SAVE SUCCESS' not in result.stdout:
            self.error_occurred.emit(
                "PGO 保存失败",
                f"PGO save_maps 服务返回失败。\n\n"
                f"stdout:\n{result.stdout[:500] if result.stdout else '(无输出)'}\n\n"
                f"stderr:\n{result.stderr[:500] if result.stderr else '(无输出)'}"
            )
            return False

        self.progress_updated.emit(40, "PGO 保存完成，复制 3D 地图文件...")

        # 4. 将 pgo_save_dir/map.pcd 复制到 self.map_path.pcd
        pgo_map = os.path.join(pgo_save_dir, 'map.pcd')
        if os.path.exists(pgo_map):
            try:
                dest_pcd = self.map_path + '.pcd'
                shutil.copy2(pgo_map, dest_pcd)
                size_mb = os.path.getsize(dest_pcd) / (1024 * 1024)
                self.progress_updated.emit(45, f"✓ 3D 点云已保存 ({size_mb:.1f} MB)")
            except Exception as e:
                self.error_occurred.emit(
                    "3D 地图复制失败",
                    f"PGO 保存成功但复制 map.pcd 失败:\n{str(e)}\n\n"
                    f"源文件: {pgo_map}\n"
                    f"目标文件: {self.map_path}.pcd"
                )
                return False
        else:
            dir_contents = '\n'.join(os.listdir(pgo_save_dir)) if os.path.isdir(pgo_save_dir) else '(目录不存在)'
            self.error_occurred.emit(
                "PGO 地图文件未生成",
                f"PGO 服务返回成功，但未找到 map.pcd:\n{pgo_save_dir}\n\n"
                f"目录内容:\n{dir_contents}"
            )
            return False

        return True

    def _start_save_process(self):
        """启动 2D 地图保存进程（只保存 2D）"""
        cmd = f'cd /home/ros/Code/Demo8 && python3 save_map.py {self.map_path} --2d-only'

        try:
            self.process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            return True
        except Exception as e:
            self.error_occurred.emit(
                "进程启动失败",
                f"无法启动 2D 地图保存脚本:\n{str(e)}"
            )
            return False

    def _wait_for_2d_map(self):
        """等待 2D 地图保存完成"""
        timeout = 30  # 30 秒超时
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self._is_cancelled:
                self._terminate_process()
                self.error_occurred.emit("已取消", "用户取消了保存操作")
                return False

            # 检查进程是否仍在运行
            if self.process.poll() is not None:
                # 进程已结束，检查是否成功
                break

            # 检查 2D 地图文件是否已生成
            if os.path.exists(f'{self.map_path}.pgm') and os.path.exists(f'{self.map_path}.yaml'):
                self.progress_updated.emit(65, "2D 地图保存成功")
                return True

            time.sleep(0.5)

        # 超时检查
        if time.time() - start_time >= timeout:
            self._terminate_process()
            self.error_occurred.emit(
                "2D 地图保存超时",
                f"等待 2D 地图数据超过 {timeout} 秒。\n\n"
                "可能原因:\n"
                "• /map 话题未发布数据\n"
                "• SLAM 节点未正常运行\n"
                "• 地图数据量过大\n\n"
                "建议:\n"
                "• 检查 SLAM 是否正常运行\n"
                "• 查看日志中的错误信息"
            )
            return False

        return True

    def _stop_slam(self):
        """停止 SLAM（可选步骤，地图已保存完毕）"""
        if not self.process_manager or not self.slam_component:
            return False

        try:
            if self.process_manager.is_running(self.slam_component):
                self.process_manager.stop_component(self.slam_component)
                self.progress_updated.emit(77, f"正在停止 {self._get_slam_name(self.slam_component)}...")

                # 等待进程完全停止
                time.sleep(2)

                return True
            else:
                return False

        except Exception as e:
            self.progress_updated.emit(80, f"⚠ 停止 SLAM 失败: {str(e)}")
            return False

    def _get_slam_name(self, slam_id):
        """获取 SLAM 显示名称"""
        names = {
            'slam': 'SLAM (简单模式)',
            'slam_pgo': 'SLAM (建图-PGO优化)'
        }
        return names.get(slam_id, slam_id)

    def _verify_saved_files(self):
        """验证保存的文件"""
        result = {
            'map_2d_saved': False,
            'map_3d_saved': False,
            'files': []
        }

        # 检查 2D 地图文件
        pgm_file = f'{self.map_path}.pgm'
        yaml_file = f'{self.map_path}.yaml'

        if os.path.exists(pgm_file) and os.path.exists(yaml_file):
            result['map_2d_saved'] = True
            result['files'].append({
                'name': os.path.basename(pgm_file),
                'size': os.path.getsize(pgm_file),
                'type': '2D 栅格图像'
            })
            result['files'].append({
                'name': os.path.basename(yaml_file),
                'size': os.path.getsize(yaml_file),
                'type': '2D 地图元数据'
            })

        # 检查 3D 地图文件（PGO 优化版本）
        pcd_file = f'{self.map_path}.pcd'

        if os.path.exists(pcd_file):
            result['map_3d_saved'] = True
            result['files'].append({
                'name': os.path.basename(pcd_file),
                'size': os.path.getsize(pcd_file),
                'type': '3D 点云地图 (PGO优化)'
            })

        # 检查 PGO 完整输出目录
        pgo_dir = self.map_path + '_pgo'
        if os.path.isdir(pgo_dir):
            # 检查 patches 目录
            patches_dir = os.path.join(pgo_dir, 'patches')
            if os.path.isdir(patches_dir):
                patch_count = len([f for f in os.listdir(patches_dir) if f.endswith('.pcd')])
                result['files'].append({
                    'name': f'{self.map_basename}_pgo/patches/',
                    'size': patch_count,
                    'type': f'PGO 分片点云 ({patch_count} 个)'
                })

            # 检查 poses.txt
            poses_file = os.path.join(pgo_dir, 'poses.txt')
            if os.path.exists(poses_file):
                result['files'].append({
                    'name': f'{self.map_basename}_pgo/poses.txt',
                    'size': os.path.getsize(poses_file),
                    'type': 'PGO 位姿数据'
                })

        # 如果 2D 和 3D 都没保存，说明完全失败
        if not result['map_2d_saved'] and not result['map_3d_saved']:
            stderr_output = ""
            if self.process and self.process.stderr:
                stderr_output = self.process.stderr.read()

            self.error_occurred.emit(
                "未生成任何地图文件",
                f"保存操作完成但未找到任何输出文件。\n\n"
                f"目标路径: {self.map_path}\n\n"
                f"脚本输出:\n{stderr_output[:500] if stderr_output else '(无输出)'}"
            )

        return result

    def _terminate_process(self):
        """终止保存进程"""
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()

    def cancel(self):
        """取消保存操作"""
        self._is_cancelled = True
