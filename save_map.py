#!/usr/bin/env python3
"""
统一地图保存脚本

用法:
  # 完整保存（PGO 3D + 2D），SLAM PGO 必须运行中
  python3 save_map.py maps/my_map

  # 只保存 2D 栅格地图（供 map_saver_thread 内部调用）
  python3 save_map.py maps/my_map --2d-only
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np
from PIL import Image
import subprocess
import shutil
import sys
import os


class Map2DSaver(Node):
    """订阅 /map 话题保存 2D 栅格地图"""

    def __init__(self, map_name):
        super().__init__('map_2d_saver')
        self.map_name = map_name
        self.map_received = False

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )

        self.get_logger().info('等待 2D 地图数据...')

    def map_callback(self, msg):
        if self.map_received:
            return

        self.get_logger().info('收到 2D 地图，正在保存...')
        self.map_received = True

        try:
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin = msg.info.origin

            map_data = np.array(msg.data).reshape((height, width))

            img_data = np.zeros((height, width), dtype=np.uint8)
            img_data[map_data == -1] = 205   # Unknown (gray)
            img_data[map_data == 0] = 254    # Free (white)
            img_data[map_data == 100] = 0    # Occupied (black)

            mask = (map_data > 0) & (map_data < 100)
            img_data[mask] = (254 - (map_data[mask] * 254 / 100)).astype(np.uint8)

            img_data = np.flipud(img_data)

            img = Image.fromarray(img_data, mode='L')
            pgm_path = f'{self.map_name}.pgm'
            img.save(pgm_path)

            yaml_data = {
                'image': f'{os.path.basename(self.map_name)}.pgm',
                'resolution': float(resolution),
                'origin': [float(origin.position.x), float(origin.position.y), 0.0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196
            }

            yaml_path = f'{self.map_name}.yaml'
            with open(yaml_path, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False)

            self.get_logger().info('✓ 2D 地图保存成功')
            self.get_logger().info(f'  {pgm_path} ({width}x{height}, {resolution:.3f} m/px)')
            self.get_logger().info(f'  {yaml_path}')

        except Exception as e:
            self.get_logger().error(f'保存 2D 地图失败: {str(e)}')
            rclpy.shutdown()
            sys.exit(1)

        rclpy.shutdown()


def save_3d_via_pgo(map_path):
    """调用 PGO 服务保存回环优化后的 3D 地图"""
    pgo_save_dir = map_path + '_pgo'
    os.makedirs(pgo_save_dir, exist_ok=True)

    print(f'[3D] 调用 /pgo/save_maps → {pgo_save_dir}')

    cmd = [
        'ros2', 'service', 'call',
        '/pgo/save_maps', 'interface/srv/SaveMaps',
        '{file_path: "' + pgo_save_dir + '", save_patches: true}'
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        print('[3D] ✗ PGO 服务调用超时 (60s)')
        return False

    if 'success=True' not in result.stdout and 'SAVE SUCCESS' not in result.stdout:
        print(f'[3D] ✗ PGO 保存失败: {result.stdout.strip()}')
        return False

    pgo_map = os.path.join(pgo_save_dir, 'map.pcd')
    if os.path.exists(pgo_map):
        dest_pcd = map_path + '.pcd'
        shutil.copy2(pgo_map, dest_pcd)
        size_mb = os.path.getsize(dest_pcd) / (1024 * 1024)
        print(f'[3D] ✓ {dest_pcd} ({size_mb:.2f} MB)')
        return True
    else:
        print(f'[3D] ✗ PGO 服务成功但 map.pcd 未生成')
        return False


def save_2d(map_path):
    """启动 ROS2 节点订阅 /map 保存 2D 地图"""
    print('[2D] 订阅 /map 保存 2D 栅格地图...')

    rclpy.init()
    node = Map2DSaver(map_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        success = node.map_received
        node.destroy_node()

    if success:
        print('[2D] ✓ 完成')
    else:
        print('[2D] ✗ 未收到 /map 数据')

    return success


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(__doc__.strip())
        sys.exit(0)

    map_path = sys.argv[1]
    only_2d = '--2d-only' in sys.argv

    # 确保目录存在
    map_dir = os.path.dirname(map_path)
    if map_dir:
        os.makedirs(map_dir, exist_ok=True)

    if only_2d:
        # 仅保存 2D（供 map_saver_thread 调用）
        ok = save_2d(map_path)
        sys.exit(0 if ok else 1)

    # 完整保存: PGO 3D → 2D
    print(f'保存地图: {map_path}')
    print('=' * 50)

    ok_3d = save_3d_via_pgo(map_path)
    ok_2d = save_2d(map_path)

    print('=' * 50)
    print(f'3D PGO: {"✓" if ok_3d else "✗"}')
    print(f'2D 栅格: {"✓" if ok_2d else "✗"}')

    sys.exit(0 if (ok_3d and ok_2d) else 1)


if __name__ == '__main__':
    main()
