#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import yaml
from PIL import Image
import open3d as o3d


class CorrectedPCDConverter:
    """修正的PCD转换器，正确处理MID360激光雷达数据"""
    
    def __init__(self, lidar_height=0.4, robot_height=0.46, ground_tolerance=0.1):
        """
        初始化转换器
        
        Args:
            lidar_height: 激光雷达安装高度 (米)
            robot_height: 机器人最高点高度 (米)
            ground_tolerance: 地面检测容差 (米)
        """
        self.lidar_height = lidar_height
        self.robot_height = robot_height
        self.ground_tolerance = ground_tolerance
        
    def detect_ground_height(self, points):
        """
        检测地面高度
        
        Args:
            points: 点云数据 (Nx3 numpy数组)
        
        Returns:
            ground_height: 检测到的地面高度 (米)
        """
        z_coords = points[:, 2]
        
        # 使用直方图方法检测地面
        hist, bin_edges = np.histogram(z_coords, bins=200)
        
        # 找到最频繁的高度区间（假设地面点最多）
        max_count_idx = np.argmax(hist)
        ground_height = (bin_edges[max_count_idx] + bin_edges[max_count_idx + 1]) / 2
        
        # 进一步优化：在检测到的地面高度附近寻找更精确的地面
        ground_candidates = z_coords[(z_coords >= ground_height - self.ground_tolerance) & 
                                    (z_coords <= ground_height + self.ground_tolerance)]
        
        if len(ground_candidates) > 0:
            ground_height = np.median(ground_candidates)
        
        return ground_height
    
    def filter_points_for_navigation(self, points, ground_height):
        """
        为机器人导航过滤点云
        
        逻辑：
        1. 激光雷达安装在0.4米高度
        2. 机器人最高点约0.46米
        3. 需要检测从地面到机器人顶部的障碍物
        4. 激光雷达只能精确扫描安装位置以上的障碍物
        
        Args:
            points: 点云数据 (Nx3 numpy数组)
            ground_height: 地面高度 (米)
        
        Returns:
            filtered_points: 过滤后的点云
            height_range: 使用的高度范围
        """
        # 计算合适的高度范围
        # 从地面开始，到机器人顶部
        height_min = ground_height  # 从地面开始
        height_max = ground_height + self.robot_height  # 到机器人顶部
        
        print(f"地面高度: {ground_height:.3f} 米")
        print(f"激光雷达安装高度: {self.lidar_height:.3f} 米")
        print(f"机器人总高度: {self.robot_height:.3f} 米")
        print(f"导航高度范围: [{height_min:.3f}, {height_max:.3f}] 米")
        
        # 过滤高度范围内的点
        mask = (points[:, 2] >= height_min) & (points[:, 2] <= height_max)
        filtered_points = points[mask]
        
        return filtered_points, (height_min, height_max)
    
    def remove_outliers(self, points, nb_neighbors=20, std_ratio=2.0):
        """
        移除离群点
        
        Args:
            points: 点云数据 (Nx3 numpy数组)
            nb_neighbors: 邻居数量
            std_ratio: 标准差比例
        
        Returns:
            filtered_points: 过滤后的点云
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # 移除离群点
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, 
                                               std_ratio=std_ratio)
        
        return np.asarray(pcd.points)
    
    def convert_to_pgm(self, pcd_file, output_dir, resolution=0.05, 
                      remove_outliers_flag=True):
        """
        将PCD文件转换为PGM栅格地图
        
        Args:
            pcd_file: 输入PCD文件路径
            output_dir: 输出目录
            resolution: 地图分辨率 (米/像素)
            remove_outliers_flag: 是否移除离群点
        
        Returns:
            success: 转换是否成功
        """
        # 读取PCD文件
        print(f"读取PCD文件: {pcd_file}")
        pcd = o3d.io.read_point_cloud(pcd_file)
        
        if len(pcd.points) == 0:
            print("错误: PCD文件为空或无法读取")
            return False
        
        # 获取点云数据
        points = np.asarray(pcd.points)
        print(f"原始点数: {len(points)}")
        
        # 移除离群点
        if remove_outliers_flag:
            print("正在移除离群点...")
            points = self.remove_outliers(points)
            print(f"移除离群点后: {len(points)} 个点")
        
        # 检测地面高度
        print("正在检测地面高度...")
        ground_height = self.detect_ground_height(points)
        
        # 为机器人导航过滤点云
        filtered_points, height_range = self.filter_points_for_navigation(points, ground_height)
        
        print(f"导航过滤后点数: {len(filtered_points)}")
        
        if len(filtered_points) == 0:
            print("错误: 过滤后没有点云数据")
            return False
        
        # 计算地图边界
        x_min, y_min = np.min(filtered_points[:, :2], axis=0)
        x_max, y_max = np.max(filtered_points[:, :2], axis=0)
        
        print(f"地图边界: X[{x_min:.2f}, {x_max:.2f}], Y[{y_min:.2f}, {y_max:.2f}]")
        
        # 计算地图尺寸
        width = int((x_max - x_min) / resolution) + 1
        height = int((y_max - y_min) / resolution) + 1
        
        print(f"地图尺寸: {width} x {height} 像素")
        
        # 创建栅格地图
        grid_map = np.zeros((height, width), dtype=np.uint8)
        
        # 将点云投影到栅格
        for point in filtered_points:
            x, y = point[0], point[1]
            col = int((x - x_min) / resolution)
            row = int((y_max - y) / resolution)  # 翻转Y轴
            
            if 0 <= row < height and 0 <= col < width:
                grid_map[row, col] = 255  # 占用区域
        
        # 保存PGM文件
        pgm_file = os.path.join(output_dir, "map.pgm")
        img = Image.fromarray(grid_map, mode='L')
        img.save(pgm_file)
        print(f"保存PGM文件: {pgm_file}")
        
        # 创建YAML配置文件
        yaml_file = os.path.join(output_dir, "map.yaml")
        yaml_data = {
            'image': 'map.pgm',
            'resolution': float(resolution),
            'origin': [float(x_min), float(y_min), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'height_range': [float(height_range[0]), float(height_range[1])],
            'ground_height': float(ground_height),
            'lidar_height': float(self.lidar_height),
            'robot_height': float(self.robot_height),
            'conversion_method': 'corrected_navigation_filtering'
        }

        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
        
        print(f"保存YAML文件: {yaml_file}")
        
        # 打印地图信息
        print("\n地图信息:")
        print(f"  分辨率: {resolution} 米/像素")
        print(f"  尺寸: {width} x {height} 像素")
        print(f"  物理尺寸: {width * resolution:.2f} x {height * resolution:.2f} 米")
        print(f"  原点: [{x_min:.2f}, {y_min:.2f}, 0.0]")
        print(f"  地面高度: {ground_height:.3f} 米")
        print(f"  激光雷达高度: {self.lidar_height:.3f} 米")
        print(f"  机器人高度: {self.robot_height:.3f} 米")
        print(f"  导航高度范围: [{height_range[0]:.3f}, {height_range[1]:.3f}] 米")
        
        return True


def main():
    parser = argparse.ArgumentParser(description='修正的MID360地图转换工具')
    parser.add_argument('pcd_file', help='输入PCD文件路径')
    parser.add_argument('-o', '--output', default='./maps', help='输出目录 (默认: ./maps)')
    parser.add_argument('-r', '--resolution', type=float, default=0.05, help='地图分辨率 (默认: 0.05)')
    parser.add_argument('--lidar-height', type=float, default=0.4, help='激光雷达安装高度 (默认: 0.4米)')
    parser.add_argument('--robot-height', type=float, default=0.46, help='机器人总高度 (默认: 0.46米)')
    parser.add_argument('--ground-tolerance', type=float, default=0.1, help='地面检测容差 (默认: 0.1米)')
    parser.add_argument('--no-outlier-removal', action='store_true', help='禁用离群点移除')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.pcd_file):
        print(f"错误: PCD文件不存在: {args.pcd_file}")
        return 1
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    # 创建转换器
    converter = CorrectedPCDConverter(
        lidar_height=args.lidar_height,
        robot_height=args.robot_height,
        ground_tolerance=args.ground_tolerance
    )
    
    # 转换地图
    success = converter.convert_to_pgm(
        args.pcd_file, 
        args.output, 
        args.resolution, 
        not args.no_outlier_removal
    )
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())