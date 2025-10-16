#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import yaml
from PIL import Image
import open3d as o3d


class SimpleAdvancedPCDConverter:
    """简化版高级PCD转换器，不依赖scipy"""
    
    def __init__(self, lidar_height=0.625, ground_tolerance=0.1):
        """
        初始化转换器
        
        Args:
            lidar_height: 激光雷达安装高度 (米)
            ground_tolerance: 地面检测容差 (米)
        """
        self.lidar_height = lidar_height
        self.ground_tolerance = ground_tolerance
        
    def detect_ground_advanced(self, points, method='histogram'):
        """
        高级地面检测算法
        
        Args:
            points: 点云数据 (Nx3 numpy数组)
            method: 检测方法 ('histogram', 'clustering')
        
        Returns:
            ground_height: 检测到的地面高度 (米)
        """
        z_coords = points[:, 2]
        
        if method == 'histogram':
            return self._detect_ground_histogram(z_coords)
        elif method == 'clustering':
            return self._detect_ground_clustering(points)
        else:
            raise ValueError(f"未知的地面检测方法: {method}")
    
    def _detect_ground_histogram(self, z_coords):
        """使用直方图方法检测地面"""
        # 创建Z坐标的直方图
        hist, bin_edges = np.histogram(z_coords, bins=200)
        
        # 找到最频繁的高度区间
        max_count_idx = np.argmax(hist)
        ground_height = (bin_edges[max_count_idx] + bin_edges[max_count_idx + 1]) / 2
        
        # 在检测到的地面高度附近寻找更精确的地面
        ground_candidates = z_coords[(z_coords >= ground_height - self.ground_tolerance) & 
                                    (z_coords <= ground_height + self.ground_tolerance)]
        
        if len(ground_candidates) > 0:
            ground_height = np.median(ground_candidates)
        
        return ground_height
    
    def _detect_ground_clustering(self, points):
        """使用简单聚类方法检测地面"""
        z_coords = points[:, 2]
        
        # 简单的基于密度的聚类
        # 将Z坐标分成多个区间，找到密度最高的区间
        z_min, z_max = np.min(z_coords), np.max(z_coords)
        bin_size = 0.05  # 5cm bins
        bins = np.arange(z_min, z_max + bin_size, bin_size)
        
        hist, _ = np.histogram(z_coords, bins=bins)
        max_density_idx = np.argmax(hist)
        ground_height = bins[max_density_idx] + bin_size / 2
        
        return ground_height
    
    def filter_points_by_height(self, points, height_min=None, height_max=None, 
                               auto_ground_detection=True, ground_method='histogram'):
        """
        根据高度过滤点云
        
        Args:
            points: 点云数据 (Nx3 numpy数组)
            height_min: 最小高度阈值
            height_max: 最大高度阈值
            auto_ground_detection: 是否启用自动地面检测
            ground_method: 地面检测方法
        
        Returns:
            filtered_points: 过滤后的点云
            height_range: 使用的高度范围
        """
        if auto_ground_detection and (height_min is None or height_max is None):
            print("正在进行高级地面检测...")
            ground_height = self.detect_ground_advanced(points, ground_method)
            print(f"检测到地面高度: {ground_height:.3f} 米")
            
            if height_min is None:
                height_min = ground_height - 0.1  # 地面以下10cm
            if height_max is None:
                height_max = ground_height + 0.4  # 地面以上40cm
                
            print(f"使用高度范围: [{height_min:.3f}, {height_max:.3f}] 米")
        else:
            if height_min is None:
                height_min = -0.1
            if height_max is None:
                height_max = 0.4
            print(f"使用指定高度范围: [{height_min:.3f}, {height_max:.3f}] 米")
        
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
                      height_min=None, height_max=None, auto_ground_detection=True,
                      ground_method='histogram', remove_outliers_flag=True):
        """
        将PCD文件转换为PGM栅格地图
        
        Args:
            pcd_file: 输入PCD文件路径
            output_dir: 输出目录
            resolution: 地图分辨率 (米/像素)
            height_min: 最小高度阈值
            height_max: 最大高度阈值
            auto_ground_detection: 是否启用自动地面检测
            ground_method: 地面检测方法
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
        
        # 过滤高度范围内的点
        filtered_points, height_range = self.filter_points_by_height(
            points, height_min, height_max, auto_ground_detection, ground_method)
        
        print(f"高度过滤后点数: {len(filtered_points)}")
        
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
            'resolution': resolution,
            'origin': [x_min, y_min, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'height_range': list(height_range),
            'ground_detection_method': ground_method if auto_ground_detection else 'manual'
        }
        
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
        print(f"保存YAML文件: {yaml_file}")
        
        # 打印地图信息
        print("\n地图信息:")
        print(f"  分辨率: {resolution} 米/像素")
        print(f"  尺寸: {width} x {height} 像素")
        print(f"  物理尺寸: {width * resolution:.2f} x {height * resolution:.2f} 米")
        print(f"  原点: [{x_min:.2f}, {y_min:.2f}, 0.0]")
        print(f"  高度范围: [{height_range[0]:.3f}, {height_range[1]:.3f}] 米")
        
        return True


def main():
    parser = argparse.ArgumentParser(description='简化版高级PCD点云文件转换为PGM栅格地图')
    parser.add_argument('pcd_file', help='输入PCD文件路径')
    parser.add_argument('-o', '--output', default='./maps', help='输出目录 (默认: ./maps)')
    parser.add_argument('-r', '--resolution', type=float, default=0.05, help='地图分辨率 (默认: 0.05)')
    parser.add_argument('--height-min', type=float, default=None, help='最小高度阈值 (默认: 自动检测)')
    parser.add_argument('--height-max', type=float, default=None, help='最大高度阈值 (默认: 自动检测)')
    parser.add_argument('--no-auto-ground', action='store_true', help='禁用自动地面检测')
    parser.add_argument('--ground-method', choices=['histogram', 'clustering'], 
                       default='histogram', help='地面检测方法 (默认: histogram)')
    parser.add_argument('--ground-tolerance', type=float, default=0.1, help='地面检测容差 (默认: 0.1米)')
    parser.add_argument('--lidar-height', type=float, default=0.625, help='激光雷达安装高度 (默认: 0.625米)')
    parser.add_argument('--no-outlier-removal', action='store_true', help='禁用离群点移除')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.pcd_file):
        print(f"错误: PCD文件不存在: {args.pcd_file}")
        return 1
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    # 创建转换器
    converter = SimpleAdvancedPCDConverter(
        lidar_height=args.lidar_height,
        ground_tolerance=args.ground_tolerance
    )
    
    # 转换地图
    success = converter.convert_to_pgm(
        args.pcd_file, 
        args.output, 
        args.resolution, 
        args.height_min, 
        args.height_max,
        not args.no_auto_ground,
        args.ground_method,
        not args.no_outlier_removal
    )
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())