#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import yaml
from PIL import Image
import open3d as o3d


def detect_ground_height(points, tolerance=0.1):
    """
    自动检测地面高度
    
    Args:
        points: 点云数据 (Nx3 numpy数组)
        tolerance: 地面高度容差 (米)
    
    Returns:
        ground_height: 检测到的地面高度 (米)
    """
    # 获取Z坐标
    z_coords = points[:, 2]
    
    # 使用直方图方法检测地面
    # 创建Z坐标的直方图
    hist, bin_edges = np.histogram(z_coords, bins=100)
    
    # 找到最频繁的高度区间（假设地面点最多）
    max_count_idx = np.argmax(hist)
    ground_height = (bin_edges[max_count_idx] + bin_edges[max_count_idx + 1]) / 2
    
    # 进一步优化：在检测到的地面高度附近寻找更精确的地面
    ground_candidates = z_coords[(z_coords >= ground_height - tolerance) & 
                                (z_coords <= ground_height + tolerance)]
    
    if len(ground_candidates) > 0:
        # 使用中位数作为更稳定的地面高度估计
        ground_height = np.median(ground_candidates)
    
    return ground_height


def pcd_to_pgm(pcd_file, output_dir, resolution=0.05, height_min=None, height_max=None, 
               auto_ground_detection=True, ground_height_tolerance=0.1):
    """
    将PCD点云文件转换为PGM栅格地图和YAML配置文件
    
    Args:
        pcd_file: 输入PCD文件路径
        output_dir: 输出目录
        resolution: 地图分辨率 (米/像素)
        height_min: 最小高度阈值 (如果为None则自动检测)
        height_max: 最大高度阈值 (如果为None则自动检测)
        auto_ground_detection: 是否启用自动地面检测
        ground_height_tolerance: 地面高度容差 (米)
    """
    
    # 读取PCD文件
    print(f"读取PCD文件: {pcd_file}")
    pcd = o3d.io.read_point_cloud(pcd_file)
    
    if len(pcd.points) == 0:
        print("错误: PCD文件为空或无法读取")
        return False
    
    # 获取点云数据
    points = np.asarray(pcd.points)
    
    # 自动地面检测
    if auto_ground_detection and (height_min is None or height_max is None):
        print("正在进行自动地面检测...")
        ground_height = detect_ground_height(points, ground_height_tolerance)
        print(f"检测到地面高度: {ground_height:.3f} 米")
        
        if height_min is None:
            height_min = ground_height - 0.1  # 地面以下10cm
        if height_max is None:
            height_max = ground_height + 0.4  # 地面以上40cm (适合机器人导航)
            
        print(f"使用高度范围: [{height_min:.3f}, {height_max:.3f}] 米")
    else:
        # 使用默认值或用户指定值
        if height_min is None:
            height_min = -0.1
        if height_max is None:
            height_max = 0.4
        print(f"使用指定高度范围: [{height_min:.3f}, {height_max:.3f}] 米")
    
    # 过滤高度范围内的点
    mask = (points[:, 2] >= height_min) & (points[:, 2] <= height_max)
    filtered_points = points[mask]
    
    print(f"原始点数: {len(points)}")
    print(f"过滤后点数: {len(filtered_points)}")
    
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
        'free_thresh': 0.196
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
    
    return True


def main():
    parser = argparse.ArgumentParser(description='将PCD点云文件转换为PGM栅格地图')
    parser.add_argument('pcd_file', help='输入PCD文件路径')
    parser.add_argument('-o', '--output', default='./maps', help='输出目录 (默认: ./maps)')
    parser.add_argument('-r', '--resolution', type=float, default=0.05, help='地图分辨率 (默认: 0.05)')
    parser.add_argument('--height-min', type=float, default=None, help='最小高度阈值 (默认: 自动检测)')
    parser.add_argument('--height-max', type=float, default=None, help='最大高度阈值 (默认: 自动检测)')
    parser.add_argument('--no-auto-ground', action='store_true', help='禁用自动地面检测')
    parser.add_argument('--ground-tolerance', type=float, default=0.1, help='地面检测容差 (默认: 0.1米)')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.pcd_file):
        print(f"错误: PCD文件不存在: {args.pcd_file}")
        return 1
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    # 转换地图
    success = pcd_to_pgm(
        args.pcd_file, 
        args.output, 
        args.resolution, 
        args.height_min, 
        args.height_max,
        not args.no_auto_ground,
        args.ground_tolerance
    )
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())