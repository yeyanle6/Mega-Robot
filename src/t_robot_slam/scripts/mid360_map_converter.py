#!/usr/bin/env python3

import os
import sys
import argparse
import yaml
import numpy as np
from advanced_pcd_converter import AdvancedPCDConverter


def load_config(config_file):
    """加载配置文件"""
    with open(config_file, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description='MID360专用地图转换工具')
    parser.add_argument('pcd_file', help='输入PCD文件路径')
    parser.add_argument('-o', '--output', default='./maps', help='输出目录 (默认: ./maps)')
    parser.add_argument('-c', '--config', default='../../config/mid360_map_config.yaml', 
                       help='配置文件路径 (默认: ../../config/mid360_map_config.yaml)')
    parser.add_argument('--override-height-min', type=float, help='覆盖最小高度设置')
    parser.add_argument('--override-height-max', type=float, help='覆盖最大高度设置')
    parser.add_argument('--override-resolution', type=float, help='覆盖分辨率设置')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.pcd_file):
        print(f"错误: PCD文件不存在: {args.pcd_file}")
        return 1
    
    # 检查配置文件
    if not os.path.exists(args.config):
        print(f"错误: 配置文件不存在: {args.config}")
        return 1
    
    # 加载配置
    print(f"加载配置文件: {args.config}")
    config = load_config(args.config)
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    # 创建转换器
    converter = AdvancedPCDConverter(
        lidar_height=config['lidar_mounting']['height'],
        ground_tolerance=config['ground_detection']['tolerance']
    )
    
    # 获取配置参数
    height_min = args.override_height_min
    height_max = args.override_height_max
    resolution = args.override_resolution or config['map_generation']['resolution']
    
    # 如果启用了自动地面检测
    auto_ground_detection = config['height_filtering']['auto_detection']
    if not auto_ground_detection:
        height_min = height_min or config['height_filtering']['manual_min']
        height_max = height_max or config['height_filtering']['manual_max']
    
    # 地面检测方法
    ground_method = config['ground_detection']['method']
    
    # 离群点移除
    remove_outliers = config['point_cloud_processing']['remove_outliers']
    
    print("\n=== MID360地图转换配置 ===")
    print(f"激光雷达高度: {config['lidar_mounting']['height']} 米")
    print(f"自动地面检测: {'启用' if auto_ground_detection else '禁用'}")
    if auto_ground_detection:
        print(f"地面检测方法: {ground_method}")
        print(f"地面检测容差: {config['ground_detection']['tolerance']} 米")
    print(f"地图分辨率: {resolution} 米/像素")
    print(f"移除离群点: {'启用' if remove_outliers else '禁用'}")
    print("=" * 30)
    
    # 转换地图
    success = converter.convert_to_pgm(
        args.pcd_file, 
        args.output, 
        resolution, 
        height_min, 
        height_max,
        auto_ground_detection,
        ground_method,
        remove_outliers
    )
    
    if success:
        print("\n✅ 地图转换完成!")
        print(f"输出目录: {args.output}")
        print("生成文件:")
        print(f"  - map.pgm (栅格地图)")
        print(f"  - map.yaml (地图配置)")
    else:
        print("\n❌ 地图转换失败!")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())