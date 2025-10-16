#!/usr/bin/env python3

import os
import sys
import argparse
import shutil
import json
from datetime import datetime
import yaml


class MapManager:
    """地图管理类，负责地图的保存、加载、编辑等功能"""
    
    def __init__(self, maps_dir="./maps"):
        self.maps_dir = os.path.abspath(maps_dir)
        self.metadata_file = os.path.join(self.maps_dir, "maps_metadata.json")
        self.ensure_maps_directory()
        self.load_metadata()
    
    def ensure_maps_directory(self):
        """确保地图目录存在"""
        os.makedirs(self.maps_dir, exist_ok=True)
        os.makedirs(os.path.join(self.maps_dir, "pcd"), exist_ok=True)
        os.makedirs(os.path.join(self.maps_dir, "pgm"), exist_ok=True)
        os.makedirs(os.path.join(self.maps_dir, "backup"), exist_ok=True)
    
    def load_metadata(self):
        """加载地图元数据"""
        if os.path.exists(self.metadata_file):
            with open(self.metadata_file, 'r', encoding='utf-8') as f:
                self.metadata = json.load(f)
        else:
            self.metadata = {"maps": []}
    
    def save_metadata(self):
        """保存地图元数据"""
        with open(self.metadata_file, 'w', encoding='utf-8') as f:
            json.dump(self.metadata, f, indent=2, ensure_ascii=False)
    

    def sanitize_map_yaml(self, pgm_dir):
        """确保地图YAML使用标准浮点格式，避免numpy标签"""
        yaml_path = os.path.join(pgm_dir, 'map.yaml')
        if not os.path.exists(yaml_path):
            return
    
        with open(yaml_path, 'r', encoding='utf-8') as f:
            content = f.read()
    
        try:
            data = yaml.safe_load(content)
        except yaml.YAMLError:
            try:
                data = yaml.load(content, Loader=yaml.Loader)  # 兼容历史numpy标签
            except Exception as exc:
                print(f"警告: 无法解析地图YAML: {yaml_path} ({exc})")
                return
    
        def normalize(value):
            if isinstance(value, dict):
                return {key: normalize(val) for key, val in value.items()}
            if isinstance(value, list):
                return [normalize(item) for item in value]
            if isinstance(value, tuple):
                return [normalize(item) for item in value]
            if isinstance(value, (float, int, str, bool)) or value is None:
                return value
            try:
                return float(value)
            except (TypeError, ValueError):
                return value
    
        normalized = normalize(data)
    
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(normalized, f, default_flow_style=False, sort_keys=False)
    
    def add_map(self, pcd_file, name=None, description=""):
        """添加新地图到管理系统中"""
        if not os.path.exists(pcd_file):
            print(f"错误: PCD文件不存在: {pcd_file}")
            return False
        
        # 生成地图名称
        if name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name = f"map_{timestamp}"
        
        # 检查名称是否已存在
        if self.get_map_by_name(name):
            print(f"错误: 地图名称 '{name}' 已存在")
            return False
        
        # 复制PCD文件
        pcd_dest = os.path.join(self.maps_dir, "pcd", f"{name}.pcd")
        shutil.copy2(pcd_file, pcd_dest)
        
        # 转换为PGM格式
        pgm_dir = os.path.join(self.maps_dir, "pgm", name)
        os.makedirs(pgm_dir, exist_ok=True)
        
        # 调用转换脚本
        convert_cmd = f"python3 {os.path.dirname(__file__)}/pcd_to_pgm.py {pcd_dest} -o {pgm_dir}"
        result = os.system(convert_cmd)
        
        if result != 0:
            print(f"错误: PCD转PGM失败")
            os.remove(pcd_dest)
            return False
        
        # 清理YAML格式，避免numpy标签
        self.sanitize_map_yaml(pgm_dir)

        # 添加元数据
        map_info = {
            "name": name,
            "description": description,
            "created_at": datetime.now().isoformat(),
            "pcd_file": pcd_dest,
            "pgm_dir": pgm_dir,
            "size": os.path.getsize(pcd_dest),
            "status": "active"
        }
        
        self.metadata["maps"].append(map_info)
        self.save_metadata()
        
        print(f"地图 '{name}' 添加成功")
        print(f"  PCD文件: {pcd_dest}")
        print(f"  PGM目录: {pgm_dir}")
        
        return True
    
    def get_map_by_name(self, name):
        """根据名称获取地图信息"""
        for map_info in self.metadata["maps"]:
            if map_info["name"] == name:
                return map_info
        return None
    
    def list_maps(self):
        """列出所有地图"""
        if not self.metadata["maps"]:
            print("没有找到任何地图")
            return
        
        print(f"{'名称':<20} {'描述':<30} {'创建时间':<20} {'状态':<10} {'大小(MB)':<10}")
        print("-" * 90)
        
        for map_info in self.metadata["maps"]:
            size_mb = map_info["size"] / (1024 * 1024)
            created_at = map_info["created_at"][:19].replace('T', ' ')
            print(f"{map_info['name']:<20} {map_info['description']:<30} {created_at:<20} {map_info['status']:<10} {size_mb:<10.2f}")
    
    def delete_map(self, name):
        """删除地图"""
        map_info = self.get_map_by_name(name)
        if not map_info:
            print(f"错误: 地图 '{name}' 不存在")
            return False
        
        # 备份地图
        backup_dir = os.path.join(self.maps_dir, "backup", name)
        os.makedirs(backup_dir, exist_ok=True)
        
        if os.path.exists(map_info["pcd_file"]):
            shutil.copy2(map_info["pcd_file"], backup_dir)
        if os.path.exists(map_info["pgm_dir"]):
            shutil.copytree(map_info["pgm_dir"], os.path.join(backup_dir, "pgm"))
        
        # 删除原文件
        if os.path.exists(map_info["pcd_file"]):
            os.remove(map_info["pcd_file"])
        if os.path.exists(map_info["pgm_dir"]):
            shutil.rmtree(map_info["pgm_dir"])
        
        # 更新元数据
        self.metadata["maps"] = [m for m in self.metadata["maps"] if m["name"] != name]
        self.save_metadata()
        
        print(f"地图 '{name}' 已删除并备份到: {backup_dir}")
        return True
    
    def restore_map(self, name):
        """从备份恢复地图"""
        backup_dir = os.path.join(self.maps_dir, "backup", name)
        if not os.path.exists(backup_dir):
            print(f"错误: 备份目录不存在: {backup_dir}")
            return False
        
        # 检查是否已存在同名地图
        if self.get_map_by_name(name):
            print(f"错误: 地图 '{name}' 已存在")
            return False
        
        # 恢复文件
        pcd_file = os.path.join(backup_dir, f"{name}.pcd")
        pgm_dir = os.path.join(backup_dir, "pgm")
        
        if os.path.exists(pcd_file):
            pcd_dest = os.path.join(self.maps_dir, "pcd", f"{name}.pcd")
            shutil.copy2(pcd_file, pcd_dest)
        else:
            print(f"错误: 备份中找不到PCD文件")
            return False
        
        if os.path.exists(pgm_dir):
            pgm_dest = os.path.join(self.maps_dir, "pgm", name)
            shutil.copytree(pgm_dir, pgm_dest)
        
        # 清理YAML格式，避免numpy标签
        self.sanitize_map_yaml(pgm_dir)

        # 添加元数据
        map_info = {
            "name": name,
            "description": f"从备份恢复 - {datetime.now().isoformat()}",
            "created_at": datetime.now().isoformat(),
            "pcd_file": pcd_dest,
            "pgm_dir": pgm_dest,
            "size": os.path.getsize(pcd_dest),
            "status": "active"
        }
        
        self.metadata["maps"].append(map_info)
        self.save_metadata()
        
        print(f"地图 '{name}' 已从备份恢复")
        return True
    
    def export_map(self, name, export_dir):
        """导出地图到指定目录"""
        map_info = self.get_map_by_name(name)
        if not map_info:
            print(f"错误: 地图 '{name}' 不存在")
            return False
        
        os.makedirs(export_dir, exist_ok=True)
        
        # 复制PCD文件
        pcd_dest = os.path.join(export_dir, f"{name}.pcd")
        shutil.copy2(map_info["pcd_file"], pcd_dest)
        
        # 复制PGM文件
        if os.path.exists(map_info["pgm_dir"]):
            pgm_dest = os.path.join(export_dir, name)
            shutil.copytree(map_info["pgm_dir"], pgm_dest)
        
        print(f"地图 '{name}' 已导出到: {export_dir}")
        return True
    
    def set_active_map(self, name):
        """设置活动地图"""
        map_info = self.get_map_by_name(name)
        if not map_info:
            print(f"错误: 地图 '{name}' 不存在")
            return False
        
        # 将所有地图状态设为非活动
        for m in self.metadata["maps"]:
            m["status"] = "inactive"
        
        # 设置指定地图为活动
        map_info["status"] = "active"
        self.save_metadata()
        
        print(f"地图 '{name}' 已设置为活动地图")
        return True
    
    def get_active_map(self):
        """获取当前活动地图"""
        for map_info in self.metadata["maps"]:
            if map_info["status"] == "active":
                return map_info
        return None


def main():
    parser = argparse.ArgumentParser(description='地图管理工具')
    parser.add_argument('--maps-dir', default='./maps', help='地图目录 (默认: ./maps)')
    
    subparsers = parser.add_subparsers(dest='command', help='可用命令')
    
    # 添加地图命令
    add_parser = subparsers.add_parser('add', help='添加新地图')
    add_parser.add_argument('pcd_file', help='PCD文件路径')
    add_parser.add_argument('--name', help='地图名称')
    add_parser.add_argument('--description', default='', help='地图描述')
    
    # 列出地图命令
    subparsers.add_parser('list', help='列出所有地图')
    
    # 删除地图命令
    delete_parser = subparsers.add_parser('delete', help='删除地图')
    delete_parser.add_argument('name', help='地图名称')
    
    # 恢复地图命令
    restore_parser = subparsers.add_parser('restore', help='从备份恢复地图')
    restore_parser.add_argument('name', help='地图名称')
    
    # 导出地图命令
    export_parser = subparsers.add_parser('export', help='导出地图')
    export_parser.add_argument('name', help='地图名称')
    export_parser.add_argument('export_dir', help='导出目录')
    
    # 设置活动地图命令
    active_parser = subparsers.add_parser('set-active', help='设置活动地图')
    active_parser.add_argument('name', help='地图名称')
    
    # 获取活动地图命令
    subparsers.add_parser('get-active', help='获取当前活动地图')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    # 创建地图管理器
    manager = MapManager(args.maps_dir)
    
    # 执行命令
    if args.command == 'add':
        success = manager.add_map(args.pcd_file, args.name, args.description)
    elif args.command == 'list':
        manager.list_maps()
        success = True
    elif args.command == 'delete':
        success = manager.delete_map(args.name)
    elif args.command == 'restore':
        success = manager.restore_map(args.name)
    elif args.command == 'export':
        success = manager.export_map(args.name, args.export_dir)
    elif args.command == 'set-active':
        success = manager.set_active_map(args.name)
    elif args.command == 'get-active':
        active_map = manager.get_active_map()
        if active_map:
            print(f"当前活动地图: {active_map['name']}")
            print(f"  描述: {active_map['description']}")
            print(f"  创建时间: {active_map['created_at']}")
            print(f"  PCD文件: {active_map['pcd_file']}")
            print(f"  PGM目录: {active_map['pgm_dir']}")
        else:
            print("没有设置活动地图")
        success = True
    else:
        parser.print_help()
        return 1
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
