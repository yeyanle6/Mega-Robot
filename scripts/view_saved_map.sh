#!/bin/bash
# 查看已保存的点云地图（离线查看）
# View saved point cloud maps (offline)

echo "=========================================="
echo "  点云地图查看器（离线模式）"
echo "=========================================="
echo ""

# Find map files
echo "搜索地图文件..."
mapfiles=($(find maps/ -name "*.ply" -o -name "*.pcd" 2>/dev/null))

if [ ${#mapfiles[@]} -eq 0 ]; then
  echo "❌ 未找到地图文件"
  echo ""
  echo "可能的原因："
  echo "  1. 尚未导出地图"
  echo "  2. 地图文件在其他位置"
  echo ""
  echo "导出地图："
  echo "  ./src/t_robot_slam/scripts/export_rtabmap.py \\"
  echo "    ~/.ros/rtabmap.db -o maps/my_map.pcd --3d --voxel 0.05"
  exit 1
fi

echo "找到 ${#mapfiles[@]} 个地图文件："
echo ""
for i in "${!mapfiles[@]}"; do
  size=$(du -h "${mapfiles[$i]}" | cut -f1)
  echo "  [$i] ${mapfiles[$i]} (${size})"
done
echo ""

# Select map to view
if [ ${#mapfiles[@]} -eq 1 ]; then
  selected=0
  echo "自动选择: ${mapfiles[0]}"
else
  read -p "选择要查看的地图编号 [0]: " selected
  selected=${selected:-0}
fi

mapfile="${mapfiles[$selected]}"

if [ ! -f "$mapfile" ]; then
  echo "❌ 文件不存在: $mapfile"
  exit 1
fi

echo ""
echo "查看地图: $mapfile"
echo ""

# Check if pcl_viewer is available
if command -v pcl_viewer >/dev/null 2>&1; then
  echo "使用 PCL Viewer 查看..."
  echo ""
  echo "操作提示："
  echo "  - 鼠标左键拖动: 旋转"
  echo "  - 鼠标滚轮: 缩放"
  echo "  - 鼠标中键: 平移"
  echo "  - r: 重置视角"
  echo "  - j: 截图"
  echo "  - q: 退出"
  echo ""
  sleep 2
  pcl_viewer "$mapfile"
elif command -v cloudcompare >/dev/null 2>&1; then
  echo "使用 CloudCompare 查看..."
  cloudcompare "$mapfile"
else
  echo "⚠️  未安装点云查看工具"
  echo ""
  echo "安装方法："
  echo "  sudo apt install pcl-tools"
  echo "  或"
  echo "  sudo apt install cloudcompare"
  echo ""
  echo "临时解决方案 - 使用RViz查看（需要ROS运行）:"
  echo "  1. 启动RViz: ros2 run rviz2 rviz2"
  echo "  2. Add -> PointCloud2"
  echo "  3. 在Topic中手动加载PCD文件"
  exit 1
fi
