#!/bin/bash
# 查看 RTAB-Map 地图的快速脚本
# View existing RTAB-Map with RViz

set -e

echo "=========================================="
echo "  RTAB-Map 地图查看器"
echo "=========================================="
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check database
DB_PATH="${HOME}/.ros/rtabmap.db"
if [ ! -f "$DB_PATH" ]; then
  echo "❌ 未找到地图数据库: $DB_PATH"
  echo "   请先运行建图: ./scripts/test_full_mapping.sh"
  exit 1
fi

DB_SIZE=$(du -h "$DB_PATH" | cut -f1)
echo "✓ 找到地图数据库: $DB_PATH"
echo "  大小: $DB_SIZE"
echo ""

# Check if bringup is running
if ! ros2 node list 2>/dev/null | grep -q "robot_state_publisher"; then
  echo "⚠️  基础系统未运行"
  echo "   将以定位模式启动（不会修改地图）"
  echo ""

  NEED_BRINGUP=true
else
  echo "✓ 基础系统已运行"
  NEED_BRINGUP=false
fi

echo "启动模式:"
echo "  - 定位模式 (Localization): 只查看地图，不修改"
echo "  - 地图数据库: $DB_PATH"
echo ""

cleanup() {
  if [ -n "$RTABMAP_PID" ] && kill -0 "$RTABMAP_PID" 2>/dev/null; then
    kill "$RTABMAP_PID" 2>/dev/null || true
    wait "$RTABMAP_PID" 2>/dev/null || true
  fi
  if [ -n "$RVIZ_PID" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  if [ "$NEED_BRINGUP" = "true" ] && [ -n "$BRINGUP_PID" ]; then
    if kill -0 "$BRINGUP_PID" 2>/dev/null; then
      kill "$BRINGUP_PID" 2>/dev/null || true
      wait "$BRINGUP_PID" 2>/dev/null || true
    fi
  fi
}

trap cleanup EXIT

# Start bringup if needed
if [ "$NEED_BRINGUP" = "true" ]; then
  echo "[1/3] 启动基础系统..."
  ros2 launch t_robot_bringup bringup.launch.py > /tmp/view_map_bringup.log 2>&1 &
  BRINGUP_PID=$!
  echo "  等待 8 秒初始化..."
  sleep 8
  echo "  ✓ 基础系统已启动"
else
  echo "[1/3] 跳过基础系统启动（已在运行）"
fi

echo ""
echo "[2/3] 以定位模式启动 RTAB-Map..."
ros2 launch t_robot_slam mapping.launch.py \
  launch_bringup:=false \
  delete_db:=false \
  > /tmp/view_map_rtabmap.log 2>&1 &
RTABMAP_PID=$!

echo "  等待 10 秒 RTAB-Map 加载地图..."
sleep 10

# Check if rtabmap started
if ! ros2 node list 2>/dev/null | grep -q rtabmap; then
  echo "  ❌ RTAB-Map 启动失败"
  echo "  查看日志: tail -f /tmp/view_map_rtabmap.log"
  exit 1
fi

echo "  ✓ RTAB-Map 已启动"

echo ""
echo "[3/3] 启动 RViz 可视化..."
RVIZ_CONFIG="src/t_robot_slam/rviz/mapping.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
  echo "  ⚠️  未找到 RViz 配置: $RVIZ_CONFIG"
  echo "  使用默认配置启动..."
  ros2 run rviz2 rviz2 > /tmp/view_map_rviz.log 2>&1 &
else
  ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" > /tmp/view_map_rviz.log 2>&1 &
fi

RVIZ_PID=$!
sleep 3

if kill -0 "$RVIZ_PID" 2>/dev/null; then
  echo "  ✓ RViz 已启动"
else
  echo "  ❌ RViz 启动失败"
  exit 1
fi

echo ""
echo "=========================================="
echo "  地图查看器已就绪"
echo "=========================================="
echo ""
echo "RViz 显示说明:"
echo "  - 白色点云: 已建立的3D地图"
echo "  - 灰色栅格: 2D占用栅格地图"
echo "  - 绿色路径: 历史轨迹"
echo "  - 蓝色球: SLAM图节点"
echo "  - 机器人模型: 当前位置"
echo ""
echo "查看地图信息:"
echo "  ros2 topic echo /rtabmap/info --once"
echo ""
echo "导出地图:"
echo "  ./src/t_robot_slam/scripts/export_rtabmap.py \\"
echo "    ~/.ros/rtabmap.db -o maps/exported --3d --voxel 0.05"
echo ""
echo "日志文件:"
if [ "$NEED_BRINGUP" = "true" ]; then
  echo "  Bringup: /tmp/view_map_bringup.log"
fi
echo "  RTAB-Map: /tmp/view_map_rtabmap.log"
echo "  RViz: /tmp/view_map_rviz.log"
echo ""
echo "按 Ctrl+C 关闭查看器"
echo "=========================================="

# Wait for user to stop
wait "$RVIZ_PID" 2>/dev/null || true

echo ""
echo "地图查看器已关闭"
