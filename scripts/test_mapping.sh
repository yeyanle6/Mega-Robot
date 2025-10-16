#!/bin/bash
# T-Robot SLAM System - Mapping Test Script
# Tests: Point Cloud Processing → RTAB-Map → Loop Closure

echo "=========================================="
echo "  T-Robot SLAM Mapping Test"
echo "=========================================="
echo ""

# Source ROS2 workspace
set -e
source /opt/ros/humble/setup.bash
source install/setup.bash

cleanup() {
  if [ -n "$MAPPING_PID" ] && kill -0 "$MAPPING_PID" 2>/dev/null; then
    kill "$MAPPING_PID" 2>/dev/null || true
    wait "$MAPPING_PID" 2>/dev/null || true
  fi
  ros2 daemon stop 2>/dev/null || true
  ros2 daemon start >/dev/null 2>&1 || true
}

trap cleanup EXIT

print_topic_hz() {
  local topic="$1"
  local label="$2"
  echo "  ${label}"
  local hz_output
  hz_output=$(timeout 5 ros2 topic hz "$topic" --window 10 2>&1 || true)
  if echo "$hz_output" | grep -q "average rate"; then
    echo "$hz_output" | head -n 3 | sed 's/^/    /'
  else
    echo "    ❌ 未检测到稳定数据"
    if [ -n "$hz_output" ]; then
      echo "$hz_output" | sed 's/^/    /'
    fi
  fi
}

# Check if bringup is running
if ! ros2 node list | grep -q "robot_state_publisher"; then
    echo "❌ ERROR: Bringup system not running!"
    echo "Please run ./test_bringup.sh first"
    exit 1
fi

echo "[1/6] Bringup system detected ✓"
echo ""

echo "[2/6] Launching SLAM mapping pipeline..."
echo "  - Point cloud preprocessor"
echo "  - RTAB-Map SLAM"
echo ""

# Launch mapping in background
ros2 launch t_robot_slam mapping.launch.py launch_bringup:=false > /tmp/mapping_test.log 2>&1 &
MAPPING_PID=$!

echo "Mapping PID: $MAPPING_PID"
echo "Waiting 15 seconds for RTAB-Map initialization..."
sleep 15

echo ""
echo "[3/6] Checking SLAM nodes..."
node_listing=$(ros2 node list)
missing_nodes=0
for required in /pointcloud_preprocessor /rtabmap; do
  if echo "$node_listing" | grep -qx "${required}"; then
    echo "  ✓ ${required}"
  else
    echo "  ❌ 缺少节点: ${required}"
    missing_nodes=1
  fi
done
if [ ${missing_nodes} -ne 0 ]; then
  echo "❌ 映射所需节点未全部就绪，退出。"
  exit 1
fi

echo ""
echo "[4/6] Verifying point cloud processing chain..."
echo "  Input: /mid360/lidar"
timeout 3 ros2 topic info /mid360/lidar 2>/dev/null || echo "  ❌ Missing /mid360/lidar"

echo ""
echo "  Processed: /mid360/points_filtered"
timeout 3 ros2 topic info /mid360/points_filtered 2>/dev/null || echo "  ❌ Missing /mid360/points_filtered"

echo ""
echo "  Obstacles: /cloud/obstacles"
timeout 3 ros2 topic info /cloud/obstacles 2>/dev/null || echo "  ❌ Missing /cloud/obstacles"

echo ""
echo "[5/6] Monitoring point cloud processing performance..."
print_topic_hz /mid360/lidar 'Original point cloud frequency:'

echo ""
print_topic_hz /mid360/points_filtered 'Filtered point cloud frequency:'

echo ""
echo "  Checking point count reduction (sample)..."
raw_sample=$(timeout 3 ros2 topic echo /mid360/lidar --once 2>/dev/null || true)
filtered_sample=$(timeout 3 ros2 topic echo /mid360/points_filtered --once 2>/dev/null || true)
if [ -n "$raw_sample" ]; then
  echo "$raw_sample" | grep -E "width:|height:" | head -2 || true
else
  echo "    ❌ 无法获取 /mid360/lidar 样本"
fi
if [ -n "$filtered_sample" ]; then
  echo "$filtered_sample" | grep -E "width:|height:" | head -2 || true
else
  echo "    ❌ 无法获取 /mid360/points_filtered 样本"
fi

echo ""
echo "[6/6] RTAB-Map status check..."
echo "  Checking if RTAB-Map is publishing map data..."
topic_listing=$(timeout 5 ros2 topic list 2>/dev/null || true)
map_topics=$(echo "$topic_listing" | grep -E '^/(mapData|rtabmap)' || true)
if [ -n "$map_topics" ]; then
  echo "$map_topics" | sed 's/^/  ✓ /'
else
  echo "  ℹ️  尚未检测到地图话题（请移动机器人或等待初始化）"
fi

echo ""
echo "=========================================="
echo "  Mapping Test Instructions"
echo "=========================================="
echo ""
echo "✅ System is ready for mapping!"
echo ""
echo "Next steps:"
echo "1. Move the robot slowly in the environment"
echo "2. Monitor RTAB-Map info:"
echo "   ros2 topic echo /rtabmap/info --once"
echo ""
echo "3. Check loop closure detection:"
echo "   ros2 topic echo /rtabmap/info | grep -i loop"
echo ""
echo "4. Monitor odometry drift:"
echo "   ros2 run t_robot_slam slam_monitor.py"
echo ""
echo "5. Record test data:"
echo "   ros2 run t_robot_slam mapping_recorder.py --duration 300"
echo ""
echo "Logs:"
echo "  Bringup: /tmp/bringup_test.log"
echo "  Mapping: /tmp/mapping_test.log"
echo ""
echo "按 Ctrl+C 停止映射测试，脚本将自动清理相关节点。"

# 保持 mapping 运行，等待用户手动终止
wait "$MAPPING_PID" 2>/dev/null || true

echo "映射测试结束，资源已清理。"
echo "=========================================="
