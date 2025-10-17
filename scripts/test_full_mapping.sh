#!/bin/bash
# T-Robot SLAM System - Full Mapping Test with Visualization
# Complete workflow: Bringup → Mapping → Visualization → Export
# 完整流程：基础启动 → 建图 → 可视化 → 地图导出

set -e

# ============================================================
# Color definitions for better readability
# ============================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
  echo ""
  echo "=========================================="
  echo -e "  ${GREEN}$1${NC}"
  echo "=========================================="
  echo ""
}

print_step() {
  echo ""
  echo -e "${BLUE}[$1]${NC} $2"
  echo ""
}

print_success() {
  echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
  echo -e "${YELLOW}⚠️${NC} $1"
}

print_error() {
  echo -e "${RED}✗${NC} $1"
}

# ============================================================
# Cleanup function
# ============================================================
BRINGUP_PID=""
MAPPING_PID=""
RVIZ_PID=""
MICRO_ROS_PID=""
MICRO_ROS_STARTED=0

cleanup() {
  echo ""
  print_header "清理资源..."

  if [ -n "$RVIZ_PID" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "停止 RViz..."
    kill "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi

  if [ -n "$MAPPING_PID" ] && kill -0 "$MAPPING_PID" 2>/dev/null; then
    echo "停止建图系统..."
    kill "$MAPPING_PID" 2>/dev/null || true
    wait "$MAPPING_PID" 2>/dev/null || true
  fi

  if [ -n "$BRINGUP_PID" ] && kill -0 "$BRINGUP_PID" 2>/dev/null; then
    echo "停止基础系统..."
    kill "$BRINGUP_PID" 2>/dev/null || true
    wait "$BRINGUP_PID" 2>/dev/null || true
  fi

  if [ "${MICRO_ROS_STARTED}" = "1" ] && [ -n "$MICRO_ROS_PID" ]; then
    if kill -0 "$MICRO_ROS_PID" 2>/dev/null; then
      echo "停止 micro-ROS Agent..."
      kill "$MICRO_ROS_PID" 2>/dev/null || true
      wait "$MICRO_ROS_PID" 2>/dev/null || true
    fi
  fi

  ros2 daemon stop 2>/dev/null || true
  ros2 daemon start >/dev/null 2>&1 || true

  print_success "资源清理完成"
}

trap cleanup EXIT

# ============================================================
# Configuration
# ============================================================
print_header "T-Robot SLAM 完整建图测试"

# Environment variables
DELETE_DB=${DELETE_DB:-false}
LAUNCH_RVIZ=${LAUNCH_RVIZ:-true}
AUTO_EXPORT=${AUTO_EXPORT:-false}
EXPORT_VOXEL=${EXPORT_VOXEL:-0.05}
MICRO_ROS_AGENT_DEV=${MICRO_ROS_AGENT_DEV:-/dev/ttyUSB0}
MICRO_ROS_AGENT_ARGS=${MICRO_ROS_AGENT_ARGS:-"serial --dev ${MICRO_ROS_AGENT_DEV} -v4"}

# Directories and files
DB_PATH="${HOME}/.ros/rtabmap.db"
BRINGUP_LOG="/tmp/full_mapping_bringup.log"
MAPPING_LOG="/tmp/full_mapping_mapping.log"
RVIZ_LOG="/tmp/full_mapping_rviz.log"
MICRO_ROS_LOG="/tmp/full_mapping_micro_ros.log"
EXPORT_DIR="./maps/$(date +%Y%m%d_%H%M%S)"

echo "配置选项:"
echo "  - DELETE_DB: $DELETE_DB (是否删除旧地图)"
echo "  - LAUNCH_RVIZ: $LAUNCH_RVIZ (是否启动可视化)"
echo "  - AUTO_EXPORT: $AUTO_EXPORT (结束时是否自动导出)"
echo "  - EXPORT_VOXEL: $EXPORT_VOXEL (导出时体素大小)"
echo "  - MICRO_ROS_AGENT_DEV: $MICRO_ROS_AGENT_DEV"
echo ""

# ============================================================
# Step 0: Source ROS 2 workspace
# ============================================================
print_step "0/8" "初始化 ROS 2 环境"

source /opt/ros/humble/setup.bash

if [ -f "install/setup.bash" ]; then
  source install/setup.bash
  print_success "工作空间已加载"
else
  print_error "未找到 install/setup.bash，请先编译工作空间"
  exit 1
fi

# ============================================================
# Step 1: Launch micro-ROS Agent (if needed)
# ============================================================
print_step "1/8" "启动 micro-ROS Agent (底盘通讯)"

if ros2 node list 2>/dev/null | grep -q micro_ros_agent; then
  print_success "micro-ROS Agent 已在运行，跳过启动"
else
  if [ -c "$MICRO_ROS_AGENT_DEV" ]; then
    if [ -f "/home/wang/uros_ws/install/setup.bash" ]; then
      echo "启动 micro-ROS Agent: ${MICRO_ROS_AGENT_ARGS}"
      (
        source /opt/ros/humble/setup.bash
        source /home/wang/uros_ws/install/setup.bash
        ros2 run micro_ros_agent micro_ros_agent ${MICRO_ROS_AGENT_ARGS}
      ) >"${MICRO_ROS_LOG}" 2>&1 &
      MICRO_ROS_PID=$!
      MICRO_ROS_STARTED=1
      echo "  PID: $MICRO_ROS_PID"
      echo "  日志: ${MICRO_ROS_LOG}"
      sleep 3
      print_success "micro-ROS Agent 已启动"
    else
      print_warning "未找到 uros_ws，micro-ROS Agent 未启动"
      echo "  手动启动: source ~/uros_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent ${MICRO_ROS_AGENT_ARGS}"
    fi
  else
    print_warning "未检测到设备 ${MICRO_ROS_AGENT_DEV}"
    echo "  如需底盘里程计，请检查USB连接"
  fi
fi

# ============================================================
# Step 2: Launch bringup system
# ============================================================
print_step "2/8" "启动基础系统 (Bringup)"

echo "启动组件:"
echo "  - 底盘驱动 (Megarover3)"
echo "  - MID360 LiDAR"
echo "  - IMU 数据中继"
echo "  - EKF 状态估计"
echo "  - 静态 TF 发布器"
echo ""

ros2 launch t_robot_bringup bringup.launch.py > "$BRINGUP_LOG" 2>&1 &
BRINGUP_PID=$!

echo "Bringup PID: $BRINGUP_PID"
echo "等待 10 秒初始化..."
sleep 10

# Verify bringup
if ! ros2 node list | grep -q "robot_state_publisher"; then
  print_error "基础系统启动失败"
  echo "请查看日志: $BRINGUP_LOG"
  exit 1
fi

print_success "基础系统启动成功"

# ============================================================
# Step 3: Check database
# ============================================================
print_step "3/8" "检查 RTAB-Map 数据库"

if [ -f "$DB_PATH" ]; then
  DB_SIZE=$(du -h "$DB_PATH" | cut -f1)
  echo "已存在数据库: $DB_PATH"
  echo "  大小: $DB_SIZE"

  if [ "$DELETE_DB" = "true" ]; then
    print_warning "将删除旧数据库，创建新地图"
  else
    echo "  继续使用现有地图（追加模式）"
    echo "  如需新建地图，设置: DELETE_DB=true"
  fi
else
  print_success "创建新数据库: $DB_PATH"
  DELETE_DB=true
fi

# ============================================================
# Step 4: Launch mapping system
# ============================================================
print_step "4/8" "启动建图系统 (RTAB-Map)"

echo "启动组件:"
echo "  - 点云预处理器 (30°倾角优化)"
echo "  - RTAB-Map SLAM"
echo "  - 地面/障碍物分割"
echo ""

ros2 launch t_robot_slam mapping.launch.py \
  launch_bringup:=false \
  delete_db:=$DELETE_DB \
  > "$MAPPING_LOG" 2>&1 &
MAPPING_PID=$!

echo "Mapping PID: $MAPPING_PID"
echo "等待 15 秒 RTAB-Map 初始化..."
sleep 15

# Verify mapping
node_missing=0
for required in /pointcloud_preprocessor /rtabmap; do
  if ros2 node list | grep -qx "${required}"; then
    print_success "节点已启动: ${required}"
  else
    print_error "节点缺失: ${required}"
    node_missing=1
  fi
done

if [ $node_missing -ne 0 ]; then
  print_error "建图系统启动失败"
  echo "请查看日志: $MAPPING_LOG"
  exit 1
fi

print_success "建图系统启动成功"

# ============================================================
# Step 5: Check data flow
# ============================================================
print_step "5/8" "验证数据流"

check_topic() {
  local topic="$1"
  local qos="$2"
  local qos_flag=""
  if [ -n "$qos" ]; then
    qos_flag="--qos-reliability $qos"
  fi

  if timeout 3 ros2 topic echo $qos_flag --once "$topic" >/dev/null 2>&1; then
    print_success "$topic"

    # Try to get frequency
    local hz_output
    hz_output=$(timeout 3 ros2 topic hz $qos_flag "$topic" --window 10 2>&1 || true)
    if echo "$hz_output" | grep -q "average rate"; then
      local rate=$(echo "$hz_output" | grep "average rate" | awk '{print $3}')
      echo "  频率: ~${rate} Hz"
    fi
  else
    print_warning "$topic (无数据)"
  fi
}

echo "输入话题:"
check_topic "/mid360/lidar" "best_effort"
check_topic "/odometry/filtered" "reliable"

echo ""
echo "处理后话题:"
check_topic "/mid360/points_filtered" ""
check_topic "/cloud/ground" ""
check_topic "/cloud/obstacles" ""

echo ""
echo "地图话题:"
if timeout 3 ros2 topic list 2>/dev/null | grep -q "^/map$"; then
  print_success "/map (等待移动机器人后发布)"
else
  print_warning "/map (移动机器人后会出现)"
fi

# ============================================================
# Step 6: Launch RViz (optional)
# ============================================================
print_step "6/8" "启动可视化 (RViz2)"

if [ "$LAUNCH_RVIZ" = "true" ]; then
  RVIZ_CONFIG="src/t_robot_slam/rviz/mapping.rviz"

  if [ -f "$RVIZ_CONFIG" ]; then
    echo "启动 RViz2: $RVIZ_CONFIG"
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" > "$RVIZ_LOG" 2>&1 &
    RVIZ_PID=$!
    echo "  PID: $RVIZ_PID"
    echo "  日志: $RVIZ_LOG"
    sleep 3

    if kill -0 "$RVIZ_PID" 2>/dev/null; then
      print_success "RViz2 已启动"
      echo ""
      echo "RViz 显示说明:"
      echo "  - 地面点云: 绿色"
      echo "  - 障碍物点云: 红色"
      echo "  - 2D 地图: 灰色栅格 (移动后出现)"
      echo "  - 机器人轨迹: 绿色路径"
      echo "  - SLAM 图节点: 蓝色球"
    else
      print_warning "RViz2 启动失败，继续运行建图"
      RVIZ_PID=""
    fi
  else
    print_warning "未找到 RViz 配置文件: $RVIZ_CONFIG"
  fi
else
  print_success "跳过 RViz（LAUNCH_RVIZ=false）"
  echo "手动启动: ros2 run rviz2 rviz2 -d src/t_robot_slam/rviz/mapping.rviz"
fi

# ============================================================
# Step 7: System ready
# ============================================================
print_step "7/8" "系统就绪"

print_success "所有组件已启动，准备建图！"

echo ""
echo "============================================"
echo "  建图操作指南"
echo "============================================"
echo ""
echo "1️⃣  控制机器人移动 (新终端):"
echo "    cd ~/Code/Demo5"
echo "    source install/setup.bash"
echo "    ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "    建图建议:"
echo "    - 🐢 慢速移动 (< 0.2 m/s)"
echo "    - 🔄 定期返回起点触发回环检测"
echo "    - 🏢 在特征丰富的环境建图"
echo ""
echo "2️⃣  监控建图进度 (新终端):"
echo "    # 查看 RTAB-Map 统计"
echo "    ros2 topic echo /rtabmap/info --once"
echo ""
echo "    # 实时监控 (如果有 slam_monitor.py)"
echo "    ros2 run t_robot_slam slam_monitor.py"
echo ""
echo "    # 查看日志"
echo "    tail -f $MAPPING_LOG | grep rtabmap"
echo ""
echo "3️⃣  观察地面分割效果 (RViz):"
echo "    - 地面点云应该稳定连续 (绿色)"
echo "    - 障碍物点云应该只包含墙壁/家具 (红色)"
echo "    - 2D 地图会在移动后出现并实时更新"
echo ""
echo "4️⃣  完成建图后按 Ctrl+C 停止"
echo "    地图会自动保存到: $DB_PATH"

if [ "$AUTO_EXPORT" = "true" ]; then
  echo "    地图将自动导出到: $EXPORT_DIR"
fi

echo ""
echo "============================================"
echo "  日志文件"
echo "============================================"
echo "  Bringup:    $BRINGUP_LOG"
echo "  Mapping:    $MAPPING_LOG"
if [ -n "$RVIZ_PID" ]; then
  echo "  RViz:       $RVIZ_LOG"
fi
if [ -n "$MICRO_ROS_PID" ]; then
  echo "  micro-ROS:  $MICRO_ROS_LOG"
fi
echo ""
echo "============================================"

# ============================================================
# Step 8: Wait for user to stop
# ============================================================
print_step "8/8" "建图进行中..."

echo "按 Ctrl+C 停止建图并导出地图"
echo ""

# Wait for mapping to complete
wait "$MAPPING_PID" 2>/dev/null || true

# ============================================================
# Post-mapping: Export maps (if requested)
# ============================================================
if [ "$AUTO_EXPORT" = "true" ]; then
  print_header "导出地图"

  if [ -f "$DB_PATH" ]; then
    DB_SIZE=$(du -h "$DB_PATH" | cut -f1)
    echo "数据库大小: $DB_SIZE"

    mkdir -p "$EXPORT_DIR"

    echo ""
    echo "导出 3D 点云..."
    if ./src/t_robot_slam/scripts/export_rtabmap.py \
      "$DB_PATH" \
      -o "$EXPORT_DIR/cloud.pcd" \
      --3d \
      --voxel "$EXPORT_VOXEL"; then
      print_success "3D 点云已导出"
    else
      print_warning "3D 点云导出失败"
    fi

    echo ""
    echo "导出轨迹..."
    if ./src/t_robot_slam/scripts/export_rtabmap.py \
      "$DB_PATH" \
      -o "$EXPORT_DIR/trajectory.txt" \
      --poses; then
      print_success "轨迹已导出"
    else
      print_warning "轨迹导出失败"
    fi

    echo ""
    print_success "导出完成: $EXPORT_DIR"
    echo ""
    echo "查看导出文件:"
    ls -lh "$EXPORT_DIR/"
  else
    print_warning "数据库不存在，跳过导出"
  fi
fi

# ============================================================
# Final summary
# ============================================================
print_header "建图测试完成"

if [ -f "$DB_PATH" ]; then
  DB_SIZE=$(du -h "$DB_PATH" | cut -f1)
  echo "地图数据库: $DB_PATH"
  echo "  大小: $DB_SIZE"
  echo ""
  echo "手动导出地图:"
  echo "  # 导出所有格式 (推荐)"
  echo "  ./src/t_robot_slam/scripts/export_rtabmap.py \\"
  echo "    $DB_PATH \\"
  echo "    -o ./maps/my_map \\"
  echo "    --3d --voxel 0.05"
  echo ""
  echo "  # 导出轨迹"
  echo "  ./src/t_robot_slam/scripts/export_rtabmap.py \\"
  echo "    $DB_PATH \\"
  echo "    -o ./maps/trajectory.txt \\"
  echo "    --poses"
  echo ""
  echo "  # 保存 2D 地图 (需要 RTAB-Map 运行中)"
  echo "  ros2 run nav2_map_server map_saver_cli -f ./maps/2d_map"
fi

echo ""
echo "下次启动选项:"
echo "  # 继续当前地图"
echo "  DELETE_DB=false ./scripts/test_full_mapping.sh"
echo ""
echo "  # 创建新地图"
echo "  DELETE_DB=true ./scripts/test_full_mapping.sh"
echo ""
echo "  # 不启动 RViz (节省资源)"
echo "  LAUNCH_RVIZ=false ./scripts/test_full_mapping.sh"
echo ""
echo "  # 自动导出地图"
echo "  AUTO_EXPORT=true ./scripts/test_full_mapping.sh"
echo ""

print_success "感谢使用 T-Robot SLAM 系统！"
echo "=========================================="
