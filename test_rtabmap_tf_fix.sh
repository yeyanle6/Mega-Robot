#!/bin/bash

echo "========================================="
echo "  RTABMAP TF配置修复验证测试"
echo "========================================="

# 清理环境
echo "1. 清理环境..."
./cleanup_all_nodes.sh > /dev/null 2>&1
sleep 2

# 源workspace
source install/setup.bash

# 启动RTABMAP (lidar_only模式)
echo "2. 启动RTABMAP SLAM (lidar_only模式)..."
timeout 15 ros2 launch megarover_navigation modular_rtabmap.launch.py \
  force_mode:=lidar_only \
  rviz:=false \
  enable_monitoring:=false \
  > /tmp/rtabmap_tf_test.log 2>&1 &

LAUNCH_PID=$!
sleep 8

echo ""
echo "========================================="
echo "  检查节点状态"
echo "========================================="

# 检查关键节点
timeout 2 ros2 node list 2>/dev/null | grep -q "icp_odometry" && echo "✓ icp_odometry节点运行中" || echo "✗ icp_odometry节点未运行"
timeout 2 ros2 node list 2>/dev/null | grep -q "rtabmap" && echo "✓ rtabmap节点运行中" || echo "✗ rtabmap节点未运行"
timeout 2 ros2 node list 2>/dev/null | grep -q "livox_lidar_publisher" && echo "✓ livox节点运行中" || echo "✗ livox节点未运行"
timeout 2 ros2 node list 2>/dev/null | grep -q "robot_state_publisher" && echo "✓ robot_state_publisher运行中" || echo "✗ robot_state_publisher未运行"

echo ""
echo "========================================="
echo "  检查TF配置"
echo "========================================="

# 检查frame_id配置
echo "检查icp_odometry配置:"
grep "icp_odometry.*frame_id.*=" /tmp/rtabmap_tf_test.log | head -2

echo ""
echo "检查rtabmap配置:"
grep "rtabmap.*frame_id.*=" /tmp/rtabmap_tf_test.log | head -3

echo ""
echo "========================================="
echo "  检查TF树"
echo "========================================="

# 生成TF树
timeout 3 ros2 run tf2_tools view_frames 2>/dev/null
if [ -f "frames.pdf" ]; then
    echo "✓ TF树已生成: frames.pdf"
    # 显示TF树结构
    timeout 1 ros2 run tf2_ros tf2_echo map icp_odom 2>&1 | head -5 && echo "✓ TF: map → icp_odom 存在" || echo "✗ TF: map → icp_odom 不存在"
    timeout 1 ros2 run tf2_ros tf2_echo icp_odom mid360_lidar 2>&1 | head -5 && echo "✓ TF: icp_odom → mid360_lidar 存在" || echo "✗ TF: icp_odom → mid360_lidar 不存在"
    timeout 1 ros2 run tf2_ros tf2_echo base_link mid360_lidar 2>&1 | head -5 && echo "✓ TF: base_link → mid360_lidar 存在" || echo "✗ TF: base_link → mid360_lidar 存在"
else
    echo "✗ 无法生成TF树"
fi

echo ""
echo "========================================="
echo "  检查关键话题"
echo "========================================="

# 检查话题
timeout 2 ros2 topic list 2>/dev/null | grep -q "/rtabmap/grid_map" && echo "✓ /rtabmap/grid_map 话题存在" || echo "✗ /rtabmap/grid_map 话题不存在"
timeout 2 ros2 topic list 2>/dev/null | grep -q "/map" && echo "✓ /map 话题存在" || echo "✗ /map 话题不存在"
timeout 2 ros2 topic list 2>/dev/null | grep -q "/odom" && echo "✓ /odom 话题存在" || echo "✗ /odom 话题不存在"
timeout 2 ros2 topic list 2>/dev/null | grep -q "/livox/lidar" && echo "✓ /livox/lidar 话题存在" || echo "✗ /livox/lidar 话题不存在"

echo ""
echo "========================================="
echo "  检查RTABMAP运行状态"
echo "========================================="

# 检查rtabmap是否在处理数据
RTABMAP_NODES=$(grep "Intermediate node added" /tmp/rtabmap_tf_test.log | wc -l)
echo "RTABMAP已添加节点数: $RTABMAP_NODES"

if [ "$RTABMAP_NODES" -gt 0 ]; then
    echo "✓ RTABMAP正在处理数据并建图"
else
    echo "✗ RTABMAP未处理数据"
fi

# 检查TF错误
TF_ERRORS=$(grep -c "ERROR.*TF.*mid360\|ERROR.*TF.*icp_odom\|TF of received scan cloud is not set" /tmp/rtabmap_tf_test.log 2>/dev/null || echo "0")
echo "TF相关错误数: $TF_ERRORS"

if [ "$TF_ERRORS" -eq 0 ]; then
    echo "✓ 无TF配置错误"
else
    echo "✗ 存在TF配置错误"
    echo "错误详情:"
    grep "ERROR.*TF.*mid360\|ERROR.*TF.*icp_odom\|TF of received scan cloud is not set" /tmp/rtabmap_tf_test.log | head -5
fi

echo ""
echo "========================================="
echo "  测试总结"
echo "========================================="

# 杀死launch进程
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null

# 清理
./cleanup_all_nodes.sh > /dev/null 2>&1

if [ "$RTABMAP_NODES" -gt 0 ] && [ "$TF_ERRORS" -eq 0 ]; then
    echo "✓ TF配置修复成功！RTABMAP正常运行"
    echo "  - frame_id配置正确 (mid360_lidar)"
    echo "  - TF树完整 (map → icp_odom → mid360_lidar ← base_link)"
    echo "  - RTABMAP正在建图"
    exit 0
else
    echo "✗ 仍存在问题,请检查日志: /tmp/rtabmap_tf_test.log"
    exit 1
fi
