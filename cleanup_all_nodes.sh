#!/bin/bash
# 彻底清理所有ROS2和相关进程

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}  清理所有ROS2和传感器节点${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# 是否显示详细信息
VERBOSE=${1:-false}

# 清理函数
cleanup_process() {
    local process_name="$1"
    # 使用 -f 参数匹配完整命令行，支持长进程名
    local count=$(pgrep -cf "$process_name" 2>/dev/null | head -1 || echo "0")

    if [ "$count" -gt 0 ]; then
        if [ "$VERBOSE" = "true" ]; then
            echo -e "${YELLOW}正在清理: $process_name ($count 个进程)${NC}"
        fi
        killall -9 "$process_name" 2>/dev/null || true
        # 额外使用pkill -f确保清理
        pkill -9 -f "$process_name" 2>/dev/null || true
        sleep 0.2

        # 验证清理
        local remaining=$(pgrep -cf "$process_name" 2>/dev/null | head -1 || echo "0")
        if [ "$remaining" -eq 0 ]; then
            if [ "$VERBOSE" = "true" ]; then
                echo -e "${GREEN}✓ $process_name 已清理${NC}"
            fi
        else
            echo -e "${RED}✗ $process_name 清理失败，还有 $remaining 个进程${NC}"
        fi
    elif [ "$VERBOSE" = "true" ]; then
        echo "  $process_name - 无运行进程"
    fi
}

echo "步骤1: 清理SLAM相关节点..."
cleanup_process "rtabmap"
cleanup_process "rgbd_sync"
cleanup_process "icp_odometry"
cleanup_process "rtabmapviz"

echo ""
echo "步骤2: 清理Nav2导航节点..."
cleanup_process "bt_navigator"
cleanup_process "controller_server"
cleanup_process "planner_server"
cleanup_process "recoveries_server"
cleanup_process "behavior_server"
cleanup_process "waypoint_follower"
cleanup_process "velocity_smoother"
cleanup_process "lifecycle_manager"

echo ""
echo "步骤3: 清理Cartographer节点（如果有）..."
cleanup_process "cartographer_node"
cleanup_process "cartographer_occupancy_grid_node"

echo ""
echo "步骤4: 清理传感器驱动节点..."
cleanup_process "livox_ros_driver2_node"
cleanup_process "realsense2_camera_node"
cleanup_process "pointcloud_to_laserscan_node"

echo ""
echo "步骤5: 清理机器人底盘节点..."
cleanup_process "micro_ros_agent"
cleanup_process "pub_odom"
cleanup_process "robot_state_publisher"

echo ""
echo "步骤6: 清理自定义节点..."
cleanup_process "odometry_fusion"
cleanup_process "sensor_detector"
cleanup_process "health_monitor"

echo ""
echo "步骤7: 清理可视化和工具..."
cleanup_process "rviz2"
cleanup_process "rqt"
cleanup_process "static_transform_publisher"

echo ""
echo "步骤8: 清理地图服务器..."
cleanup_process "map_server"
cleanup_process "map_saver"
cleanup_process "amcl"

echo ""
echo "步骤9: 彻底清理Python节点（通过名称）..."
# 清理可能通过python3运行的节点
pkill -9 -f "odometry_fusion.py" 2>/dev/null || true
pkill -9 -f "sensor_detector.py" 2>/dev/null || true
pkill -9 -f "health_monitor.py" 2>/dev/null || true

echo ""
echo "步骤10: 等待进程完全退出..."
sleep 1

# 验证清理效果
echo ""
echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}  验证清理结果${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# 检查是否还有ROS2节点
if command -v ros2 &> /dev/null; then
    # 尝试列出节点（可能失败，因为daemon可能已停止）
    NODES=$(timeout 2 ros2 node list 2>/dev/null || echo "")

    if [ -n "$NODES" ]; then
        NODE_COUNT=$(echo "$NODES" | grep -v "^$" | wc -l)
        if [ "$NODE_COUNT" -gt 0 ]; then
            echo -e "${YELLOW}警告: 还有 $NODE_COUNT 个ROS2节点在运行:${NC}"
            echo "$NODES"
            echo ""
            echo "这可能是系统节点（如ros2 daemon）或其他应用的节点"
        else
            echo -e "${GREEN}✓ 无ROS2节点运行${NC}"
        fi
    else
        echo -e "${GREEN}✓ ROS2 daemon可能已停止或无节点运行${NC}"
    fi
fi

# 检查关键进程
echo ""
echo "检查关键进程..."
check_process() {
    local name="$1"
    # 使用 -f 参数匹配完整命令行
    local count=$(pgrep -cf "$name" 2>/dev/null | head -1 || echo "0")
    if [ "$count" -gt 0 ]; then
        echo -e "${YELLOW}⚠ $name: $count 个进程仍在运行${NC}"
        if [ "$VERBOSE" = "true" ]; then
            pgrep -af "$name"
        fi
        return 1
    else
        echo -e "${GREEN}✓ $name: 已清理${NC}"
        return 0
    fi
}

ALL_CLEAN=true

check_process "rtabmap" || ALL_CLEAN=false
check_process "livox" || ALL_CLEAN=false
check_process "realsense" || ALL_CLEAN=false
check_process "cartographer" || ALL_CLEAN=false
check_process "bt_navigator" || ALL_CLEAN=false

echo ""
echo -e "${BLUE}======================================${NC}"
if [ "$ALL_CLEAN" = true ]; then
    echo -e "${GREEN}✓ 所有节点已成功清理！${NC}"
    echo -e "${GREEN}可以安全开始新的测试${NC}"
else
    echo -e "${YELLOW}⚠ 部分进程可能未完全清理${NC}"
    echo ""
    echo "如果遇到问题，可以："
    echo "  1. 重启终端"
    echo "  2. 运行: pkill -9 -f ros"
    echo "  3. 重启系统（最后手段）"
fi
echo -e "${BLUE}======================================${NC}"
echo ""

# 可选：深度清理
if [ "$2" = "--deep" ]; then
    echo ""
    echo "执行深度清理..."

    # 清理ROS2共享内存
    echo "  - 清理共享内存..."
    rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
    rm -rf /tmp/rmw_* 2>/dev/null || true

    # 重启ROS2 daemon以清除缓存
    echo "  - 重启ROS2 daemon..."
    ros2 daemon stop 2>/dev/null || true
    sleep 1
    ros2 daemon start 2>/dev/null || true

    echo -e "${GREEN}✓ 深度清理完成${NC}"
fi

exit 0
