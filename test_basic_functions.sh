#!/bin/bash
# 阶段1：基础功能自动化测试

# 颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}"
echo "========================================"
echo "  阶段1: 基础功能测试"
echo "========================================"
echo -e "${NC}"

# 首先清理所有残留节点
echo ""
echo "--- 0. 清理残留节点 ---"
echo ""
if [ -f "./cleanup_all_nodes.sh" ]; then
    echo "运行清理脚本..."
    ./cleanup_all_nodes.sh
    echo -e "${GREEN}✓ 清理完成${NC}"
else
    echo -e "${YELLOW}⚠ 清理脚本不存在，手动清理...${NC}"
    killall -9 rtabmap livox_ros_driver2_node realsense2_camera_node \
        pointcloud_to_laserscan_node rgbd_sync micro_ros_agent pub_odom \
        robot_state_publisher rviz2 bt_navigator controller_server \
        planner_server cartographer_node odometry_fusion 2>/dev/null || true
    sleep 2
fi

# 测试结果统计
PASS=0
FAIL=0

# 测试函数
test_item() {
    local name="$1"
    local command="$2"

    echo -n "测试: $name ... "

    if eval "$command" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ PASS${NC}"
        ((PASS++))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC}"
        ((FAIL++))
        return 1
    fi
}

echo ""
echo "--- 1. 硬件检测 ---"
echo ""

# Livox MID360检测
test_item "Livox MID360硬件" "ls /dev/ttyUSB* 2>/dev/null"

# RealSense D455检测
test_item "RealSense D455硬件" "lsusb | grep -i 'Intel' | grep -i 'RealSense'"

# 用户权限检测
test_item "dialout组权限" "groups | grep dialout"

echo ""
echo "--- 2. ROS2环境检测 ---"
echo ""

# Source环境
source install/setup.bash 2>/dev/null || true

# 包是否存在
test_item "megarover_navigation包" "ros2 pkg list | grep megarover_navigation"

echo ""
echo "--- 3. 自定义消息检测 ---"
echo ""

# 消息定义
test_item "SensorStatus消息" "ros2 interface show megarover_navigation/msg/SensorStatus"
test_item "HealthStatus消息" "ros2 interface show megarover_navigation/msg/HealthStatus"
test_item "SwitchMode服务" "ros2 interface show megarover_navigation/srv/SwitchMode"

echo ""
echo "--- 4. Livox MID360功能测试 ---"
echo ""

# 启动Livox
echo "启动Livox驱动..."
ros2 launch megarover_navigation livox_mid360.launch.py > /tmp/livox_basic_test.log 2>&1 &
LIVOX_PID=$!
echo "等待Livox初始化（10秒）..."
sleep 10

# 检查节点
if ps -p $LIVOX_PID > /dev/null; then
    test_item "Livox节点启动" "ros2 node list | grep livox"
    test_item "点云话题发布" "timeout 5 ros2 topic echo /livox/lidar --once"
    test_item "IMU话题发布" "timeout 5 ros2 topic echo /livox/imu --once"
    test_item "2D扫描话题发布" "timeout 5 ros2 topic echo /scan --once"
else
    echo -e "${RED}✗ Livox启动失败${NC}"
    ((FAIL+=4))
fi

# 清理
kill $LIVOX_PID 2>/dev/null || true
sleep 2
killall -9 livox_ros_driver2_node pointcloud_to_laserscan_node 2>/dev/null || true

echo ""
echo "--- 5. RealSense D455功能测试 ---"
echo ""

# 启动RealSense
echo "启动RealSense驱动..."
ros2 launch megarover_navigation realsense_d455.launch.py > /tmp/realsense_basic_test.log 2>&1 &
RS_PID=$!
echo "等待RealSense初始化（15秒）..."
sleep 15

# 检查节点
if ps -p $RS_PID > /dev/null; then
    test_item "RealSense节点启动" "ros2 node list | grep camera"
    test_item "RGB图像话题发布" "timeout 5 ros2 topic list | grep '/camera.*color/image_raw'"
    test_item "深度图像话题发布" "timeout 5 ros2 topic list | grep '/camera.*depth.*image_raw'"
else
    echo -e "${YELLOW}⚠ RealSense启动失败（可跳过）${NC}"
    ((FAIL+=3))
fi

# 清理
kill $RS_PID 2>/dev/null || true
sleep 2
killall -9 realsense2_camera_node 2>/dev/null || true

echo ""
echo "--- 6. 里程计融合测试 ---"
echo ""

# 运行里程计融合测试
if [ -f "./test_odometry_fusion.sh" ]; then
    echo "运行里程计融合测试脚本..."
    # 设置SKIP_CLEANUP避免重复清理
    if SKIP_CLEANUP=1 ./test_odometry_fusion.sh > /tmp/odom_fusion_basic_test.log 2>&1; then
        echo -e "${GREEN}✓ 里程计融合测试通过${NC}"
        ((PASS++))
    else
        echo -e "${RED}✗ 里程计融合测试失败${NC}"
        echo "查看日志: /tmp/odom_fusion_basic_test.log"
        ((FAIL++))
    fi
else
    echo -e "${YELLOW}⚠ 里程计融合测试脚本不存在${NC}"
fi

# 最终清理
killall -9 livox_ros_driver2_node pointcloud_to_laserscan_node realsense2_camera_node odometry_fusion 2>/dev/null || true

echo ""
echo -e "${BLUE}"
echo "========================================"
echo "  测试完成"
echo "========================================"
echo -e "${NC}"
echo ""
echo -e "通过: ${GREEN}$PASS${NC}"
echo -e "失败: ${RED}$FAIL${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✓ 所有测试通过！可以进行阶段2测试${NC}"
    exit 0
else
    echo -e "${YELLOW}⚠ 有 $FAIL 项测试失败，请检查后再继续${NC}"
    echo ""
    echo "故障排查："
    echo "  - 硬件问题: 检查传感器连接和权限"
    echo "  - 驱动问题: 查看日志 /tmp/livox_basic_test.log 和 /tmp/realsense_basic_test.log"
    echo "  - 环境问题: 确保已执行 source install/setup.bash"
    exit 1
fi
