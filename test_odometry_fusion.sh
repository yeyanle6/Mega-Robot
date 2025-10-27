#!/bin/bash
# 测试里程计融合节点

echo "========================================="
echo "里程计融合节点测试"
echo "========================================="

# 颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 只在单独运行时清理（不在test_basic_functions.sh中清理）
if [ -z "$SKIP_CLEANUP" ]; then
    echo "清理残留节点..."
    if [ -f "./cleanup_all_nodes.sh" ]; then
        ./cleanup_all_nodes.sh > /dev/null 2>&1
    else
        killall -9 livox_ros_driver2_node micro_ros_agent pub_odom odometry_fusion \
            cartographer_node pointcloud_to_laserscan_node rtabmap realsense2_camera_node 2>/dev/null || true
        sleep 1
    fi
fi

# Source环境
source install/setup.bash

echo ""
echo "步骤1: 启动里程计融合节点..."
ros2 launch megarover_navigation odometry_fusion.launch.py > /tmp/odom_fusion_test.log 2>&1 &
FUSION_PID=$!
echo "  PID: $FUSION_PID"
sleep 3

# 检查节点是否运行
TEST_FAIL=0
if ! ps -p $FUSION_PID > /dev/null; then
    echo -e "${RED}✗ 里程计融合节点启动失败${NC}"
    cat /tmp/odom_fusion_test.log
    TEST_FAIL=1
else
    echo -e "${GREEN}✓ 里程计融合节点已启动${NC}"
fi

echo ""
echo "步骤2: 检查节点..."
if ros2 node list | grep -q "odometry_fusion"; then
    echo -e "${GREEN}✓ 节点已注册${NC}"
else
    echo -e "${RED}✗ 节点未注册${NC}"
    TEST_FAIL=1
fi

echo ""
echo "步骤3: 检查发布的话题..."
echo "等待话题发布..."
sleep 2

if ros2 topic list | grep -q "/odom_fused"; then
    echo -e "${GREEN}✓ /odom_fused 话题存在${NC}"
else
    echo -e "${YELLOW}⚠ /odom_fused 话题不存在（可能因为没有输入数据）${NC}"
fi

echo ""
echo "步骤4: 检查话题信息..."
ros2 topic info /odom_fused -v 2>/dev/null || echo -e "${YELLOW}⚠ 话题信息不可用${NC}"

echo ""
echo "步骤5: 查看节点日志..."
echo "----------------------------------------"
tail -20 /tmp/odom_fusion_test.log
echo "----------------------------------------"

echo ""
echo "步骤6: 清理..."
kill $FUSION_PID 2>/dev/null || true
sleep 1

echo ""
echo "========================================="
if [ $TEST_FAIL -eq 0 ]; then
    echo -e "${GREEN}测试完成！所有检查通过${NC}"
else
    echo -e "${YELLOW}测试完成！部分检查未通过${NC}"
fi
echo "========================================="
echo ""
echo "注意："
echo "  - 里程计融合需要轮式里程计(/rover_odo)和IMU(/livox/imu)数据"
echo "  - 在完整系统中测试时，这些数据会由相应传感器提供"
echo "  - 节点日志保存在: /tmp/odom_fusion_test.log"
echo ""

exit $TEST_FAIL
