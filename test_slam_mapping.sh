#!/bin/bash
# 阶段2：SLAM建图测试（半自动）

# 颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}"
echo "========================================"
echo "  阶段2: SLAM建图测试"
echo "========================================"
echo -e "${NC}"

echo ""
echo "本测试需要手动控制机器人移动进行建图"
echo ""

# 检查前置条件
echo "--- 检查前置条件 ---"
echo ""

# 检查是否完成基础测试
read -p "是否已完成阶段1基础功能测试? (y/N): " basic_done
if [[ ! $basic_done =~ ^[Yy]$ ]]; then
    echo -e "${RED}请先完成阶段1测试！${NC}"
    echo "运行: ./test_basic_functions.sh"
    exit 1
fi

# 检查环境
source install/setup.bash

# 选择SLAM模式
echo ""
echo "可用的SLAM模式："
echo "  1) fusion       - 融合模式 (MID360 + D455, 最高精度)"
echo "  2) lidar_only   - 激光雷达模式 (仅MID360)"
echo "  3) rgbd_only    - RGB-D模式 (仅D455)"
echo "  4) auto         - 自动检测（推荐）"
echo ""
read -p "选择SLAM模式 [1-4, 默认4]: " mode_choice

case $mode_choice in
    1) SLAM_MODE="fusion" ;;
    2) SLAM_MODE="lidar_only" ;;
    3) SLAM_MODE="rgbd_only" ;;
    *) SLAM_MODE="auto" ;;
esac

echo -e "${GREEN}✓ 将使用模式: $SLAM_MODE${NC}"

# 询问是否启动RViz
read -p "是否启动RViz可视化? (Y/n): " use_rviz
if [[ $use_rviz =~ ^[Nn]$ ]]; then
    RVIZ_ARG="rviz:=false"
else
    RVIZ_ARG="rviz:=true"
fi

# 清理旧进程
echo ""
echo "--- 清理残留节点 ---"
if [ -f "./cleanup_all_nodes.sh" ]; then
    echo "运行统一清理脚本..."
    ./cleanup_all_nodes.sh
    echo -e "${GREEN}✓ 清理完成${NC}"
else
    echo -e "${YELLOW}⚠ 使用手动清理...${NC}"
    killall -9 rtabmap livox_ros_driver2_node realsense2_camera_node \
        pointcloud_to_laserscan_node rgbd_sync micro_ros_agent pub_odom \
        robot_state_publisher rviz2 bt_navigator controller_server \
        planner_server cartographer_node odometry_fusion 2>/dev/null || true
    sleep 2
fi

# 清理旧数据库（可选）
echo ""
read -p "是否删除旧的RTABMAP数据库? (y/N): " clean_db
if [[ $clean_db =~ ^[Yy]$ ]]; then
    rm -f ~/.ros/rtabmap.db
    echo -e "${GREEN}✓ 已删除旧数据库${NC}"
fi

# 启动SLAM系统
echo ""
echo -e "${BLUE}"
echo "========================================"
echo "  启动SLAM系统"
echo "========================================"
echo -e "${NC}"

if [ "$SLAM_MODE" = "auto" ]; then
    # 使用交互式启动脚本
    echo "使用交互式启动脚本..."
    ./start_navigation.sh &
    SLAM_PID=$!
else
    # 直接启动
    echo "启动SLAM系统，模式: $SLAM_MODE"
    ros2 launch megarover_navigation modular_rtabmap.launch.py \
        force_mode:=$SLAM_MODE \
        $RVIZ_ARG > /tmp/slam_test.log 2>&1 &
    SLAM_PID=$!
fi

# 等待启动
echo "等待系统初始化..."
sleep 15

# 检查系统状态
echo ""
echo "--- 系统状态检查 ---"
echo ""

# 检查关键节点
check_node() {
    local node_name="$1"
    if ros2 node list 2>/dev/null | grep -q "$node_name"; then
        echo -e "${GREEN}✓${NC} 节点: $node_name"
        return 0
    else
        echo -e "${RED}✗${NC} 节点: $node_name 未找到"
        return 1
    fi
}

check_node "rtabmap"
check_node "livox" || check_node "camera"
check_node "odometry_fusion"

# 检查关键话题
echo ""
echo "--- 话题检查 ---"
echo ""

check_topic() {
    local topic_name="$1"
    if ros2 topic list 2>/dev/null | grep -q "$topic_name"; then
        echo -e "${GREEN}✓${NC} 话题: $topic_name"
        # 检查频率
        hz=$(timeout 3 ros2 topic hz "$topic_name" 2>/dev/null | grep "average rate" | awk '{print $3}')
        if [ -n "$hz" ]; then
            echo "    频率: $hz Hz"
        fi
        return 0
    else
        echo -e "${RED}✗${NC} 话题: $topic_name 未找到"
        return 1
    fi
}

# 检查地图话题 (RTABMAP发布到/map而不是/rtabmap/grid_map)
check_topic "/map"
check_topic "/odom"
check_topic "/scan" || check_topic "/camera/depth/image_raw"

# 等待用户准备
echo ""
echo -e "${BLUE}"
echo "========================================"
echo "  准备建图"
echo "========================================"
echo -e "${NC}"
echo ""
echo "系统已启动，现在可以开始建图了！"
echo ""
echo -e "${YELLOW}建图建议:${NC}"
echo "  1. 缓慢移动机器人 (速度 < 0.3 m/s)"
echo "  2. 避免急转弯"
echo "  3. 定期回到已知区域（帮助闭环检测）"
echo "  4. 建议建图时间: 5-10分钟"
echo ""

if [[ $use_rviz =~ ^[Nn]$ ]]; then
    echo -e "${YELLOW}注意: 未启动RViz，建议在另一终端运行:${NC}"
    echo "  rviz2"
fi

echo ""
echo "控制机器人移动进行建图..."
echo "（使用键盘、手柄或自定义控制程序）"
echo ""

# 监控循环
echo "按 Ctrl+C 停止建图并保存地图"
echo ""

# 捕获Ctrl+C信号
trap 'echo ""; echo "正在停止..."; kill $SLAM_PID 2>/dev/null; sleep 2; save_map; exit' INT

# 实时监控
monitor_slam() {
    local count=0
    while true; do
        sleep 5
        ((count++))

        echo -e "${BLUE}--- 状态监控 (${count}x5秒) ---${NC}"

        # 检查RTABMAP信息
        if ros2 topic list 2>/dev/null | grep -q "/rtabmap/info"; then
            # 获取地图点数
            local info=$(timeout 2 ros2 topic echo /rtabmap/info --once 2>/dev/null)
            if [ -n "$info" ]; then
                echo -e "${GREEN}✓ RTABMAP运行正常${NC}"
                # 可以解析info获取更多信息
            fi
        fi

        # 检查里程计
        if timeout 2 ros2 topic hz /odom 2>/dev/null | grep -q "average"; then
            echo -e "${GREEN}✓ 里程计正常${NC}"
        fi

        echo ""
    done
}

# 保存地图函数
save_map() {
    echo ""
    echo -e "${BLUE}"
    echo "========================================"
    echo "  保存地图"
    echo "========================================"
    echo -e "${NC}"

    read -p "是否保存地图? (Y/n): " save
    if [[ ! $save =~ ^[Nn]$ ]]; then
        # 默认地图名称
        MAP_NAME="test_map_$(date +%Y%m%d_%H%M%S)"

        read -p "地图名称 [默认: $MAP_NAME]: " user_map_name
        if [ -n "$user_map_name" ]; then
            MAP_NAME="$user_map_name"
        fi

        echo "保存地图为: $MAP_NAME"

        # 保存为ROS2格式
        echo "导出栅格地图..."
        timeout 10 ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" 2>/dev/null || \
            echo -e "${YELLOW}⚠ 栅格地图导出失败（可能需要手动导出）${NC}"

        # RTABMAP数据库位置
        if [ -f ~/.ros/rtabmap.db ]; then
            echo -e "${GREEN}✓ RTABMAP数据库已保存: ~/.ros/rtabmap.db${NC}"
            echo "  可使用 rtabmap-databaseViewer ~/.ros/rtabmap.db 查看"
        fi

        echo ""
        echo -e "${GREEN}✓ 地图保存完成${NC}"
        echo "  地图文件: $MAP_NAME.yaml 和 $MAP_NAME.pgm"
    fi

    # 清理
    echo ""
    echo "清理进程..."
    killall -9 rtabmap livox_ros_driver2_node realsense2_camera_node \
        pointcloud_to_laserscan_node rgbd_sync micro_ros_agent pub_odom \
        robot_state_publisher rviz2 2>/dev/null || true
}

# 运行监控
monitor_slam
