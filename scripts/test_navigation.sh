#!/bin/bash
# 阶段4：导航功能测试

# 颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}"
echo "========================================"
echo "  阶段4: 导航功能测试"
echo "========================================"
echo -e "${NC}"

# 检查前置条件
echo ""
echo "--- 检查前置条件 ---"
echo ""

echo -e "${YELLOW}导航测试前提条件:${NC}"
echo "  1. 已完成SLAM建图（阶段2）"
echo "  2. 已保存地图文件"
echo "  3. 机器人在已知地图中"
echo ""

read -p "是否满足所有前提条件? (y/N): " prereq_ok
if [[ ! $prereq_ok =~ ^[Yy]$ ]]; then
    echo -e "${RED}请先完成SLAM建图！${NC}"
    echo "运行: ./test_slam_mapping.sh"
    exit 1
fi

# Source环境
source install/setup.bash

# 选择导航模式
echo ""
echo "导航模式选择："
echo "  1) SLAM+导航 - 边建图边导航（推荐用于测试）"
echo "  2) 纯导航   - 使用已有地图（需要地图文件）"
echo ""
read -p "选择模式 [1-2, 默认1]: " nav_mode

case $nav_mode in
    2)
        NAV_MODE="pure_nav"
        # 要求提供地图文件
        echo ""
        read -p "地图文件路径 (yaml): " map_file
        if [ ! -f "$map_file" ]; then
            echo -e "${RED}地图文件不存在: $map_file${NC}"
            exit 1
        fi
        echo -e "${GREEN}✓ 将使用地图: $map_file${NC}"
        ;;
    *)
        NAV_MODE="slam_nav"
        echo -e "${GREEN}✓ 将使用SLAM+导航模式${NC}"
        ;;
esac

# 询问是否启动RViz
read -p "是否启动RViz? (Y/n): " use_rviz
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
        planner_server recoveries_server cartographer_node odometry_fusion 2>/dev/null || true
    sleep 2
fi

# 启动系统
echo ""
echo -e "${BLUE}"
echo "========================================"
echo "  启动导航系统"
echo "========================================"
echo -e "${NC}"

if [ "$NAV_MODE" = "slam_nav" ]; then
    # SLAM+导航模式
    echo "启动SLAM+导航系统..."
    ./start_navigation.sh &
    SYS_PID=$!
else
    # 纯导航模式
    echo "启动纯导航系统..."
    echo -e "${YELLOW}注意: 纯导航模式需要手动配置，此脚本提供基础框架${NC}"
    # TODO: 实现纯导航模式启动
    echo -e "${RED}纯导航模式尚未完全实现，请使用SLAM+导航模式${NC}"
    exit 1
fi

# 等待系统启动
echo "等待系统初始化..."
sleep 20

# 检查系统状态
echo ""
echo "--- 系统状态检查 ---"
echo ""

# 检查节点
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

echo "核心节点:"
check_node "rtabmap"
check_node "odometry_fusion"

echo ""
echo "导航节点 (可能需要更多时间启动):"
# Nav2节点可能还未完全启动
ros2 node list 2>/dev/null | grep -E "(bt_navigator|controller|planner)" || \
    echo -e "${YELLOW}⚠ Nav2节点可能还在启动中...${NC}"

# 检查话题
echo ""
echo "--- 关键话题检查 ---"
echo ""

check_topic() {
    local topic_name="$1"
    if ros2 topic list 2>/dev/null | grep -q "$topic_name"; then
        echo -e "${GREEN}✓${NC} 话题: $topic_name"
        return 0
    else
        echo -e "${YELLOW}⚠${NC} 话题: $topic_name 未找到"
        return 1
    fi
}

check_topic "/odom"
check_topic "/scan"
check_topic "/cmd_vel"
check_topic "/rtabmap/grid_map" || check_topic "/map"

# 导航测试菜单
echo ""
echo -e "${BLUE}"
echo "========================================"
echo "  导航测试选项"
echo "========================================"
echo -e "${NC}"
echo ""
echo "系统已启动，选择测试项目："
echo ""
echo "  1) 基础测试 - 检查导航栈是否正常"
echo "  2) 简单目标 - 发送一个简单的导航目标"
echo "  3) 手动测试 - 在RViz中手动设置目标"
echo "  4) 退出测试"
echo ""

while true; do
    read -p "选择测试 [1-4]: " test_choice

    case $test_choice in
        1)
            echo ""
            echo "=== 基础测试 ==="
            echo ""

            # 检查/cmd_vel
            echo "检查速度命令话题..."
            if timeout 5 ros2 topic echo /cmd_vel --once > /dev/null 2>&1; then
                echo -e "${GREEN}✓ /cmd_vel 话题正常${NC}"
            else
                echo -e "${YELLOW}⚠ /cmd_vel 无数据（正常，等待导航命令）${NC}"
            fi

            # 检查地图
            echo "检查地图话题..."
            if timeout 3 ros2 topic echo /rtabmap/grid_map --once > /dev/null 2>&1; then
                echo -e "${GREEN}✓ 地图话题正常${NC}"
            else
                echo -e "${RED}✗ 地图话题无数据${NC}"
            fi

            # 检查TF
            echo "检查TF树..."
            if timeout 3 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1; then
                echo -e "${GREEN}✓ TF变换正常 (map → base_link)${NC}"
            else
                echo -e "${RED}✗ TF变换失败${NC}"
            fi

            echo ""
            ;;

        2)
            echo ""
            echo "=== 简单目标测试 ==="
            echo ""
            echo "将发送一个简单的导航目标：向前1米"
            echo ""

            read -p "是否继续? (y/N): " send_goal
            if [[ $send_goal =~ ^[Yy]$ ]]; then
                echo "发送导航目标..."

                # 发送简单目标
                timeout 30 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
                    "{pose: {header: {frame_id: 'map'},
                     pose: {position: {x: 1.0, y: 0.0, z: 0.0},
                     orientation: {w: 1.0}}}}" &

                GOAL_PID=$!

                # 监控导航
                echo "监控导航状态..."
                echo "（观察机器人是否移动）"
                echo ""

                # 监控/cmd_vel
                timeout 30 bash -c '
                    while true; do
                        vel=$(timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null | grep "linear:" -A 1 | tail -1 | awk "{print \$2}")
                        if [ -n "$vel" ]; then
                            echo "速度命令: linear.x = $vel"
                        fi
                        sleep 1
                    done
                ' &
                MON_PID=$!

                # 等待完成
                wait $GOAL_PID 2>/dev/null
                kill $MON_PID 2>/dev/null

                echo ""
                echo -e "${GREEN}✓ 导航目标已发送${NC}"
                echo "观察机器人是否移动到目标位置"
            fi

            echo ""
            ;;

        3)
            echo ""
            echo "=== 手动测试模式 ==="
            echo ""

            if [[ $use_rviz =~ ^[Nn]$ ]]; then
                echo -e "${YELLOW}未启动RViz！${NC}"
                read -p "是否现在启动RViz? (y/N): " start_rviz
                if [[ $start_rviz =~ ^[Yy]$ ]]; then
                    rviz2 &
                    sleep 3
                fi
            fi

            echo "在RViz中进行手动测试："
            echo ""
            echo "1. 确认RViz已打开"
            echo "2. 点击工具栏 'Nav2 Goal' 按钮"
            echo "3. 在地图上点击设置目标位置"
            echo "4. 观察机器人移动"
            echo ""
            echo "按回车返回菜单..."
            read
            ;;

        4)
            echo ""
            echo "退出测试..."
            break
            ;;

        *)
            echo -e "${RED}无效选择${NC}"
            ;;
    esac
done

# 清理
echo ""
echo "--- 清理 ---"
echo ""

read -p "是否停止系统? (Y/n): " stop_sys
if [[ ! $stop_sys =~ ^[Nn]$ ]]; then
    echo "停止所有节点..."
    killall -9 rtabmap livox_ros_driver2_node realsense2_camera_node \
        pointcloud_to_laserscan_node rgbd_sync micro_ros_agent pub_odom \
        robot_state_publisher rviz2 bt_navigator controller_server \
        planner_server recoveries_server odometry_fusion 2>/dev/null || true

    echo -e "${GREEN}✓ 系统已停止${NC}"
fi

echo ""
echo -e "${BLUE}"
echo "========================================"
echo "  导航测试完成"
echo "========================================"
echo -e "${NC}"
echo ""
echo "测试总结："
echo "  - 如果机器人能够移动到目标点，说明导航功能正常"
echo "  - 如果遇到问题，查看日志和TESTING_GUIDE.md的故障排查部分"
echo ""
