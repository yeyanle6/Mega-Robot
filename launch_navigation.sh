#!/bin/bash

# MegaRover3 Navigation System Launcher
# 简化的启动脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}"
echo "╔════════════════════════════════════════╗"
echo "║   MegaRover3 Navigation System         ║"
echo "║   模块化SLAM + Nav2导航                ║"
echo "╚════════════════════════════════════════╝"
echo -e "${NC}"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS2环境未设置${NC}"
    echo "请运行: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# 源工作空间
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo -e "${GREEN}✓ 工作空间已加载${NC}"
else
    echo -e "${YELLOW}⚠ 工作空间未构建，正在构建...${NC}"
    cd "$SCRIPT_DIR"
    colcon build --packages-select megarover_navigation
    source install/setup.bash
fi

# 解析参数
MODE="slam_nav"
SENSOR="auto"
MAP=""

print_usage() {
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "快速命令:"
    echo "  $0              # SLAM + 导航（默认）"
    echo "  $0 slam         # 仅SLAM建图"
    echo "  $0 nav <地图>   # 使用已有地图导航"
    echo ""
    echo "选项:"
    echo "  --help          显示帮助"
    echo "  --sensor MODE   传感器模式 (auto/fusion/lidar/rgbd)"
    echo "  --no-rviz       不启动RViz"
    echo ""
}

# 简化的命令解析
case "$1" in
    slam)
        MODE="slam_only"
        echo -e "${BLUE}模式: 仅SLAM建图${NC}"
        ;;
    nav)
        MODE="nav_only"
        if [ -z "$2" ]; then
            echo -e "${RED}错误: 导航模式需要地图文件${NC}"
            echo "用法: $0 nav /path/to/map.yaml"
            exit 1
        fi
        MAP="$2"
        echo -e "${BLUE}模式: 导航（使用地图: $MAP）${NC}"
        ;;
    --help|-h)
        print_usage
        exit 0
        ;;
    "")
        echo -e "${BLUE}模式: SLAM + 导航（默认）${NC}"
        ;;
    *)
        echo -e "${RED}未知参数: $1${NC}"
        print_usage
        exit 1
        ;;
esac

# 检测传感器
echo ""
echo -e "${BLUE}[传感器检测]${NC}"

LIVOX_FOUND=false
REALSENSE_FOUND=false

if ls /dev/ttyUSB* 2>/dev/null | grep -q .; then
    echo -e "${GREEN}✓ Livox MID360${NC}"
    LIVOX_FOUND=true
else
    echo -e "${YELLOW}✗ Livox MID360${NC}"
fi

if lsusb 2>/dev/null | grep -q "Intel.*RealSense"; then
    echo -e "${GREEN}✓ Intel RealSense D455i${NC}"
    REALSENSE_FOUND=true
else
    echo -e "${YELLOW}✗ Intel RealSense D455i${NC}"
fi

# 自动选择传感器模式
if [ "$SENSOR" == "auto" ]; then
    if $LIVOX_FOUND && $REALSENSE_FOUND; then
        SENSOR="fusion"
        echo -e "${GREEN}→ 使用融合模式${NC}"
    elif $LIVOX_FOUND; then
        SENSOR="lidar_only"
        echo -e "${GREEN}→ 使用激光雷达模式${NC}"
    elif $REALSENSE_FOUND; then
        SENSOR="rgbd_only"
        echo -e "${GREEN}→ 使用RGB-D模式${NC}"
    else
        echo -e "${YELLOW}⚠ 未检测到传感器，使用纯里程计模式${NC}"
        SENSOR="auto"
    fi
fi

# 构建launch命令
LAUNCH_CMD="ros2 launch megarover_navigation navigation.launch.py"
LAUNCH_CMD="$LAUNCH_CMD mode:=$MODE"
LAUNCH_CMD="$LAUNCH_CMD sensor_mode:=$SENSOR"

if [ ! -z "$MAP" ]; then
    LAUNCH_CMD="$LAUNCH_CMD map:=$MAP"
fi

# 处理其他参数
shift
for arg in "$@"; do
    case $arg in
        --no-rviz)
            LAUNCH_CMD="$LAUNCH_CMD rviz:=false"
            ;;
        --sensor)
            shift
            ;;
    esac
done

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}启动导航系统...${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "命令: $LAUNCH_CMD"
echo ""

# 执行launch命令
$LAUNCH_CMD