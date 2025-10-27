#!/bin/bash

# 模块化RTABMAP SLAM启动脚本
# 自动检测传感器并选择最佳配置

echo "=========================================="
echo "  模块化RTABMAP SLAM系统"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS2环境未设置${NC}"
    echo "请先运行: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# 源工作空间
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo -e "${GREEN}已加载本地工作空间${NC}"
else
    echo -e "${YELLOW}警告: 本地工作空间未构建${NC}"
    echo "请先运行: colcon build"
fi

# 默认参数
MODE="auto"
RVIZ="true"
MONITORING="true"
DATABASE=""
FORCE_MODE=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)
            MODE="$2"
            shift 2
            ;;
        --no-rviz)
            RVIZ="false"
            shift
            ;;
        --no-monitoring)
            MONITORING="false"
            shift
            ;;
        --database)
            DATABASE="$2"
            shift 2
            ;;
        --force)
            FORCE_MODE="$2"
            shift 2
            ;;
        --help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --mode MODE          设置SLAM模式:"
            echo "                       auto - 自动检测（默认）"
            echo "                       fusion - MID360 + D455i融合"
            echo "                       lidar_only - 仅MID360"
            echo "                       rgbd_only - 仅D455i"
            echo "                       odometry_only - 仅里程计"
            echo "  --force MODE        强制使用特定模式（忽略传感器检测）"
            echo "  --no-rviz          不启动RViz"
            echo "  --no-monitoring    不启用健康监控"
            echo "  --database PATH    指定数据库路径"
            echo "  --help            显示帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                          # 自动模式"
            echo "  $0 --mode lidar_only       # 仅激光雷达"
            echo "  $0 --force fusion          # 强制融合模式"
            exit 0
            ;;
        *)
            echo -e "${RED}未知参数: $1${NC}"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 检测传感器
echo -e "${BLUE}[传感器检测]${NC}"
echo "-------------------"

# 检查Livox MID360
LIVOX_AVAILABLE=false
if ls /dev/ttyUSB* 2>/dev/null | grep -q .; then
    echo -e "${GREEN}✓ 检测到USB设备（可能是Livox MID360）${NC}"
    LIVOX_AVAILABLE=true

    # 检查权限
    if ! groups | grep -q dialout; then
        echo -e "${YELLOW}  ⚠ 用户不在dialout组，可能无法访问设备${NC}"
        echo "  运行: sudo usermod -a -G dialout $USER"
    fi
else
    echo -e "${YELLOW}✗ 未检测到Livox MID360${NC}"
fi

# 检查RealSense D455
REALSENSE_AVAILABLE=false
if lsusb | grep -q "Intel.*RealSense"; then
    echo -e "${GREEN}✓ 检测到Intel RealSense相机${NC}"
    REALSENSE_AVAILABLE=true

    # 检查SDK
    if ! command -v realsense-viewer &> /dev/null; then
        echo -e "${YELLOW}  ⚠ RealSense SDK未安装${NC}"
    fi
else
    echo -e "${YELLOW}✗ 未检测到RealSense相机${NC}"
fi

echo ""

# 确定运行模式
if [ ! -z "$FORCE_MODE" ]; then
    RUN_MODE=$FORCE_MODE
    echo -e "${YELLOW}强制模式: $RUN_MODE${NC}"
elif [ "$MODE" == "auto" ]; then
    if $LIVOX_AVAILABLE && $REALSENSE_AVAILABLE; then
        RUN_MODE="fusion"
        echo -e "${GREEN}自动选择: 传感器融合模式${NC}"
    elif $LIVOX_AVAILABLE; then
        RUN_MODE="lidar_only"
        echo -e "${GREEN}自动选择: 仅激光雷达模式${NC}"
    elif $REALSENSE_AVAILABLE; then
        RUN_MODE="rgbd_only"
        echo -e "${GREEN}自动选择: 仅RGB-D模式${NC}"
    else
        RUN_MODE="odometry_only"
        echo -e "${YELLOW}自动选择: 降级到纯里程计模式${NC}"
    fi
else
    RUN_MODE=$MODE
    echo -e "${BLUE}手动选择: $RUN_MODE${NC}"
fi

# 构建launch参数
LAUNCH_ARGS="use_sim_time:=false"
LAUNCH_ARGS="$LAUNCH_ARGS rviz:=$RVIZ"
LAUNCH_ARGS="$LAUNCH_ARGS enable_monitoring:=$MONITORING"

if [ ! -z "$FORCE_MODE" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS force_mode:=$FORCE_MODE"
    LAUNCH_ARGS="$LAUNCH_ARGS auto_mode:=false"
else
    LAUNCH_ARGS="$LAUNCH_ARGS force_mode:=$RUN_MODE"
    LAUNCH_ARGS="$LAUNCH_ARGS auto_mode:=true"
fi

if [ ! -z "$DATABASE" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS database_path:=$DATABASE"
fi

echo ""
echo -e "${BLUE}[系统配置]${NC}"
echo "-------------------"
echo "SLAM模式: $RUN_MODE"
echo "RViz可视化: $RVIZ"
echo "健康监控: $MONITORING"
[ ! -z "$DATABASE" ] && echo "数据库: $DATABASE"

# 检查系统资源
echo ""
echo -e "${BLUE}[系统资源]${NC}"
echo "-------------------"
CPU_CORES=$(nproc)
MEM_TOTAL=$(free -h | grep "^Mem:" | awk '{print $2}')
echo "CPU核心: $CPU_CORES"
echo "总内存: $MEM_TOTAL"

# 检查必要的ROS包
echo ""
echo -e "${BLUE}[依赖检查]${NC}"
echo "-------------------"

REQUIRED_PACKAGES=(
    "rtabmap_slam"
    "pointcloud_to_laserscan"
    "tf2_ros"
)

MISSING_PACKAGES=()
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
        echo -e "${GREEN}✓ $pkg${NC}"
    else
        echo -e "${RED}✗ $pkg${NC}"
        MISSING_PACKAGES+=($pkg)
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo ""
    echo -e "${RED}缺少必要的包，请安装:${NC}"
    for pkg in "${MISSING_PACKAGES[@]}"; do
        echo "  sudo apt install ros-$ROS_DISTRO-$pkg"
    done
    echo ""
    read -p "是否继续？(y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 创建日志目录
LOG_DIR="$HOME/.ros/rtabmap_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/rtabmap_$(date +%Y%m%d_%H%M%S).log"

echo ""
echo "=========================================="
echo -e "${BLUE}启动模块化SLAM系统...${NC}"
echo "模式: $RUN_MODE"
echo "日志: $LOG_FILE"
echo "=========================================="
echo ""

# 设置Python脚本权限
chmod +x sensor_detector.py health_monitor.py 2>/dev/null

# 启动系统
echo "启动命令:"
echo "ros2 launch modular_rtabmap.launch.py $LAUNCH_ARGS"
echo ""

# 捕获Ctrl+C信号
trap cleanup INT

cleanup() {
    echo ""
    echo -e "${YELLOW}正在关闭系统...${NC}"
    pkill -f "ros2 launch"
    pkill -f "rtabmap"
    pkill -f "sensor_detector"
    pkill -f "health_monitor"
    echo -e "${GREEN}系统已关闭${NC}"
    exit 0
}

# 启动launch文件
ros2 launch megarover_navigation modular_rtabmap.launch.py $LAUNCH_ARGS 2>&1 | tee "$LOG_FILE"