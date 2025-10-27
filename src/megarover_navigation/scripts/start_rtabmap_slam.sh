#!/bin/bash

# MegaRover3 RTABMAP SLAM启动脚本
# 用于启动完整的SLAM系统

echo "=========================================="
echo "  MegaRover3 RTABMAP SLAM System"
echo "=========================================="
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置"
    echo "请先运行: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# 源工作空间
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "已加载本地工作空间"
else
    echo "警告: 本地工作空间未构建"
    echo "请先运行: colcon build"
fi

# 解析参数
MODE="mapping"
RVIZ="true"
DATABASE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --localization)
            MODE="localization"
            shift
            ;;
        --no-rviz)
            RVIZ="false"
            shift
            ;;
        --database)
            DATABASE="$2"
            shift 2
            ;;
        --help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --localization     使用定位模式（默认为建图模式）"
            echo "  --no-rviz         不启动RViz"
            echo "  --database PATH   指定数据库路径"
            echo "  --help           显示帮助信息"
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 设置参数
LAUNCH_ARGS="use_sim_time:=false rviz:=$RVIZ"

if [ "$MODE" == "localization" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS localization:=true"
    echo "模式: 定位"
else
    echo "模式: 建图"
fi

if [ ! -z "$DATABASE" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS database_path:=$DATABASE"
    echo "数据库: $DATABASE"
fi

echo ""
echo "启动参数: $LAUNCH_ARGS"
echo ""

# 检查设备
echo "检查设备..."

# 检查Livox MID360
if ls /dev/ttyUSB* 2>/dev/null | grep -q .; then
    echo "✓ 检测到USB设备（可能是Livox MID360）"
else
    echo "⚠ 未检测到USB设备，Livox MID360可能未连接"
fi

# 检查RealSense相机
if lsusb | grep -q "Intel.*RealSense"; then
    echo "✓ 检测到Intel RealSense相机"
else
    echo "⚠ 未检测到Intel RealSense相机"
fi

echo ""
echo "=========================================="
echo "启动系统..."
echo "=========================================="
echo ""

# 启动launch文件
ros2 launch megarover_rtabmap.launch.py $LAUNCH_ARGS