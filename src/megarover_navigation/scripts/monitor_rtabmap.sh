#!/bin/bash

# RTABMAP系统监控脚本
# 用于监控和调试SLAM系统运行状态

clear
echo "=========================================="
echo "    RTABMAP SLAM系统监控工具"
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
    exit 1
fi

while true; do
    clear
    echo "=========================================="
    echo "    RTABMAP SLAM系统监控"
    echo "    $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=========================================="
    echo ""

    # 检查节点
    echo -e "${BLUE}[节点状态]${NC}"
    echo "-------------------"

    # 检查关键节点
    NODES=(
        "/rtabmap"
        "/livox_ros_driver2_node"
        "/camera/realsense2_camera"
        "/pub_odom"
        "/robot_state_publisher"
        "/joint_state_publisher"
    )

    for node in "${NODES[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            echo -e "${GREEN}✓${NC} $node"
        else
            echo -e "${RED}✗${NC} $node"
        fi
    done

    echo ""

    # 检查话题
    echo -e "${BLUE}[话题频率]${NC}"
    echo "-------------------"

    # 检查关键话题的频率
    TOPICS=(
        "/livox/lidar"
        "/camera/color/image_raw"
        "/camera/depth/image_rect_raw"
        "/odom"
        "/tf"
    )

    for topic in "${TOPICS[@]}"; do
        HZ=$(timeout 1 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | tail -n1 | awk '{print $3}')
        if [ ! -z "$HZ" ]; then
            # 根据频率设置颜色
            if (( $(echo "$HZ > 5" | bc -l) )); then
                COLOR=$GREEN
            elif (( $(echo "$HZ > 1" | bc -l) )); then
                COLOR=$YELLOW
            else
                COLOR=$RED
            fi
            printf "%-30s: ${COLOR}%6.1f Hz${NC}\n" "$topic" "$HZ"
        else
            printf "%-30s: ${RED}无数据${NC}\n" "$topic"
        fi
    done

    echo ""

    # 检查TF变换
    echo -e "${BLUE}[TF变换]${NC}"
    echo "-------------------"

    # 检查关键TF
    TF_PAIRS=(
        "map:odom"
        "odom:base_footprint"
        "base_footprint:base_link"
        "base_link:mid360_frame"
        "base_link:d455_link"
    )

    for tf_pair in "${TF_PAIRS[@]}"; do
        PARENT=$(echo $tf_pair | cut -d: -f1)
        CHILD=$(echo $tf_pair | cut -d: -f2)

        TF_RESULT=$(timeout 0.5 ros2 run tf2_ros tf2_echo "$PARENT" "$CHILD" 2>/dev/null | head -n1)
        if [ ! -z "$TF_RESULT" ]; then
            echo -e "${GREEN}✓${NC} $PARENT -> $CHILD"
        else
            echo -e "${RED}✗${NC} $PARENT -> $CHILD"
        fi
    done

    echo ""

    # RTABMAP统计信息
    echo -e "${BLUE}[RTABMAP统计]${NC}"
    echo "-------------------"

    # 获取RTABMAP信息
    INFO_TOPIC="/rtabmap/info"
    if ros2 topic list 2>/dev/null | grep -q "$INFO_TOPIC"; then
        INFO=$(timeout 1 ros2 topic echo "$INFO_TOPIC" --once 2>/dev/null)

        if [ ! -z "$INFO" ]; then
            LOOP_CLOSURES=$(echo "$INFO" | grep "loop_closure_id:" | awk '{print $2}')
            KEYFRAMES=$(echo "$INFO" | grep "keyframe_id:" | awk '{print $2}')
            WM_SIZE=$(echo "$INFO" | grep "wm_size:" | awk '{print $2}')

            [ ! -z "$LOOP_CLOSURES" ] && echo "闭环检测: $LOOP_CLOSURES"
            [ ! -z "$KEYFRAMES" ] && echo "关键帧数: $KEYFRAMES"
            [ ! -z "$WM_SIZE" ] && echo "工作内存: $WM_SIZE"
        fi
    fi

    echo ""

    # 系统资源
    echo -e "${BLUE}[系统资源]${NC}"
    echo "-------------------"

    # CPU使用率
    CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
    echo "CPU使用率: $CPU_USAGE%"

    # 内存使用
    MEM_TOTAL=$(free -h | grep "^Mem:" | awk '{print $2}')
    MEM_USED=$(free -h | grep "^Mem:" | awk '{print $3}')
    echo "内存使用: $MEM_USED / $MEM_TOTAL"

    # RTABMAP进程内存
    RTABMAP_PID=$(pgrep -f "rtabmap" | head -n1)
    if [ ! -z "$RTABMAP_PID" ]; then
        RTABMAP_MEM=$(ps -o rss= -p "$RTABMAP_PID" | awk '{print $1/1024 "MB"}')
        echo "RTABMAP内存: $RTABMAP_MEM"
    fi

    echo ""
    echo "=========================================="
    echo "按 Ctrl+C 退出监控"
    echo "=========================================="

    # 刷新间隔
    sleep 2
done