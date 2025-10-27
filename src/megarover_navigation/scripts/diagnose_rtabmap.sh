#!/bin/bash

# RTABMAP系统诊断脚本
# 用于诊断和排查SLAM系统问题

echo "=========================================="
echo "    RTABMAP SLAM系统诊断工具"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 诊断结果
ISSUES=()
WARNINGS=()
SUCCESS=()

# 函数：添加问题
add_issue() {
    ISSUES+=("$1")
    echo -e "${RED}✗ $1${NC}"
}

# 函数：添加警告
add_warning() {
    WARNINGS+=("$1")
    echo -e "${YELLOW}⚠ $1${NC}"
}

# 函数：添加成功
add_success() {
    SUCCESS+=("$1")
    echo -e "${GREEN}✓ $1${NC}"
}

echo -e "${BLUE}[1. ROS2环境检查]${NC}"
echo "-------------------"
if [ -z "$ROS_DISTRO" ]; then
    add_issue "ROS2环境未设置"
    echo "  解决方案: source /opt/ros/<distro>/setup.bash"
else
    add_success "ROS2环境已设置: $ROS_DISTRO"
fi

# 检查工作空间
if [ -f "install/setup.bash" ]; then
    add_success "本地工作空间已构建"
else
    add_warning "本地工作空间未构建"
    echo "  解决方案: colcon build"
fi
echo ""

echo -e "${BLUE}[2. 硬件设备检查]${NC}"
echo "-------------------"

# 检查Livox设备
if ls /dev/ttyUSB* 2>/dev/null | grep -q .; then
    add_success "检测到USB串口设备"

    # 检查用户权限
    if groups | grep -q dialout; then
        add_success "用户在dialout组中"
    else
        add_warning "用户不在dialout组中"
        echo "  解决方案: sudo usermod -a -G dialout $USER"
        echo "  然后重新登录"
    fi
else
    add_warning "未检测到USB串口设备（Livox MID360）"
fi

# 检查RealSense相机
if lsusb | grep -q "Intel.*RealSense"; then
    add_success "检测到Intel RealSense相机"

    # 检查realsense-viewer
    if command -v realsense-viewer &> /dev/null; then
        add_success "RealSense SDK已安装"
    else
        add_warning "RealSense SDK未安装"
        echo "  解决方案: 安装librealsense2-dkms和librealsense2-utils"
    fi
else
    add_warning "未检测到Intel RealSense相机"
fi
echo ""

echo -e "${BLUE}[3. ROS2包依赖检查]${NC}"
echo "-------------------"

# 检查必需的包
REQUIRED_PACKAGES=(
    "rtabmap_ros"
    "livox_ros_driver2"
    "realsense2_camera"
    "megarover3_bringup"
    "pointcloud_to_laserscan"
    "tf2_ros"
    "robot_state_publisher"
)

for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
        add_success "包已安装: $pkg"
    else
        add_issue "包未找到: $pkg"
        echo "  解决方案: 检查包是否正确构建和源"
    fi
done
echo ""

echo -e "${BLUE}[4. 运行时节点检查]${NC}"
echo "-------------------"

# 检查节点是否运行
CRITICAL_NODES=(
    "/rtabmap"
    "/livox_ros_driver2_node"
    "/camera/realsense2_camera"
    "/pub_odom"
    "/robot_state_publisher"
)

NODES_RUNNING=$(ros2 node list 2>/dev/null)
if [ -z "$NODES_RUNNING" ]; then
    add_warning "没有ROS2节点在运行"
else
    for node in "${CRITICAL_NODES[@]}"; do
        if echo "$NODES_RUNNING" | grep -q "$node"; then
            add_success "节点运行中: $node"
        else
            add_warning "节点未运行: $node"
        fi
    done
fi
echo ""

echo -e "${BLUE}[5. 话题连接检查]${NC}"
echo "-------------------"

# 检查关键话题
CRITICAL_TOPICS=(
    "/livox/lidar:传感器点云"
    "/camera/color/image_raw:RGB图像"
    "/camera/depth/image_rect_raw:深度图像"
    "/odom:里程计"
    "/tf:坐标变换"
)

TOPICS_LIST=$(ros2 topic list 2>/dev/null)
if [ ! -z "$TOPICS_LIST" ]; then
    for topic_desc in "${CRITICAL_TOPICS[@]}"; do
        TOPIC=$(echo "$topic_desc" | cut -d: -f1)
        DESC=$(echo "$topic_desc" | cut -d: -f2)

        if echo "$TOPICS_LIST" | grep -q "^$TOPIC$"; then
            # 检查是否有订阅者
            SUBS=$(ros2 topic info "$TOPIC" 2>/dev/null | grep "Subscription count:" | awk '{print $3}')
            if [ "$SUBS" -gt 0 ] 2>/dev/null; then
                add_success "$DESC ($TOPIC) - $SUBS个订阅者"
            else
                add_warning "$DESC ($TOPIC) - 无订阅者"
            fi
        else
            add_warning "$DESC ($TOPIC) - 话题不存在"
        fi
    done
fi
echo ""

echo -e "${BLUE}[6. TF树完整性检查]${NC}"
echo "-------------------"

# 检查TF链
if [ ! -z "$TOPICS_LIST" ] && echo "$TOPICS_LIST" | grep -q "/tf"; then
    # 生成TF树
    timeout 3 ros2 run tf2_tools view_frames --no-exe 2>/dev/null
    if [ -f "frames.pdf" ]; then
        add_success "TF树已生成 (frames.pdf)"

        # 检查关键TF链
        TF_LINKS=(
            "map->odom"
            "odom->base_footprint"
            "base_footprint->base_link"
            "base_link->mid360_frame"
            "base_link->d455_link"
        )

        for link in "${TF_LINKS[@]}"; do
            PARENT=$(echo $link | cut -d'-' -f1)
            CHILD=$(echo $link | cut -d'>' -f2)

            if timeout 1 ros2 run tf2_ros tf2_echo "$PARENT" "$CHILD" 2>/dev/null | head -n1 | grep -q "Translation"; then
                add_success "TF链存在: $link"
            else
                add_warning "TF链缺失: $link"
            fi
        done
    else
        add_warning "无法生成TF树"
    fi
fi
echo ""

echo -e "${BLUE}[7. 参数配置检查]${NC}"
echo "-------------------"

# 检查RTABMAP参数
if [ ! -z "$NODES_RUNNING" ] && echo "$NODES_RUNNING" | grep -q "/rtabmap"; then
    # 获取一些关键参数
    KEY_PARAMS=(
        "frame_id"
        "subscribe_depth"
        "subscribe_rgb"
        "subscribe_scan_cloud"
    )

    for param in "${KEY_PARAMS[@]}"; do
        VALUE=$(ros2 param get /rtabmap "$param" 2>/dev/null | awk '{print $NF}')
        if [ ! -z "$VALUE" ]; then
            echo "  $param: $VALUE"
        fi
    done
else
    add_warning "RTABMAP节点未运行，无法检查参数"
fi
echo ""

echo -e "${BLUE}[8. 系统性能检查]${NC}"
echo "-------------------"

# CPU核心数
CPU_CORES=$(nproc)
echo "CPU核心数: $CPU_CORES"

# 内存大小
MEM_TOTAL=$(free -h | grep "^Mem:" | awk '{print $2}')
echo "总内存: $MEM_TOTAL"

# 检查交换空间
SWAP_TOTAL=$(free -h | grep "^Swap:" | awk '{print $2}')
if [ "$SWAP_TOTAL" != "0B" ]; then
    add_success "交换空间可用: $SWAP_TOTAL"
else
    add_warning "无交换空间"
    echo "  建议: 为大型地图创建交换文件"
fi

# 磁盘空间
DISK_AVAILABLE=$(df -h . | tail -n1 | awk '{print $4}')
echo "可用磁盘空间: $DISK_AVAILABLE"
echo ""

echo "=========================================="
echo -e "${BLUE}诊断摘要${NC}"
echo "=========================================="

if [ ${#ISSUES[@]} -gt 0 ]; then
    echo -e "${RED}发现 ${#ISSUES[@]} 个问题:${NC}"
    for issue in "${ISSUES[@]}"; do
        echo "  - $issue"
    done
    echo ""
fi

if [ ${#WARNINGS[@]} -gt 0 ]; then
    echo -e "${YELLOW}发现 ${#WARNINGS[@]} 个警告:${NC}"
    for warning in "${WARNINGS[@]}"; do
        echo "  - $warning"
    done
    echo ""
fi

if [ ${#SUCCESS[@]} -gt 0 ]; then
    echo -e "${GREEN}通过 ${#SUCCESS[@]} 项检查${NC}"
fi

echo ""
echo "=========================================="

# 提供建议
if [ ${#ISSUES[@]} -gt 0 ] || [ ${#WARNINGS[@]} -gt 0 ]; then
    echo -e "${BLUE}建议操作:${NC}"
    echo "1. 解决所有标记为红色的问题"
    echo "2. 检查黄色警告项是否影响系统运行"
    echo "3. 确保所有硬件设备正确连接"
    echo "4. 运行 'colcon build' 重新构建工作空间"
    echo "5. 使用 './monitor_rtabmap.sh' 监控运行时状态"
else
    echo -e "${GREEN}系统状态良好！${NC}"
    echo "可以运行 './start_rtabmap_slam.sh' 启动SLAM系统"
fi

echo "==========================================="