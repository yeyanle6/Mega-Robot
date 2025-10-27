#!/bin/bash

# MegaRover Navigation 依赖安装脚本
# 自动安装所有必需的ROS2包和Python依赖

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔════════════════════════════════════════╗"
echo "║   MegaRover Navigation 依赖安装器      ║"
echo "╚════════════════════════════════════════╝"
echo -e "${NC}"

# 检查ROS2版本
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: 未检测到ROS2环境${NC}"
    echo "请先安装ROS2并source环境:"
    echo "  source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo -e "${GREEN}检测到ROS2版本: $ROS_DISTRO${NC}"
echo ""

# 检查是否为root
if [ "$EUID" -eq 0 ]; then
   echo -e "${RED}请不要使用sudo运行此脚本${NC}"
   exit 1
fi

# 更新包列表
echo -e "${BLUE}[1/6] 更新软件包列表...${NC}"
sudo apt update

# 安装Nav2导航栈
echo ""
echo -e "${BLUE}[2/6] 安装Nav2导航栈...${NC}"
PACKAGES_NAV2=(
    "ros-$ROS_DISTRO-navigation2"
    "ros-$ROS_DISTRO-nav2-bringup"
    "ros-$ROS_DISTRO-nav2-common"
    "ros-$ROS_DISTRO-nav2-msgs"
    "ros-$ROS_DISTRO-nav2-bt-navigator"
    "ros-$ROS_DISTRO-nav2-controller"
    "ros-$ROS_DISTRO-nav2-planner"
    "ros-$ROS_DISTRO-nav2-behaviors"
    "ros-$ROS_DISTRO-nav2-waypoint-follower"
    "ros-$ROS_DISTRO-nav2-lifecycle-manager"
    "ros-$ROS_DISTRO-nav2-costmap-2d"
    "ros-$ROS_DISTRO-nav2-amcl"
    "ros-$ROS_DISTRO-nav2-map-server"
    "ros-$ROS_DISTRO-nav2-velocity-smoother"
    "ros-$ROS_DISTRO-nav2-rviz-plugins"
    "ros-$ROS_DISTRO-nav2-navfn-planner"
    "ros-$ROS_DISTRO-nav2-smoother"
)

for pkg in "${PACKAGES_NAV2[@]}"; do
    echo -n "  安装 $pkg... "
    if dpkg -l | grep -q "^ii  $pkg"; then
        echo -e "${GREEN}已安装${NC}"
    else
        if sudo apt install -y "$pkg" >/dev/null 2>&1; then
            echo -e "${GREEN}成功${NC}"
        else
            echo -e "${YELLOW}失败（可选）${NC}"
        fi
    fi
done

# 安装RTABMAP SLAM
echo ""
echo -e "${BLUE}[3/6] 安装RTABMAP SLAM...${NC}"
PACKAGES_RTABMAP=(
    "ros-$ROS_DISTRO-rtabmap"
    "ros-$ROS_DISTRO-rtabmap-ros"
    "ros-$ROS_DISTRO-rtabmap-slam"
    "ros-$ROS_DISTRO-rtabmap-sync"
    "ros-$ROS_DISTRO-rtabmap-msgs"
    "ros-$ROS_DISTRO-rtabmap-launch"
    "ros-$ROS_DISTRO-rtabmap-examples"
)

for pkg in "${PACKAGES_RTABMAP[@]}"; do
    echo -n "  安装 $pkg... "
    if dpkg -l | grep -q "^ii  $pkg"; then
        echo -e "${GREEN}已安装${NC}"
    else
        if sudo apt install -y "$pkg" >/dev/null 2>&1; then
            echo -e "${GREEN}成功${NC}"
        else
            echo -e "${YELLOW}失败（可选）${NC}"
        fi
    fi
done

# 安装传感器相关包
echo ""
echo -e "${BLUE}[4/6] 安装传感器相关包...${NC}"
PACKAGES_SENSORS=(
    "ros-$ROS_DISTRO-pointcloud-to-laserscan"
    "ros-$ROS_DISTRO-robot-state-publisher"
    "ros-$ROS_DISTRO-joint-state-publisher"
    "ros-$ROS_DISTRO-tf2-ros"
    "ros-$ROS_DISTRO-tf2-tools"
    "ros-$ROS_DISTRO-tf2-geometry-msgs"
    "ros-$ROS_DISTRO-sensor-msgs"
    "ros-$ROS_DISTRO-geometry-msgs"
    "ros-$ROS_DISTRO-visualization-msgs"
    "ros-$ROS_DISTRO-diagnostic-msgs"
)

for pkg in "${PACKAGES_SENSORS[@]}"; do
    echo -n "  安装 $pkg... "
    if dpkg -l | grep -q "^ii  $pkg"; then
        echo -e "${GREEN}已安装${NC}"
    else
        if sudo apt install -y "$pkg" >/dev/null 2>&1; then
            echo -e "${GREEN}成功${NC}"
        else
            echo -e "${YELLOW}失败（可选）${NC}"
        fi
    fi
done

# 安装Python依赖
echo ""
echo -e "${BLUE}[5/6] 安装Python依赖...${NC}"
PYTHON_PACKAGES=(
    "python3-yaml"
    "python3-psutil"
    "python3-pip"
)

for pkg in "${PYTHON_PACKAGES[@]}"; do
    echo -n "  安装 $pkg... "
    if dpkg -l | grep -q "^ii  $pkg"; then
        echo -e "${GREEN}已安装${NC}"
    else
        if sudo apt install -y "$pkg" >/dev/null 2>&1; then
            echo -e "${GREEN}成功${NC}"
        else
            echo -e "${YELLOW}失败${NC}"
        fi
    fi
done

# 安装Python pip包
echo ""
echo "安装Python pip包..."
pip3 install --user termcolor >/dev/null 2>&1 && echo -e "  ${GREEN}✓ termcolor${NC}" || echo -e "  ${YELLOW}⚠ termcolor${NC}"

# 系统配置
echo ""
echo -e "${BLUE}[6/6] 系统配置...${NC}"

# 检查用户组
echo -n "检查dialout组权限... "
if groups | grep -q dialout; then
    echo -e "${GREEN}已在dialout组${NC}"
else
    echo -e "${YELLOW}不在dialout组${NC}"
    echo "添加用户到dialout组..."
    sudo usermod -a -G dialout $USER
    echo -e "${GREEN}已添加（需要重新登录生效）${NC}"
fi

# 检查可选的硬件驱动
echo ""
echo -e "${BLUE}检查可选硬件驱动...${NC}"

# RealSense SDK
echo -n "Intel RealSense SDK... "
if dpkg -l | grep -q "librealsense2"; then
    echo -e "${GREEN}已安装${NC}"
else
    echo -e "${YELLOW}未安装${NC}"
    echo "  如需使用RealSense相机，请运行:"
    echo "  sudo apt install librealsense2-dkms librealsense2-utils"
fi

# Livox SDK
echo -n "Livox SDK... "
if [ -d "/usr/local/include/livox_sdk" ]; then
    echo -e "${GREEN}已安装${NC}"
else
    echo -e "${YELLOW}未安装${NC}"
    echo "  如需使用Livox激光雷达，请访问:"
    echo "  https://github.com/Livox-SDK/Livox-SDK"
fi

# 创建必要的目录
echo ""
echo -e "${BLUE}创建必要的目录...${NC}"
mkdir -p ~/.ros
echo -e "${GREEN}✓ ~/.ros${NC}"

# 总结
echo ""
echo -e "${CYAN}════════════════════════════════════════${NC}"
echo -e "${GREEN}依赖安装完成！${NC}"
echo ""

# 检查是否需要重新登录
NEED_RELOGIN=false
if ! groups | grep -q dialout; then
    NEED_RELOGIN=true
    echo -e "${YELLOW}注意: 您需要重新登录以使dialout组权限生效${NC}"
fi

echo "下一步:"
echo ""
echo "1. 构建工作空间:"
echo "   colcon build --packages-select megarover_navigation"
echo "   source install/setup.bash"
echo ""
echo "2. 运行构建测试:"
echo "   ./test_build.sh"
echo ""
echo "3. 验证系统:"
echo "   python3 src/megarover_navigation/scripts/validate_system.py"
echo ""
echo "4. 启动导航系统:"
echo "   ./launch_navigation.sh"

if [ "$NEED_RELOGIN" = true ]; then
    echo ""
    echo -e "${YELLOW}重要: 请先注销并重新登录以使权限生效！${NC}"
fi

echo -e "${CYAN}════════════════════════════════════════${NC}"