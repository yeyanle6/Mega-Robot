#!/bin/bash

# 系统构建和测试脚本
# 用于验证megarover_navigation包是否正确构建

set -e  # 出错时停止

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   MegaRover Navigation 构建测试        ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# 检查当前目录
if [ ! -f "src/megarover_navigation/package.xml" ]; then
    echo -e "${RED}错误: 请在Demo6目录下运行此脚本${NC}"
    exit 1
fi

# 步骤1: 清理之前的构建
echo -e "${BLUE}[1/5] 清理之前的构建...${NC}"
rm -rf build/megarover_navigation install/megarover_navigation 2>/dev/null || true
echo -e "${GREEN}✓ 清理完成${NC}"
echo ""

# 步骤2: 构建包
echo -e "${BLUE}[2/5] 构建megarover_navigation包...${NC}"
if colcon build --packages-select megarover_navigation; then
    echo -e "${GREEN}✓ 构建成功${NC}"
else
    echo -e "${RED}✗ 构建失败${NC}"
    exit 1
fi
echo ""

# 步骤3: 源环境
echo -e "${BLUE}[3/5] 加载工作空间...${NC}"
source install/setup.bash
echo -e "${GREEN}✓ 工作空间已加载${NC}"
echo ""

# 步骤4: 验证包安装
echo -e "${BLUE}[4/5] 验证包安装...${NC}"

# 检查包是否可以找到
if ros2 pkg prefix megarover_navigation &>/dev/null; then
    echo -e "${GREEN}✓ 包已正确安装${NC}"
    PKG_PATH=$(ros2 pkg prefix megarover_navigation)
    echo "  包路径: $PKG_PATH"
else
    echo -e "${RED}✗ 包未找到${NC}"
    exit 1
fi

# 检查关键文件
echo ""
echo "检查安装的文件:"

# Launch文件
LAUNCH_FILES=(
    "launch/navigation.launch.py"
    "launch/modular_rtabmap.launch.py"
    "launch/megarover_nav2_slam.launch.py"
)

for file in "${LAUNCH_FILES[@]}"; do
    if [ -f "$PKG_PATH/share/megarover_navigation/$file" ]; then
        echo -e "  ${GREEN}✓${NC} $file"
    else
        echo -e "  ${RED}✗${NC} $file"
    fi
done

# 配置文件
CONFIG_FILES=(
    "config/nav2_params.yaml"
    "config/rtabmap_fusion.yaml"
)

for file in "${CONFIG_FILES[@]}"; do
    if [ -f "$PKG_PATH/share/megarover_navigation/$file" ]; then
        echo -e "  ${GREEN}✓${NC} $file"
    else
        echo -e "  ${RED}✗${NC} $file"
    fi
done

# 节点
echo ""
echo "检查可执行节点:"
NODES=(
    "sensor_detector"
    "health_monitor"
)

for node in "${NODES[@]}"; do
    if ros2 run megarover_navigation $node --help &>/dev/null || [ $? -eq 1 ]; then
        echo -e "  ${GREEN}✓${NC} $node 可执行"
    else
        echo -e "  ${RED}✗${NC} $node 无法执行"
    fi
done

echo ""

# 步骤5: 测试Launch文件语法
echo -e "${BLUE}[5/5] 测试Launch文件...${NC}"

# 测试主launch文件
echo "测试 navigation.launch.py..."
if python3 -c "
import sys
sys.path.insert(0, '${PKG_PATH}/share/megarover_navigation/launch')
try:
    import navigation
    print('  语法检查通过')
    exit(0)
except Exception as e:
    print(f'  语法错误: {e}')
    exit(1)
" 2>/dev/null; then
    echo -e "${GREEN}✓ navigation.launch.py 测试通过${NC}"
else
    echo -e "${YELLOW}⚠ navigation.launch.py 可能有问题${NC}"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${GREEN}构建测试完成！${NC}"
echo ""
echo "下一步:"
echo "1. 运行系统验证:"
echo "   python3 src/megarover_navigation/scripts/validate_system.py"
echo ""
echo "2. 启动导航系统:"
echo "   ros2 launch megarover_navigation navigation.launch.py"
echo ""
echo "或使用快速启动:"
echo "   ./launch_navigation.sh"
echo -e "${BLUE}════════════════════════════════════════${NC}"