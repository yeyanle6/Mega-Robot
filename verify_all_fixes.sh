#!/bin/bash
# 验证所有修复的测试脚本
# 用法: ./verify_all_fixes.sh

echo "=================================================="
echo "  RTABMAP MID360 修复验证脚本"
echo "  版本: v1.0"
echo "  日期: 2025-10-22"
echo "=================================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 测试计数
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# 测试函数
run_test() {
    local test_name="$1"
    local test_cmd="$2"
    local expected_pattern="$3"

    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    echo -e "${BLUE}[测试 $TOTAL_TESTS]${NC} $test_name"

    # 运行测试命令
    result=$(eval "$test_cmd" 2>&1)

    # 检查结果
    if echo "$result" | grep -q "$expected_pattern"; then
        echo -e "${GREEN}✓ 通过${NC}"
        PASSED_TESTS=$((PASSED_TESTS + 1))
        return 0
    else
        echo -e "${RED}✗ 失败${NC}"
        echo "预期包含: $expected_pattern"
        echo "实际结果: $result"
        FAILED_TESTS=$((FAILED_TESTS + 1))
        return 1
    fi
    echo ""
}

# 检查函数
check_exists() {
    local item="$1"
    local type="$2"  # file, node, topic

    case "$type" in
        file)
            if [ -f "$item" ]; then
                echo -e "${GREEN}✓${NC} 文件存在: $item"
                return 0
            else
                echo -e "${RED}✗${NC} 文件不存在: $item"
                return 1
            fi
            ;;
        node)
            if ros2 node list 2>/dev/null | grep -q "$item"; then
                echo -e "${GREEN}✓${NC} 节点运行: $item"
                return 0
            else
                echo -e "${YELLOW}⚠${NC} 节点未运行: $item"
                return 1
            fi
            ;;
        topic)
            if ros2 topic list 2>/dev/null | grep -q "$item"; then
                echo -e "${GREEN}✓${NC} 话题存在: $item"
                return 0
            else
                echo -e "${YELLOW}⚠${NC} 话题不存在: $item"
                return 1
            fi
            ;;
    esac
}

echo -e "${YELLOW}阶段 1: 检查修复文件${NC}"
echo "------------------------------------"
check_exists "/home/wang/Code/Demo6/TF_EMPTY_FRAME_ID_FIX.md" "file"
check_exists "/home/wang/Code/Demo6/MID360_CONFIG_FIX_SUMMARY.md" "file"
check_exists "/home/wang/Code/Demo6/COMPLETE_FIX_SUMMARY.md" "file"
echo ""

echo -e "${YELLOW}阶段 2: 检查源代码修复${NC}"
echo "------------------------------------"

# 检查pub_odom.cpp修复
if grep -q "t.header.frame_id = \"odom\"" /home/wang/Code/Demo6/src/megarover3_ros2/megarover3_bringup/src/pub_odom.cpp; then
    echo -e "${GREEN}✓${NC} pub_odom.cpp TF初始化已添加"
else
    echo -e "${RED}✗${NC} pub_odom.cpp TF初始化缺失"
fi

# 检查modular_rtabmap.launch.py修复
if grep -q "mid360_imu_broadcaster" /home/wang/Code/Demo6/src/megarover_navigation/launch/modular_rtabmap.launch.py; then
    echo -e "${GREEN}✓${NC} mid360_imu静态TF广播器已添加"
else
    echo -e "${RED}✗${NC} mid360_imu静态TF广播器缺失"
fi

# 检查YAML配置清理
if ! grep -q "scan_cloud_topic:" /home/wang/Code/Demo6/src/megarover_navigation/config/rtabmap_lidar_only.yaml; then
    echo -e "${GREEN}✓${NC} rtabmap_lidar_only.yaml已清理冗余参数"
else
    echo -e "${YELLOW}⚠${NC} rtabmap_lidar_only.yaml仍包含冗余参数"
fi

echo ""

echo -e "${YELLOW}阶段 3: 检查编译状态${NC}"
echo "------------------------------------"
if [ -f "/home/wang/Code/Demo6/install/megarover_navigation/share/megarover_navigation/launch/modular_rtabmap.launch.py" ]; then
    echo -e "${GREEN}✓${NC} megarover_navigation已编译"
else
    echo -e "${RED}✗${NC} megarover_navigation未编译"
fi

if [ -f "/home/wang/Code/Demo6/install/megarover3_bringup/lib/megarover3_bringup/pub_odom" ]; then
    echo -e "${GREEN}✓${NC} megarover3_bringup已编译"
else
    echo -e "${RED}✗${NC} megarover3_bringup未编译"
fi
echo ""

echo -e "${YELLOW}阶段 4: 运行时检查（需要系统运行）${NC}"
echo "------------------------------------"
echo "检查ROS2系统状态..."

# 检查ROS2守护进程
if pgrep -x "ros2" > /dev/null; then
    echo -e "${GREEN}✓${NC} ROS2守护进程运行中"
else
    echo -e "${YELLOW}⚠${NC} ROS2守护进程未运行"
fi

# 检查节点
echo ""
echo "检查关键节点状态:"
check_exists "/pub_odom" "node"
check_exists "/mid360_imu_broadcaster" "node"
check_exists "/robot_state_publisher_node" "node"
check_exists "/rtabmap" "node"

# 检查话题
echo ""
echo "检查关键话题:"
check_exists "/livox/lidar" "topic"
check_exists "/livox/imu" "topic"
check_exists "/odom" "topic"
check_exists "/tf" "topic"
check_exists "/tf_static" "topic"

echo ""

# TF树检查
if ros2 node list 2>/dev/null | grep -q "/robot_state_publisher"; then
    echo -e "${YELLOW}阶段 5: TF树验证${NC}"
    echo "------------------------------------"
    echo "尝试生成TF树（5秒超时）..."

    # 临时生成TF树
    timeout 5 ros2 run tf2_tools view_frames 2>&1 > /tmp/tf_check.log

    if [ -f "frames.pdf" ]; then
        echo -e "${GREEN}✓${NC} TF树生成成功: frames.pdf"

        # 检查TF错误
        error_count=$(grep -c "Error:" /tmp/tf_check.log 2>/dev/null || echo "0")
        if [ "$error_count" -eq "0" ]; then
            echo -e "${GREEN}✓${NC} 无TF错误！"
        else
            echo -e "${YELLOW}⚠${NC} 发现 $error_count 个TF错误"
            echo "查看详情: cat /tmp/tf_check.log"
        fi
    else
        echo -e "${YELLOW}⚠${NC} TF树生成失败（可能需要更长时间）"
    fi
else
    echo -e "${YELLOW}阶段 5: TF树验证 [跳过 - 系统未运行]${NC}"
fi

echo ""
echo "=================================================="
echo -e "${BLUE}测试总结${NC}"
echo "=================================================="
echo "总测试数: $TOTAL_TESTS"
echo -e "通过: ${GREEN}$PASSED_TESTS${NC}"
echo -e "失败: ${RED}$FAILED_TESTS${NC}"
echo ""

# 给出建议
if ros2 node list 2>/dev/null | wc -l | grep -q "^[01]$"; then
    echo -e "${YELLOW}💡 提示:${NC}"
    echo "系统当前未运行。要进行完整验证，请运行:"
    echo ""
    echo "  ros2 launch megarover_navigation modular_rtabmap.launch.py \\"
    echo "      force_mode:=lidar_only \\"
    echo "      rviz:=true"
    echo ""
    echo "然后在新终端重新运行此脚本。"
fi

echo ""
echo "查看详细修复文档:"
echo "  - TF_EMPTY_FRAME_ID_FIX.md"
echo "  - MID360_CONFIG_FIX_SUMMARY.md"
echo "  - COMPLETE_FIX_SUMMARY.md"
echo ""
echo "=================================================="
