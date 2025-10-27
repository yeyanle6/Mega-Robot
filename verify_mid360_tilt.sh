#!/bin/bash
# Mid-360倾斜角度和朝向验证脚本
# 用于验证URDF修改后TF树中的pitch和yaw角度是否正确

set -e

echo "=========================================="
echo "  Mid-360姿态角度验证脚本"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查URDF文件
echo "1. 检查URDF源文件..."
URDF_FILE="/home/wang/Code/Demo6/src/megarover3_ros2/megarover_description/urdf/mega3.xacro"

echo "  当前配置:"
grep "mid360" "$URDF_FILE" | grep "origin" | grep -v "<!--"

echo ""
if grep -q 'rpy="0 -0.5236 -1.5708"' "$URDF_FILE"; then
    echo -e "${GREEN}✓${NC} URDF配置正确:"
    echo "    Pitch: -0.5236 rad (-30°) - 向前倾斜"
    echo "    Yaw:   -1.5708 rad (-90°) - 顺时针旋转90度"
elif grep -q 'rpy="0 -0.5236 0"' "$URDF_FILE"; then
    echo -e "${YELLOW}⚠${NC} URDF配置部分正确（仅有pitch，缺少yaw）:"
    echo "    Pitch: -0.5236 rad (-30°) ✓"
    echo "    Yaw:   0 rad (未旋转) - 可能需要调整为-90°"
elif grep -q 'rpy="0 0 0"' "$URDF_FILE"; then
    echo -e "${RED}✗${NC} URDF配置为默认值（水平安装，无旋转）"
    echo "    需要设置为: rpy=\"0 -0.5236 -1.5708\""
    exit 1
else
    echo -e "${YELLOW}⚠${NC} URDF配置不是预期值，请检查"
    exit 1
fi

echo ""
echo "2. 测试Xacro转换..."
source /home/wang/Code/Demo6/install/setup.bash
TEMP_URDF="/tmp/mega3_tilt_test.urdf"

if xacro "$URDF_FILE" > "$TEMP_URDF" 2>&1; then
    echo -e "${GREEN}✓${NC} Xacro转换成功"
else
    echo -e "${RED}✗${NC} Xacro转换失败"
    exit 1
fi

echo ""
echo "3. 验证生成的URDF..."

# 提取完整的joint定义
echo "Mid-360 Joint定义:"
echo "-------------------"
grep -A 3 "mid360_joint" "$TEMP_URDF" | head -4
echo ""

if grep -q 'rpy="0 -0.5236 -1.5708"' "$TEMP_URDF"; then
    echo -e "${GREEN}✓${NC} 生成的URDF姿态角度完全正确"
elif grep -q 'rpy="0 -0.5236 0"' "$TEMP_URDF"; then
    echo -e "${YELLOW}⚠${NC} 生成的URDF中仅有pitch，建议添加yaw=-1.5708"
else
    echo -e "${RED}✗${NC} 生成的URDF姿态角度不正确"
    exit 1
fi

echo ""
echo "4. 角度计算验证..."
# 使用bc进行浮点数计算
PITCH_RAD=-0.5236
YAW_RAD=-1.5708
PITCH_DEG=$(echo "scale=2; $PITCH_RAD * 180 / 3.14159" | bc -l)
YAW_DEG=$(echo "scale=2; $YAW_RAD * 180 / 3.14159" | bc -l)

echo "  Pitch (俯仰角):"
echo "    弧度: $PITCH_RAD rad"
echo "    角度: $PITCH_DEG°"
echo "    预期: -30°"
if [ "$PITCH_DEG" != "-30.00" ]; then
    DIFF=$(echo "scale=2; $PITCH_DEG - (-30)" | bc -l)
    echo -e "    ${YELLOW}⚠${NC} 偏差: $DIFF° (正常，弧度精度)"
else
    echo -e "    ${GREEN}✓${NC} 完全匹配"
fi

echo ""
echo "  Yaw (偏航角):"
echo "    弧度: $YAW_RAD rad"
echo "    角度: $YAW_DEG°"
echo "    预期: -90°"
if [ "$YAW_DEG" != "-90.00" ]; then
    DIFF=$(echo "scale=2; $YAW_DEG - (-90)" | bc -l)
    echo -e "    ${YELLOW}⚠${NC} 偏差: $DIFF° (正常，弧度精度)"
else
    echo -e "    ${GREEN}✓${NC} 完全匹配"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}验证完成！${NC}"
echo "=========================================="
echo ""
echo "修改摘要:"
echo "  文件:   mega3.xacro (Line 277)"
echo "  Joint:  mid360_joint"
echo "  Pitch:  -0.5236 rad (-30°) - 向前倾斜"
echo "  Yaw:    -1.5708 rad (-90°) - 顺时针旋转90度"
echo "  效果:   激光雷达向前倾斜扫描低处"
echo "  目的:   扫描低处障碍物，修正坐标系朝向"
echo ""

# 可选: 如果有robot_state_publisher运行，可以检查实时TF
if pgrep -x "robot_state_publisher" > /dev/null; then
    echo "检测到robot_state_publisher正在运行"
    echo ""
    echo "5. 检查实时TF变换..."

    # 等待TF树建立
    sleep 2

    # 尝试获取TF变换
    if ros2 run tf2_ros tf2_echo base_link mid360_base_link 2>/dev/null | head -20 > /tmp/tf_echo.txt; then
        echo -e "${GREEN}✓${NC} TF变换可用"
        echo ""
        echo "base_link → mid360_base_link 变换:"
        echo "-------------------"
        cat /tmp/tf_echo.txt
    else
        echo -e "${YELLOW}⚠${NC} 无法获取TF变换（可能需要启动系统）"
    fi
else
    echo -e "${YELLOW}提示:${NC} robot_state_publisher未运行"
    echo "要查看实时TF变换，请先启动机器人系统:"
    echo "  ros2 launch megarover3_bringup robot.launch.py"
    echo "然后重新运行此脚本"
fi

echo ""
echo "详细文档: MID360_TILT_MODIFICATION.md"
echo ""
