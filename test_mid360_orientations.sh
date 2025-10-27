#!/bin/bash
# Mid-360方向测试脚本
# 测试不同的yaw角度来找到正确的物理安装朝向

echo "=========================================="
echo "  Mid-360方向测试工具"
echo "=========================================="
echo ""
echo "请选择测试方向（根据您实际看到的Mid-360接线口朝向）："
echo ""
echo "1) 接线口朝向机器人后方 (yaw=0°, 标准朝向)"
echo "2) 接线口朝向机器人右侧 (yaw=90°, 逆时针旋转90度)"
echo "3) 接线口朝向机器人前方 (yaw=180°)"
echo "4) 接线口朝向机器人左侧 (yaw=-90°, 顺时针旋转90度)"
echo ""
echo "当前URDF配置: rpy=\"0 -0.5236 0\" (pitch=-30°, yaw=0°)"
echo ""

read -p "请输入选项 (1-4): " choice

case $choice in
    1)
        yaw=0
        yaw_deg=0
        desc="接线口朝后（标准）"
        ;;
    2)
        yaw=1.5708
        yaw_deg=90
        desc="接线口朝右（逆时针90°）"
        ;;
    3)
        yaw=3.14159
        yaw_deg=180
        desc="接线口朝前"
        ;;
    4)
        yaw=-1.5708
        yaw_deg=-90
        desc="接线口朝左（顺时针90°）"
        ;;
    *)
        echo "无效选项"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "测试配置: $desc"
echo "RPY: roll=0°, pitch=-30°, yaw=$yaw_deg°"
echo "=========================================="
echo ""

# 创建临时测试URDF
TEMP_URDF="/tmp/test_mid360_urdf.xacro"
URDF_FILE="/home/wang/Code/Demo6/src/megarover3_ros2/megarover_description/urdf/mega3.xacro"

# 备份当前配置
cp "$URDF_FILE" "${URDF_FILE}.backup_orientation_test"

# 修改URDF
sed -i "s|<origin xyz=\"0 0.1073 0.375\" rpy=\"0 -0.5236 .*\" />|<origin xyz=\"0 0.1073 0.375\" rpy=\"0 -0.5236 $yaw\" />|g" "$URDF_FILE"

echo "✓ URDF已临时修改"
echo ""
echo "正在编译..."
cd /home/wang/Code/Demo6
source install/setup.bash
colcon build --packages-select megarover_description --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | grep -E "Finished|ERROR" || echo "编译完成"

echo ""
echo "正在启动RViz可视化..."
echo ""
echo "请在RViz中："
echo "  1. 观察mid360_base_link的X轴（红色）朝向"
echo "  2. 检查是否与您实际的Mid-360 Livox Logo方向一致"
echo "  3. 按Ctrl+C退出"
echo ""
echo "=========================================="

source install/setup.bash
ros2 launch megarover_navigation visualize_robot.launch.py

# 测试结束后恢复原配置
echo ""
echo "恢复原始配置..."
mv "${URDF_FILE}.backup_orientation_test" "$URDF_FILE"
echo "✓ 已恢复"
