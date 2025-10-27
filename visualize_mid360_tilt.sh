#!/bin/bash
# Mid-360倾斜角度可视化脚本
# 启动robot_state_publisher和RViz来查看倾斜效果

set -e

echo "=========================================="
echo "  Mid-360倾斜角度可视化"
echo "=========================================="
echo ""

# 加载ROS环境
source /home/wang/Code/Demo6/install/setup.bash

echo "正在启动可视化..."
echo ""
echo "启动内容:"
echo "  1. robot_state_publisher - 发布机器人TF树"
echo "  2. joint_state_publisher - 发布关节状态"
echo "  3. RViz2 - 可视化界面"
echo ""
echo "可视化内容:"
echo "  - 机器人3D模型"
echo "  - TF坐标系 (可看到Mid-360的倾斜30度)"
echo "  - Mid-360向前倾斜的姿态"
echo ""
echo "按 Ctrl+C 停止"
echo ""
echo "=========================================="

# 使用ros2 launch启动
ros2 launch megarover3_bringup robot.launch.py
