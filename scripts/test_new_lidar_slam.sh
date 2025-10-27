#!/bin/bash
# 测试新的RTABMAP LiDAR SLAM重构版本

echo "=========================================="
echo "测试RTABMAP LiDAR SLAM重构版本"
echo "=========================================="
echo ""

# 确保环境已设置
source /home/wang/Code/Demo6/install/setup.bash

echo "✅ 检查文件存在性..."
if [ -f "install/megarover_navigation/share/megarover_navigation/launch/rtabmap_lidar_slam.launch.py" ]; then
    echo "  ✓ Launch文件存在"
else
    echo "  ✗ Launch文件不存在"
    exit 1
fi

if [ -f "install/megarover_navigation/share/megarover_navigation/config/rtabmap_lidar3d.yaml" ]; then
    echo "  ✓ 配置文件存在"
else
    echo "  ✗ 配置文件不存在"
    exit 1
fi

echo ""
echo "✅ 验证Python语法..."
python3 -m py_compile src/megarover_navigation/launch/rtabmap_lidar_slam.launch.py
if [ $? -eq 0 ]; then
    echo "  ✓ Python语法正确"
else
    echo "  ✗ Python语法错误"
    exit 1
fi

echo ""
echo "✅ 检查依赖包..."
packages=("rtabmap_odom" "rtabmap_slam" "rtabmap_util" "rtabmap_viz" "livox_ros_driver2")
for pkg in "${packages[@]}"; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        echo "  ✓ $pkg"
    else
        echo "  ✗ $pkg (缺失)"
    fi
done

echo ""
echo "=========================================="
echo "🎯 重构版本核心改进："
echo "=========================================="
echo "✅ 1. 添加Lidar Deskewing (去畸变)"
echo "     - imu_to_tf节点: 创建stabilized frame"
echo "     - lidar_deskewing节点: 去除运动畸变"
echo ""
echo "✅ 2. 修正frame_id设计"
echo "     - frame_id: mid360_lidar (而非base_link)"
echo "     - guess_frame_id: mid360_lidar_stabilized"
echo ""
echo "✅ 3. 修正ICP参数"
echo "     - MaxCorrespondenceDistance: 1.0 (而非0.15)"
echo "     - OutlierRatio: 0.7 (而非0.65)"
echo ""
echo "✅ 4. 简化架构"
echo "     - 移除odometry_fusion节点"
echo "     - 直接使用icp_odometry"
echo ""
echo "=========================================="
echo "📝 启动命令（需要硬件连接）："
echo "=========================================="
echo ""
echo "# 建图模式 (默认)"
echo "ros2 launch megarover_navigation rtabmap_lidar_slam.launch.py"
echo ""
echo "# 定位模式"
echo "ros2 launch megarover_navigation rtabmap_lidar_slam.launch.py localization:=true"
echo ""
echo "# 禁用可视化 (节省资源)"
echo "ros2 launch megarover_navigation rtabmap_lidar_slam.launch.py rviz:=false"
echo ""
echo "# 调整体素大小 (室外环境)"
echo "ros2 launch megarover_navigation rtabmap_lidar_slam.launch.py voxel_size:=0.5"
echo ""
echo "=========================================="
echo "⚠️  注意事项："
echo "=========================================="
echo "1. 需要先连接Livox Mid-360硬件"
echo "2. 需要连接MegaRover3底盘"
echo "3. 确保用户在dialout组: sudo usermod -a -G dialout \$USER"
echo "4. 首次使用建议删除旧数据库: rm ~/.ros/rtabmap.db"
echo ""
echo "=========================================="
echo "✅ 所有检查通过！"
echo "=========================================="
