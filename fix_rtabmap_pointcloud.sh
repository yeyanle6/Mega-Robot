#!/bin/bash
# RTABMAP点云问题自动诊断和修复脚本

echo "=========================================="
echo "🔍 RTABMAP点云问题 - 精准诊断"
echo "=========================================="
echo ""

source /home/wang/Code/Demo6/install/setup.bash

# 用户已确认：rtabmap订阅了/livox/lidar/deskewed
echo "✅ 已确认：rtabmap正确订阅 /livox/lidar/deskewed"
echo ""

echo "🎯 核心问题定位："
echo "=========================================="
echo ""

# 检查1：去畸变点云话题是否有数据
echo "1️⃣  检查 /livox/lidar/deskewed 是否发布数据..."
echo "---"

if timeout 2 ros2 topic list 2>/dev/null | grep -q "/livox/lidar/deskewed"; then
    echo "  ✓ 话题存在"

    # 检查频率
    echo -n "  📊 检查频率: "
    freq=$(timeout 5 ros2 topic hz /livox/lidar/deskewed 2>&1 | grep "average rate" | awk '{print $3}')

    if [ -n "$freq" ]; then
        echo "$freq Hz ✓"
        echo ""
        echo "  ✅ /livox/lidar/deskewed 正常发布数据！"
        echo ""
        echo "  ⚠️  rtabmap订阅了话题且话题有数据，但看不到点云"
        echo "  🔍 问题可能是："
        echo "     1. TF转换问题 - rtabmap无法将点云转换到显示frame"
        echo "     2. 可视化参数问题 - 点云被过度采样或过滤"
        echo ""
    else
        echo "❌ 无数据"
        echo ""
        echo "  🔴 问题找到了！/livox/lidar/deskewed话题无数据"
        echo ""
        echo "  原因可能是："
        echo "     1. lidar_deskewing节点未运行"
        echo "     2. imu_to_tf节点未运行（去畸变依赖TF）"
        echo "     3. Livox原始点云 /livox/lidar 无数据"
        echo ""
    fi
else
    echo "  ✗ 话题不存在 ❌"
    echo ""
    echo "  🔴 严重问题！/livox/lidar/deskewed话题不存在"
    echo ""
    echo "  原因："
    echo "     lidar_deskewing节点未启动"
    echo ""
fi

# 检查2：关键节点是否运行
echo "2️⃣  检查关键节点运行状态..."
echo "---"

nodes=("livox_lidar_publisher" "imu_to_tf" "lidar_deskewing" "icp_odometry" "rtabmap")
missing_nodes=()

for node in "${nodes[@]}"; do
    if timeout 2 ros2 node list 2>/dev/null | grep -q "/$node"; then
        echo "  ✓ /$node"
    else
        echo "  ✗ /$node ❌"
        missing_nodes+=("$node")
    fi
done
echo ""

if [ ${#missing_nodes[@]} -gt 0 ]; then
    echo "  🔴 缺少关键节点："
    for node in "${missing_nodes[@]}"; do
        echo "     - $node"
    done
    echo ""
fi

# 检查3：Livox原始点云
echo "3️⃣  检查Livox原始点云 /livox/lidar..."
echo "---"

if timeout 2 ros2 topic list 2>/dev/null | grep -q "^/livox/lidar$"; then
    echo "  ✓ 话题存在"

    echo -n "  📊 检查频率: "
    freq=$(timeout 5 ros2 topic hz /livox/lidar 2>&1 | grep "average rate" | awk '{print $3}')

    if [ -n "$freq" ]; then
        echo "$freq Hz ✓"
        echo ""
        echo "  ✅ Livox驱动正常发布点云"
    else
        echo "❌ 无数据"
        echo ""
        echo "  🔴 Livox驱动节点运行但无数据"
        echo "     - 检查Mid-360硬件连接"
        echo "     - 检查USB权限（dialout组）"
    fi
else
    echo "  ✗ 话题不存在 ❌"
    echo ""
    echo "  🔴 Livox驱动未发布点云话题"
fi
echo ""

# 检查4：TF树
echo "4️⃣  检查TF树完整性..."
echo "---"

# 检查关键TF
frames=("mid360_lidar" "mid360_lidar_stabilized")
tf_issues=()

for frame in "${frames[@]}"; do
    if timeout 2 ros2 run tf2_ros tf2_echo map "$frame" 2>/dev/null | grep -q "At time"; then
        echo "  ✓ $frame → map TF存在"
    else
        echo "  ✗ $frame → map TF缺失 ⚠️"
        tf_issues+=("$frame")
    fi
done
echo ""

if [ ${#tf_issues[@]} -gt 0 ]; then
    echo "  ⚠️  TF问题："
    for frame in "${tf_issues[@]}"; do
        echo "     - $frame frame无法转换到map"
    done
    echo ""
    echo "  如果mid360_lidar_stabilized缺失："
    echo "     → imu_to_tf节点未运行或IMU无数据"
    echo ""
fi

# 检查5：rtabmap可视化参数
echo "5️⃣  检查rtabmap可视化参数..."
echo "---"

if timeout 2 ros2 node list 2>/dev/null | grep -q "/rtabmap"; then
    voxel_size=$(ros2 param get /rtabmap cloud_voxel_size 2>/dev/null | awk '{print $NF}')
    max_depth=$(ros2 param get /rtabmap cloud_max_depth 2>/dev/null | awk '{print $NF}')
    min_depth=$(ros2 param get /rtabmap cloud_min_depth 2>/dev/null | awk '{print $NF}')

    echo "  cloud_voxel_size: $voxel_size"
    echo "  cloud_max_depth: $max_depth"
    echo "  cloud_min_depth: $min_depth"
    echo ""

    # 检查是否合理
    if (( $(echo "$voxel_size > 0.1" | bc -l) )); then
        echo "  ⚠️  cloud_voxel_size过大 ($voxel_size > 0.1)"
        echo "     点云会非常稀疏，建议设置为0.05"
    fi
fi
echo ""

echo "=========================================="
echo "💡 智能诊断结果"
echo "=========================================="
echo ""

# 智能判断问题
problem_found=false

if [ ${#missing_nodes[@]} -gt 0 ]; then
    echo "🔴 问题1：关键节点未运行"
    echo ""
    for node in "${missing_nodes[@]}"; do
        case $node in
            "livox_lidar_publisher")
                echo "   ❌ Livox驱动未启动"
                echo "      原因："
                echo "      - Mid-360硬件未连接"
                echo "      - USB权限不足（未加入dialout组）"
                echo "      - livox_ros_driver2包未安装"
                echo ""
                echo "      解决方案："
                echo "      sudo usermod -a -G dialout \$USER"
                echo "      # 然后重新登录"
                ;;
            "imu_to_tf")
                echo "   ❌ imu_to_tf节点未启动"
                echo "      原因："
                echo "      - IMU话题/livox/imu无数据"
                echo "      - Livox驱动未正常启动"
                echo ""
                echo "      影响："
                echo "      - 无法创建mid360_lidar_stabilized frame"
                echo "      - 去畸变无法工作"
                ;;
            "lidar_deskewing")
                echo "   ❌ lidar_deskewing节点未启动"
                echo "      原因："
                echo "      - imu_to_tf节点未运行"
                echo "      - TF: mid360_lidar_stabilized不存在"
                echo ""
                echo "      影响："
                echo "      - /livox/lidar/deskewed话题不会发布"
                echo "      - rtabmap无法接收点云数据"
                ;;
        esac
    done
    problem_found=true
fi

if ! $problem_found; then
    echo "✅ 所有节点正常运行"
    echo ""
    echo "如果rtabmap中仍然看不到点云，可能是："
    echo ""
    echo "1️⃣  rtabmap_viz的可视化参数问题"
    echo "   解决方案："
    echo "   ros2 param set /rtabmap cloud_voxel_size 0.05"
    echo "   ros2 param set /rtabmap cloud_max_depth 20.0"
    echo ""
    echo "2️⃣  点云在rtabmap中但不可见"
    echo "   - 检查rtabmap_viz的显示设置"
    echo "   - 确保'Cloud'图层已启用"
    echo ""
    echo "3️⃣  TF转换延迟"
    echo "   - rtabmap收到点云但TF转换失败"
    echo "   - 增加wait_for_transform时间"
    echo ""
fi

echo "=========================================="
echo "🚀 推荐操作"
echo "=========================================="
echo ""

# 根据诊断结果给出建议
if [[ " ${missing_nodes[@]} " =~ " livox_lidar_publisher " ]]; then
    echo "第1步：修复Livox驱动"
    echo "  1. 检查Mid-360 USB连接"
    echo "  2. 添加用户到dialout组：sudo usermod -a -G dialout \$USER"
    echo "  3. 重新登录或重启"
    echo "  4. 重新启动launch文件"
    echo ""
elif [[ " ${missing_nodes[@]} " =~ " lidar_deskewing " ]]; then
    echo "第1步：修复去畸变节点"
    echo "  原因：imu_to_tf节点可能未运行"
    echo ""
    echo "  临时解决方案（禁用去畸变）："
    echo "  ros2 launch megarover_navigation rtabmap_lidar_slam.launch.py \\"
    echo "      deskewing:=false \\"
    echo "      imu_topic:='' \\"
    echo "      rviz:=true"
    echo ""
    echo "  这样rtabmap会直接订阅/livox/lidar（原始点云）"
    echo ""
else
    echo "💡 尝试以下操作："
    echo ""
    echo "1. 调整可视化参数（实时）："
    echo "   ros2 param set /rtabmap cloud_voxel_size 0.05"
    echo "   ros2 param set /rtabmap cloud_max_depth 20.0"
    echo ""
    echo "2. 检查rtabmap_viz显示设置："
    echo "   - 确保'Cloud'图层已启用"
    echo "   - 调整点云大小和颜色"
    echo ""
    echo "3. 查看rtabmap统计信息："
    echo "   ros2 topic echo /rtabmap/info --once | grep -E '(nodes|cloud)'"
    echo ""
fi

echo "=========================================="
echo "📞 需要更多信息？运行："
echo "=========================================="
echo ""
echo "# 查看去畸变点云详细信息"
echo "ros2 topic echo /livox/lidar/deskewed --once | head -50"
echo ""
echo "# 查看TF树"
echo "ros2 run tf2_tools view_frames"
echo ""
echo "# 实时监控点云频率"
echo "watch -n 1 'ros2 topic hz /livox/lidar/deskewed'"
echo ""
