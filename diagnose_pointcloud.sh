#!/bin/bash
# RTABMAP点云问题诊断脚本

echo "=========================================="
echo "RTABMAP点云数据诊断"
echo "=========================================="
echo ""

source /home/wang/Code/Demo6/install/setup.bash

echo "📋 诊断步骤："
echo ""

# 1. 检查节点运行状态
echo "1️⃣  检查关键节点是否运行..."
echo "---"
nodes=("livox_lidar_publisher" "imu_to_tf" "lidar_deskewing" "icp_odometry" "rtabmap")
for node in "${nodes[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo "  ✓ $node - 运行中"
    else
        echo "  ✗ $node - 未运行 ⚠️"
    fi
done
echo ""

# 2. 检查点云话题
echo "2️⃣  检查点云话题发布情况..."
echo "---"
topics=("/livox/lidar" "/livox/lidar/deskewed" "/livox/imu")
for topic in "${topics[@]}"; do
    if timeout 1 ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo "  ✓ $topic - 话题存在"

        # 检查话题频率
        echo -n "    频率: "
        timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3 " Hz"}' || echo "无数据 ⚠️"

        # 检查消息数量
        echo -n "    消息数: "
        timeout 1 ros2 topic echo "$topic" --once 2>/dev/null >/dev/null && echo "有数据 ✓" || echo "无数据 ⚠️"
    else
        echo "  ✗ $topic - 话题不存在 ⚠️"
    fi
done
echo ""

# 3. 检查TF树
echo "3️⃣  检查TF树完整性..."
echo "---"
frames=("mid360_lidar" "mid360_lidar_stabilized" "base_link" "odom" "icp_odom")
for frame in "${frames[@]}"; do
    if timeout 2 ros2 run tf2_ros tf2_echo map "$frame" 2>/dev/null | grep -q "At time"; then
        echo "  ✓ $frame - TF存在"
    else
        echo "  ✗ $frame - TF缺失或无法到达 ⚠️"
    fi
done
echo ""

# 4. 检查rtabmap订阅
echo "4️⃣  检查rtabmap订阅的话题..."
echo "---"
if ros2 node list 2>/dev/null | grep -q "rtabmap"; then
    echo "rtabmap节点订阅的话题："
    ros2 node info /rtabmap 2>/dev/null | grep -A 20 "Subscribers:" | grep ":" | head -10
else
    echo "  ✗ rtabmap节点未运行 ⚠️"
fi
echo ""

# 5. 检查rtabmap参数
echo "5️⃣  检查rtabmap关键参数..."
echo "---"
if ros2 node list 2>/dev/null | grep -q "rtabmap"; then
    params=("frame_id" "subscribe_scan_cloud" "subscribe_depth" "subscribe_rgb")
    for param in "${params[@]}"; do
        value=$(ros2 param get /rtabmap "$param" 2>/dev/null | awk '{print $2}')
        if [ -n "$value" ]; then
            echo "  $param: $value"
        fi
    done
else
    echo "  ✗ rtabmap节点未运行，无法检查参数 ⚠️"
fi
echo ""

# 6. 检查点云消息类型
echo "6️⃣  检查点云消息类型..."
echo "---"
for topic in "/livox/lidar" "/livox/lidar/deskewed"; do
    if timeout 1 ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        msg_type=$(ros2 topic info "$topic" 2>/dev/null | grep "Type:" | awk '{print $2}')
        echo "  $topic: $msg_type"
    fi
done
echo ""

# 7. 查看rtabmap统计信息
echo "7️⃣  检查rtabmap统计信息..."
echo "---"
if timeout 2 ros2 topic echo /rtabmap/info --once 2>/dev/null | grep -q "loop"; then
    echo "  ✓ rtabmap正在发布统计信息"
    timeout 2 ros2 topic echo /rtabmap/info --once 2>/dev/null | grep -E "(loop_closures|local_map_size)" | head -5
else
    echo "  ⚠️  未收到rtabmap统计信息"
fi
echo ""

echo "=========================================="
echo "🔍 常见问题和解决方案"
echo "=========================================="
echo ""
echo "❌ 问题1: Livox驱动未启动或点云话题无数据"
echo "   解决: 检查Livox连接，重启驱动"
echo "   命令: ros2 topic hz /livox/lidar"
echo ""
echo "❌ 问题2: 去畸变节点未运行或输出话题无数据"
echo "   解决: 检查imu_to_tf和lidar_deskewing节点"
echo "   命令: ros2 node list | grep -E 'imu_to_tf|lidar_deskewing'"
echo ""
echo "❌ 问题3: TF树不完整（mid360_lidar_stabilized缺失）"
echo "   解决: 确保imu_to_tf节点正常运行"
echo "   命令: ros2 run tf2_ros tf2_echo mid360_lidar_stabilized mid360_lidar"
echo ""
echo "❌ 问题4: rtabmap话题映射错误"
echo "   解决: 检查launch文件中的remappings"
echo "   检查: ros2 node info /rtabmap"
echo ""
echo "❌ 问题5: rtabmap的frame_id设置错误"
echo "   解决: frame_id必须是'mid360_lidar'"
echo "   检查: ros2 param get /rtabmap frame_id"
echo ""
echo "❌ 问题6: subscribe_scan_cloud未启用"
echo "   解决: 确保rtabmap配置中subscribe_scan_cloud=true"
echo "   检查: ros2 param get /rtabmap subscribe_scan_cloud"
echo ""
echo "=========================================="
echo "💡 快速诊断命令"
echo "=========================================="
echo ""
echo "# 实时监控点云频率"
echo "ros2 topic hz /livox/lidar"
echo ""
echo "# 查看点云数据内容"
echo "ros2 topic echo /livox/lidar --once"
echo ""
echo "# 查看去畸变后的点云"
echo "ros2 topic hz /livox/lidar/deskewed"
echo ""
echo "# 检查TF变换"
echo "ros2 run tf2_ros tf2_echo mid360_lidar_stabilized mid360_lidar"
echo ""
echo "# 查看rtabmap订阅的所有话题"
echo "ros2 node info /rtabmap"
echo ""
echo "# 查看rtabmap日志"
echo "ros2 node list | grep rtabmap"
echo ""
echo "=========================================="
