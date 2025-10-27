#!/bin/bash

# ROS2 Bag录制脚本
# 用于记录SLAM数据以供离线处理和调试

echo "=========================================="
echo "    RTABMAP数据录制工具"
echo "=========================================="
echo ""

# 默认参数
BAG_NAME="rtabmap_$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="$HOME/rosbags"
DURATION=""

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --name)
            BAG_NAME="$2"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --name NAME       指定bag文件名称"
            echo "  --output DIR      指定输出目录（默认: ~/rosbags）"
            echo "  --duration SEC    录制时长（秒）"
            echo "  --help           显示帮助信息"
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            exit 1
            ;;
    esac
done

# 创建输出目录
mkdir -p "$OUTPUT_DIR"
BAG_PATH="$OUTPUT_DIR/$BAG_NAME"

# 要录制的话题
TOPICS=(
    # 传感器数据
    "/livox/lidar"
    "/livox/imu"
    "/camera/color/image_raw"
    "/camera/color/camera_info"
    "/camera/depth/image_rect_raw"
    "/camera/aligned_depth_to_color/image_raw"
    "/camera/imu"

    # 里程计和TF
    "/odom"
    "/tf"
    "/tf_static"

    # 机器人状态
    "/joint_states"

    # RTABMAP输出（用于分析）
    "/rtabmap/grid_map"
    "/rtabmap/cloud_map"
    "/rtabmap/info"
    "/rtabmap/mapData"
    "/rtabmap/odom_info"
)

# 构建话题列表
TOPIC_LIST=""
for topic in "${TOPICS[@]}"; do
    TOPIC_LIST="$TOPIC_LIST $topic"
done

echo "录制配置:"
echo "  输出路径: $BAG_PATH"
if [ ! -z "$DURATION" ]; then
    echo "  录制时长: ${DURATION}秒"
fi
echo ""
echo "录制话题:"
for topic in "${TOPICS[@]}"; do
    echo "  - $topic"
done
echo ""

# 检查话题是否存在
echo "检查话题可用性..."
AVAILABLE_TOPICS=()
MISSING_TOPICS=()

for topic in "${TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
        AVAILABLE_TOPICS+=("$topic")
        echo "✓ $topic"
    else
        MISSING_TOPICS+=("$topic")
        echo "✗ $topic (不可用)"
    fi
done

if [ ${#MISSING_TOPICS[@]} -gt 0 ]; then
    echo ""
    echo "警告: 部分话题不可用，是否继续录制? (y/n)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "录制已取消"
        exit 0
    fi
fi

# 构建可用话题列表
AVAILABLE_TOPIC_LIST=""
for topic in "${AVAILABLE_TOPICS[@]}"; do
    AVAILABLE_TOPIC_LIST="$AVAILABLE_TOPIC_LIST $topic"
done

echo ""
echo "=========================================="
echo "开始录制..."
echo "按 Ctrl+C 停止录制"
echo "=========================================="
echo ""

# 构建录制命令
RECORD_CMD="ros2 bag record -o $BAG_PATH"

# 添加时长参数
if [ ! -z "$DURATION" ]; then
    RECORD_CMD="$RECORD_CMD --max-duration ${DURATION}s"
fi

# 添加话题
RECORD_CMD="$RECORD_CMD $AVAILABLE_TOPIC_LIST"

# 执行录制
$RECORD_CMD

echo ""
echo "=========================================="
echo "录制完成!"
echo "文件保存在: $BAG_PATH"
echo ""
echo "查看录制信息:"
echo "  ros2 bag info $BAG_PATH"
echo ""
echo "回放数据:"
echo "  ros2 bag play $BAG_PATH"
echo "=========================================="