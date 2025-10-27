#!/bin/bash
# MegaRover3 导航系统启动脚本（交互式模式选择版）

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检测传感器硬件
detect_sensors() {
    print_info "正在检测传感器硬件..."

    LIVOX_DETECTED=false
    REALSENSE_DETECTED=false

    # 检测Livox MID360
    if ls /dev/ttyUSB* >/dev/null 2>&1; then
        LIVOX_DETECTED=true
        print_success "✓ Livox MID360 已检测到"
    else
        print_warning "✗ Livox MID360 未检测到"
    fi

    # 检测RealSense D455
    if lsusb | grep -i "Intel" | grep -i "RealSense" >/dev/null 2>&1; then
        REALSENSE_DETECTED=true
        print_success "✓ RealSense D455 已检测到"
    else
        print_warning "✗ RealSense D455 未检测到"
    fi
}

# 确定推荐模式
determine_recommended_mode() {
    if [ "$LIVOX_DETECTED" = true ] && [ "$REALSENSE_DETECTED" = true ]; then
        RECOMMENDED_MODE="fusion"
        RECOMMENDED_DESC="融合模式（最高精度）"
    elif [ "$LIVOX_DETECTED" = true ]; then
        RECOMMENDED_MODE="lidar_only"
        RECOMMENDED_DESC="激光雷达模式"
    elif [ "$REALSENSE_DETECTED" = true ]; then
        RECOMMENDED_MODE="rgbd_only"
        RECOMMENDED_DESC="RGB-D模式"
    else
        RECOMMENDED_MODE="odom_only"
        RECOMMENDED_DESC="纯里程计模式（降级）"
    fi
}

# 显示模式选择菜单
show_mode_menu() {
    echo ""
    echo "=========================================="
    echo "  MegaRover3 SLAM模式选择"
    echo "=========================================="
    echo ""
    echo "检测到的传感器："
    [ "$LIVOX_DETECTED" = true ] && echo "  ✓ Livox MID360" || echo "  ✗ Livox MID360"
    [ "$REALSENSE_DETECTED" = true ] && echo "  ✓ RealSense D455" || echo "  ✗ RealSense D455"
    echo ""
    echo "推荐模式: ${RECOMMENDED_DESC}"
    echo ""
    echo "可用模式："
    echo "  1) fusion       - 融合模式 (MID360 + D455, 最高精度)"
    echo "  2) lidar_only   - 激光雷达模式 (仅MID360)"
    echo "  3) rgbd_only    - RGB-D模式 (仅D455)"
    echo "  4) odom_only    - 纯里程计模式 (降级)"
    echo "  0) auto         - 自动模式 (推荐: ${RECOMMENDED_MODE})"
    echo ""
}

# 获取用户选择
get_user_choice() {
    while true; do
        read -p "请选择模式 [0-4，直接回车使用推荐模式]: " choice

        # 如果用户直接回车，使用推荐模式
        if [ -z "$choice" ]; then
            SELECTED_MODE="$RECOMMENDED_MODE"
            print_success "使用推荐模式: ${RECOMMENDED_DESC}"
            break
        fi

        case $choice in
            0)
                SELECTED_MODE="$RECOMMENDED_MODE"
                print_success "使用自动检测模式: ${RECOMMENDED_MODE}"
                break
                ;;
            1)
                SELECTED_MODE="fusion"
                if [ "$LIVOX_DETECTED" = false ] || [ "$REALSENSE_DETECTED" = false ]; then
                    print_warning "警告: 融合模式需要两个传感器，但未全部检测到"
                    read -p "确定要继续吗? (y/N): " confirm
                    if [[ $confirm =~ ^[Yy]$ ]]; then
                        break
                    fi
                else
                    break
                fi
                ;;
            2)
                SELECTED_MODE="lidar_only"
                if [ "$LIVOX_DETECTED" = false ]; then
                    print_warning "警告: 未检测到Livox MID360"
                    read -p "确定要继续吗? (y/N): " confirm
                    [[ $confirm =~ ^[Yy]$ ]] && break
                else
                    break
                fi
                ;;
            3)
                SELECTED_MODE="rgbd_only"
                if [ "$REALSENSE_DETECTED" = false ]; then
                    print_warning "警告: 未检测到RealSense D455"
                    read -p "确定要继续吗? (y/N): " confirm
                    [[ $confirm =~ ^[Yy]$ ]] && break
                else
                    break
                fi
                ;;
            4)
                SELECTED_MODE="odom_only"
                print_warning "使用降级模式，精度受限"
                break
                ;;
            *)
                print_error "无效选择，请输入0-4"
                ;;
        esac
    done
}

# 询问是否启动RViz
ask_rviz() {
    echo ""
    read -p "是否启动RViz可视化? (Y/n): " rviz_choice
    if [[ $rviz_choice =~ ^[Nn]$ ]]; then
        USE_RVIZ="false"
    else
        USE_RVIZ="true"
    fi
}

# 主函数
main() {
    echo ""
    echo "=========================================="
    echo "  MegaRover3 导航系统启动工具"
    echo "=========================================="
    echo ""

    # 检测传感器
    detect_sensors

    # 确定推荐模式
    determine_recommended_mode

    # 显示菜单并获取选择
    show_mode_menu
    get_user_choice

    # 询问RViz
    ask_rviz

    # 确认启动
    echo ""
    echo "=========================================="
    echo "启动配置："
    echo "  SLAM模式: ${SELECTED_MODE}"
    echo "  RViz可视化: ${USE_RVIZ}"
    echo "=========================================="
    echo ""
    read -p "确认启动? (Y/n): " confirm

    if [[ $confirm =~ ^[Nn]$ ]]; then
        print_info "已取消启动"
        exit 0
    fi

    # Source ROS环境
    print_info "加载ROS2环境..."
    source install/setup.bash

    # 启动系统
    print_success "正在启动系统..."
    echo ""

    ros2 launch megarover_navigation modular_rtabmap.launch.py \
        force_mode:=${SELECTED_MODE} \
        rviz:=${USE_RVIZ}
}

# 运行主函数
main
