#!/bin/bash
# D455 前方相机启动脚本 (x86_64 平台)
# 强制使用 ROS2 自带的 librealsense 2.56.4

echo "启动 D455 前方相机 (SN: 239222302509)..."

# 设置库路径，优先使用 ROS 的 librealsense 2.56.4
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$LD_LIBRARY_PATH

ros2 launch realsense2_camera rs_launch.py \
    camera_name:=d455_front \
    serial_no:="'239222302509'" \
    depth_module.depth_profile:=640x480x15 \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    publish_tf:=false

# 启动后可以手动执行启用点云:
# ros2 param set /camera/d455_front pointcloud.enable true
