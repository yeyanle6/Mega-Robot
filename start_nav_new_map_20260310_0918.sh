#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="/home/ros/Code/Demo8"
SETUP_BASH="$WORKSPACE/install/setup.bash"
MAP_DIR="$WORKSPACE/maps/new_map_20260310_0918"
MAP_YAML="$MAP_DIR/map.yaml"
MAP_PCD="$MAP_DIR/map.pcd"
LOG_DIR="$WORKSPACE/log/nav_new_map_20260310_0918"

MICRO_ROS_LOG="$LOG_DIR/micro_ros_agent.log"
D455_LOG="$LOG_DIR/d455_front.log"
NAV_LOG="$LOG_DIR/nav_stack.log"

mkdir -p "$LOG_DIR"

if [[ ! -f "$SETUP_BASH" ]]; then
  echo "missing setup: $SETUP_BASH" >&2
  exit 1
fi

if [[ ! -f "$MAP_YAML" ]]; then
  echo "missing map yaml: $MAP_YAML" >&2
  exit 1
fi

if [[ ! -f "$MAP_PCD" ]]; then
  echo "missing map pcd: $MAP_PCD" >&2
  exit 1
fi

cleanup_old_processes() {
  pkill -f "micro_ros_agent serial --dev /dev/ttyUSB0" || true
  pkill -f "realsense2_camera rs_launch.py" || true
  pkill -f "camera.d455_front" || true
  pkill -f "fastlio2_pgo_navigation.launch.py mode:=nav" || true
  pkill -f "/opt/ros/humble/lib/nav2_" || true
  pkill -f "/home/ros/Code/Demo8/install/localizer/lib/localizer/localizer_node" || true
  pkill -f "/home/ros/Code/Demo8/install/fastlio2/lib/fastlio2/lio_node" || true
  pkill -f "/home/ros/Code/Demo8/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node" || true
  pkill -f "/home/ros/Code/Demo8/install/megarover3_navigation/lib/megarover3_navigation/nav_initializer.py" || true
  pkill -f "/home/ros/Code/Demo8/install/megarover3_navigation/lib/megarover3_navigation/pointcloud_relay.py" || true
  pkill -f "/home/ros/Code/Demo8/install/megarover3_navigation/lib/megarover3_navigation/d455_nearfield_obstacle_filter.py" || true
  pkill -f "/home/ros/Code/Demo8/install/megarover3_navigation/lib/megarover3_navigation/yolo_obstacle_detector.py" || true
  pkill -f "/home/ros/Code/Demo8/install/megarover3_navigation/lib/megarover3_navigation/bumper_safety_monitor.py" || true
  pkill -f "/opt/ros/humble/lib/rviz2/rviz2 -d .*fastlio2_nav.rviz" || true
  sleep 2
}

start_micro_ros() {
  echo "starting micro-ROS agent..."
  nohup bash -lc "
    source '$SETUP_BASH'
    exec ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4
  " >"$MICRO_ROS_LOG" 2>&1 &
}

start_d455() {
  echo "starting D455 front camera..."
  nohup bash -lc "
    source '$SETUP_BASH'
    export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:\$LD_LIBRARY_PATH
    exec ros2 launch realsense2_camera rs_launch.py \
      camera_name:=d455_front \
      serial_no:=\"'239222302509'\" \
      depth_module.depth_profile:=640x480x15 \
      pointcloud.enable:=true \
      align_depth.enable:=true \
      publish_tf:=false
  " >"$D455_LOG" 2>&1 &
}

start_nav_stack() {
  echo "starting navigation stack..."
  nohup bash -lc "
    source '$SETUP_BASH'
    exec ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py \
      mode:=nav \
      map:='$MAP_YAML' \
      pcd_map:='$MAP_PCD'
  " >"$NAV_LOG" 2>&1 &
}

print_summary() {
  cat <<EOF
navigation startup launched
map yaml: $MAP_YAML
map pcd : $MAP_PCD
logs:
  $MICRO_ROS_LOG
  $D455_LOG
  $NAV_LOG

next:
  1. wait for RViz and localization to settle
  2. if robot pose is off, use 2D Pose Estimate once
  3. then send Nav Goal
EOF
}

echo "using map:"
echo "  $MAP_YAML"
echo "  $MAP_PCD"

cleanup_old_processes
start_micro_ros
sleep 3
start_d455
sleep 5
start_nav_stack
sleep 3
print_summary
