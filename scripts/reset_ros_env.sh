#!/bin/bash
# Reset ROS 2 environment for T-Robot SLAM workspace
set -e

# Source ROS 2 (allow failure if workspace not built yet)
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
  source install/setup.bash
fi

# Processes to terminate (ignore failures)
PATTERNS=(
  "micro_ros_agent"
  "ros2 launch"
  "rtabmap"
  "livox_ros_driver2"
  "pointcloud_preprocessor"
  "nav2"
  "ekf_node"
  "megarover3_bringup"
  "t_robot_bringup"
  "python3 .*bringup.launch.py"
  "python3 .*sensors.launch.py"
  "python3 .*state_estimation.launch.py"
  "python3 .*mapping.launch.py"
  "python3 .*navigation.launch.py"
  "joint_state_publisher"
  "robot_state_publisher"
  "pub_odom"
  "static_transform_publisher"
  "time_sync_monitor"
  "topic_tools"
  "mid360_lidar_relay"
)

for pattern in "${PATTERNS[@]}"; do
  pkill -f "$pattern" 2>/dev/null || true
done

# Give processes一段时间退出，再做一次强制清理
sleep 1
for pattern in "${PATTERNS[@]}"; do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

# Restart ROS 2 daemon to clear cached graph state
ros2 daemon stop 2>/dev/null || true
ros2 daemon start >/dev/null 2>&1 || true

# Clean temporary launch artifacts
rm -f /tmp/nav2_test.launch.py 2>/dev/null || true
rm -f frames.* 2>/dev/null || true

# Summarize
echo "ROS 2 environment reset complete."
ros2 node list || true
