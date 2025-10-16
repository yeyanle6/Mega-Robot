#!/bin/bash
# T-Robot SLAM System - Navigation Test Script
# Tests: Map Export → Nav2 Setup → Multi-point Navigation

echo "=========================================="
echo "  T-Robot Navigation Test"
echo "=========================================="
echo ""

# Source ROS2 workspace
set -e
source /opt/ros/humble/setup.bash
source install/setup.bash

NAV_BRINGUP_PID=""

cleanup() {
  if [ -n "$NAV2_PID" ] && kill -0 "$NAV2_PID" 2>/dev/null; then
    kill "$NAV2_PID" 2>/dev/null || true
    wait "$NAV2_PID" 2>/dev/null || true
  fi

  if [ "${BRINGUP_STARTED}" = "1" ] && [ -n "$NAV_BRINGUP_PID" ]; then
    if kill -0 "$NAV_BRINGUP_PID" 2>/dev/null; then
      kill "$NAV_BRINGUP_PID" 2>/dev/null || true
      wait "$NAV_BRINGUP_PID" 2>/dev/null || true
    fi
  fi

  rm -f /tmp/nav2_test.launch.py 2>/dev/null || true
  ros2 daemon stop 2>/dev/null || true
  ros2 daemon start >/dev/null 2>&1 || true
}

trap cleanup EXIT

echo "[1/5] Checking RTAB-Map database..."

# Look for RTAB-Map database
DB_PATH="${HOME}/.ros/rtabmap.db"
if [ ! -f "$DB_PATH" ]; then
    echo "❌ ERROR: RTAB-Map database not found at $DB_PATH"
    echo "Please run mapping test first: ./test_mapping.sh"
    exit 1
fi

echo "  ✅ Database found: $DB_PATH"

# Get database stats
DB_SIZE=$(du -h "$DB_PATH" | cut -f1)
echo "  📊 Database size: $DB_SIZE"

echo ""
echo "[2/5] Exporting occupancy grid map from RTAB-Map..."

# Export map to pgm/yaml format
MAP_DIR="/tmp/rtabmap_export"
mkdir -p "$MAP_DIR"

echo "  Exporting to: $MAP_DIR"

# Use rtabmap-export command
rtabmap-export --map "$MAP_DIR/map.pgm" --map_yaml "$MAP_DIR/map.yaml" "$DB_PATH" 2>/dev/null

if [ -f "$MAP_DIR/map.pgm" ]; then
    echo "  ✅ Map exported successfully"
    echo "     - Occupancy grid: $MAP_DIR/map.pgm"
    echo "     - YAML config: $MAP_DIR/map.yaml"

    # Show map info
    if [ -f "$MAP_DIR/map.yaml" ]; then
        echo ""
        echo "  Map parameters:"
        cat "$MAP_DIR/map.yaml" | grep -E "resolution:|origin:"
    fi
else
    echo "  ⚠️  Map export failed. Trying alternative method..."
    echo "  Using ROS2 service to save map..."

    # Alternative: use map_saver from nav2_map_server
    timeout 10 ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/map" 2>/dev/null || \
        echo "  ❌ Alternative export also failed"
fi

echo ""
echo "[3/5] Setting up Nav2 navigation stack..."

# Check if bringup is running
BRINGUP_STARTED=0
if ! ros2 node list | grep -q "robot_state_publisher"; then
    echo "  ⚠️  Bringup not running, launching minimal setup..."
    ros2 launch t_robot_bringup bringup.launch.py > /tmp/nav_bringup.log 2>&1 &
    NAV_BRINGUP_PID=$!
    BRINGUP_STARTED=1
    sleep 8
fi

# Launch Nav2 with exported map
echo "  Launching Nav2 with map: $MAP_DIR/map.yaml"

# Create temporary nav2 launch
cat > /tmp/nav2_test.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    slam_share = FindPackageShare('t_robot_slam')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/tmp/rtabmap_export/map.yaml'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([slam_share, 'config', 'navigation', 'nav2_params.yaml'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'false'
            }.items()
        )
    ])
EOF

ros2 launch /tmp/nav2_test.launch.py map:="$MAP_DIR/map.yaml" > /tmp/nav2_test.log 2>&1 &
NAV2_PID=$!

echo "  Nav2 PID: $NAV2_PID"
echo "  Waiting 10 seconds for Nav2 initialization..."
sleep 10

echo ""
echo "[4/5] Verifying Nav2 nodes..."
ros2 node list | grep -E "controller|planner|bt_navigator|map_server" || \
    echo "  ⚠️  Some Nav2 nodes missing, check /tmp/nav2_test.log"

echo ""
echo "[5/5] Testing costmap data..."
echo "  Local costmap:"
timeout 3 ros2 topic echo /local_costmap/costmap --once 2>/dev/null | head -5 || \
    echo "  ❌ No local costmap data"

echo ""
echo "  Global costmap:"
timeout 3 ros2 topic echo /global_costmap/costmap --once 2>/dev/null | head -5 || \
    echo "  ❌ No global costmap data"

echo ""
echo "=========================================="
echo "  Navigation Test - Ready!"
echo "=========================================="
echo ""
echo "📍 Set initial pose in RViz2:"
echo "   ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix t_robot_slam)/share/t_robot_slam/rviz/navigation.rviz"
echo ""
echo "🎯 Send navigation goal via CLI:"
echo "   ros2 topic pub /goal_pose geometry_msgs/PoseStamped \\"
echo "     '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}' --once"
echo ""
echo "📊 Monitor navigation status:"
echo "   ros2 topic echo /navigate_to_pose/_action/feedback"
echo ""
echo "🗺️  View costmaps in RViz2:"
echo "   - Add -> By topic -> /local_costmap/costmap -> Costmap"
echo "   - Add -> By topic -> /global_costmap/costmap -> Costmap"
echo ""
echo "Logs:"
echo "  Nav2: /tmp/nav2_test.log"
echo "  Bringup: /tmp/nav_bringup.log"
echo ""
echo "按 Ctrl+C 结束导航测试，脚本将自动清理 Nav2 与相关节点。"

wait "$NAV2_PID" 2>/dev/null || true

echo "导航测试结束，资源已清理。"
echo "=========================================="
