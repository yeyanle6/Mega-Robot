#!/bin/bash
# Verify MID360 30-degree forward tilt configuration

echo "=========================================="
echo "  MID360 30° Forward Tilt Verification"
echo "=========================================="
echo ""

# Source ROS environment
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null

echo "[1] Checking URDF configuration..."
PITCH_VALUE=$(grep "mid360_mount_pitch" src/megarover3_ros2/megarover_description/urdf/calibration_offsets.xacro | grep -oP 'value="\K[^"]+')
if [ "$PITCH_VALUE" == "0.5236" ]; then
    echo "✓ MID360 pitch configured to 0.5236 rad (30°)"
else
    echo "✗ MID360 pitch is $PITCH_VALUE (expected 0.5236)"
fi

MOUNT_Z=$(grep "mid360_mount_z" src/megarover3_ros2/megarover_description/urdf/calibration_offsets.xacro | grep -oP 'value="\K[^"]+')
if [ "$MOUNT_Z" == "0.10" ]; then
    echo "✓ MID360 mount height set to 0.10 m (raised by 8 cm)"
else
    echo "✗ MID360 mount height is $MOUNT_Z m (expected 0.10 m)"
fi

echo ""
echo "[2] Expected transformation:"
echo "   - Roll:  0.0°   (no rotation around X-axis)"
echo "   - Pitch: 30.0°  (forward tilt around Y-axis)"
echo "   - Yaw:   0.0°   (no rotation around Z-axis)"
echo ""
echo "   Effect: Point cloud will be tilted 30° down relative to robot"
echo "   - Ground visibility: Extended forward"
echo "   - Vertical FOV: -59.5° to +0.5° (relative to robot horizontal)"
echo ""

echo "[3] To apply changes, restart the system:"
echo "   1. Stop current bringup/mapping (Ctrl+C in those terminals)"
echo "   2. Source the updated workspace:"
echo "      cd ~/Code/Demo5 && source install/setup.bash"
echo "   3. Restart: ./scripts/test_bringup.sh"
echo "   4. Check TF in RViz: odom → base_link → mid360_base → mid360_lidar"
echo ""

echo "[4] To verify in RViz:"
echo "   - Fixed Frame: base_link"
echo "   - Add TF display"
echo "   - Add PointCloud2: /mid360/lidar"
echo "   - Observe that point cloud is tilted forward 30°"
echo ""

echo "=========================================="
echo "Configuration complete! Restart system to apply."
echo "=========================================="
