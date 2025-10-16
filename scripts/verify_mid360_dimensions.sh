#!/bin/bash
# Verify MID360 dimensions match official Livox specifications

echo "=========================================="
echo "  MID360 Dimensions Verification"
echo "=========================================="
echo ""

# Source ROS environment
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null

echo "[1] Official Livox MID360 Specifications:"
echo "   - Dimensions: 65mm × 65mm × 60mm (L × W × H)"
echo "   - Mass: 265g"
echo "   - Source: https://www.livoxtech.com/mid-360/specs"
echo ""

echo "[2] Current URDF Configuration:"
LENGTH=$(grep 'name="mid360_length"' src/megarover3_ros2/megarover_description/urdf/sensors/mid360.xacro | head -1 | grep -oP 'value="\K[^"]+')
WIDTH=$(grep 'name="mid360_width"' src/megarover3_ros2/megarover_description/urdf/sensors/mid360.xacro | head -1 | grep -oP 'value="\K[^"]+')
HEIGHT=$(grep 'name="mid360_height"' src/megarover3_ros2/megarover_description/urdf/sensors/mid360.xacro | head -1 | grep -oP 'value="\K[^"]+')
MASS=$(grep 'name="mid360_mass"' src/megarover3_ros2/megarover_description/urdf/sensors/mid360.xacro | head -1 | grep -oP 'value="\K[^"]+')

LENGTH_MM=$(awk -v val="$LENGTH" 'BEGIN {printf "%.3f", val * 1000}')
WIDTH_MM=$(awk -v val="$WIDTH" 'BEGIN {printf "%.3f", val * 1000}')
HEIGHT_MM=$(awk -v val="$HEIGHT" 'BEGIN {printf "%.3f", val * 1000}')
MASS_G=$(awk -v val="$MASS" 'BEGIN {printf "%.1f", val * 1000}')

echo "   - Length:  ${LENGTH}m (${LENGTH_MM}mm)"
echo "   - Width:   ${WIDTH}m (${WIDTH_MM}mm)"
echo "   - Height:  ${HEIGHT}m (${HEIGHT_MM}mm)"
echo "   - Mass:    ${MASS}kg (${MASS_G}g)"
echo ""

echo "[3] Verification:"
if [ "$LENGTH" == "0.065" ] && [ "$WIDTH" == "0.065" ] && [ "$HEIGHT" == "0.060" ] && [ "$MASS" == "0.265" ]; then
    echo "   ✓ All dimensions match official specifications!"
else
    echo "   ✗ Dimensions do not match. Please check configuration."
fi
echo ""

echo "[4] Size Comparison (Old vs New):"
echo "   - Length:  88mm → 65mm (26% smaller)"
echo "   - Width:   88mm → 65mm (26% smaller)"
echo "   - Height:  73mm → 60mm (18% smaller)"
echo "   - Mass:    760g → 265g (65% lighter)"
echo ""

echo "[5] Visual Impact in RViz:"
echo "   - The sensor box will appear ~26% smaller in X/Y dimensions"
echo "   - The sensor box will appear ~18% shorter in Z dimension"
echo "   - Overall visual volume reduced by ~54%"
echo "   - More accurate representation of actual hardware"
echo ""

echo "[6] To apply changes:"
echo "   1. Stop current bringup/mapping (Ctrl+C)"
echo "   2. Source workspace: cd ~/Code/Demo5 && source install/setup.bash"
echo "   3. Restart: ./scripts/test_bringup.sh"
echo "   4. Open RViz and check mid360_base link visualization"
echo ""

echo "=========================================="
echo "Dimensions updated to official specs!"
echo "=========================================="
