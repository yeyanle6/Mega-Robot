#!/bin/bash
# Pre-test Hardware and Software Check Script

echo "=========================================="
echo "  T-Robot SLAM Pre-Test Check"
echo "=========================================="
echo ""

PASS=0
FAIL=0

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
    ((PASS++))
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    ((FAIL++))
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

echo "[1/7] Checking ROS2 Environment..."
if [ -z "$ROS_DISTRO" ]; then
    check_fail "ROS_DISTRO not set. Run: source /opt/ros/humble/setup.bash"
else
    check_pass "ROS2 environment: $ROS_DISTRO"
fi

if [ -f "install/setup.bash" ]; then
    check_pass "Workspace found"
else
    check_fail "Workspace not built. Run: colcon build"
fi

echo ""
echo "[2/7] Checking Network Configuration..."

# Check IP configuration
MY_IP=$(ip addr show | grep 'inet 192.168.1' | awk '{print $2}' | cut -d'/' -f1)
if [ -n "$MY_IP" ]; then
    check_pass "Host IP: $MY_IP (in MID360 subnet 192.168.1.x)"
else
    check_fail "No 192.168.1.x IP found. Configure network for MID360"
fi

# Check MID360 connection (actual IP: 192.168.1.3)
if ping -c 1 -W 1 192.168.1.3 &> /dev/null; then
    check_pass "MID360 reachable at 192.168.1.3"
elif ping -c 1 -W 1 192.168.1.1 &> /dev/null; then
    check_pass "MID360 reachable at 192.168.1.1 (default IP)"
else
    check_warn "MID360 not responding (tried 192.168.1.1 and 192.168.1.3)"
fi

echo ""
echo "[3/7] Checking Serial Devices..."

# Check for serial ports
SERIAL_DEVICES=$(ls /dev/ttyUSB* 2>/dev/null)
if [ -n "$SERIAL_DEVICES" ]; then
    check_pass "Serial devices found: $SERIAL_DEVICES"

    # Check permissions
    for dev in $SERIAL_DEVICES; do
        if [ -r "$dev" ] && [ -w "$dev" ]; then
            check_pass "  $dev: Read/Write OK"
        else
            check_warn "  $dev: Permission denied (run: sudo chmod 666 $dev)"
        fi
    done
else
    check_warn "No /dev/ttyUSB* devices found"
fi

echo ""
echo "[4/7] Checking ROS2 Packages..."

source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "install/setup.bash" ] && source install/setup.bash 2>/dev/null

# Check critical packages
REQUIRED_PKGS=(
    "t_robot_bringup"
    "t_robot_slam"
    "robot_localization"
    "rtabmap_ros"
    "nav2_bringup"
)

for pkg in "${REQUIRED_PKGS[@]}"; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        check_pass "Package: $pkg"
    else
        check_fail "Package missing: $pkg"
    fi
done

echo ""
echo "[5/7] Checking Disk Space..."

DISK_AVAIL=$(df -h . | tail -1 | awk '{print $4}')
DISK_AVAIL_NUM=$(df . | tail -1 | awk '{print $4}')

if [ $DISK_AVAIL_NUM -gt 5000000 ]; then
    check_pass "Disk space: $DISK_AVAIL available (>5GB OK)"
else
    check_warn "Disk space: $DISK_AVAIL (recommend >5GB)"
fi

echo ""
echo "[6/7] Checking System Resources..."

# CPU cores
CPU_CORES=$(nproc)
check_pass "CPU cores: $CPU_CORES"

# Memory
MEM_TOTAL=$(free -h | grep Mem | awk '{print $2}')
MEM_AVAIL=$(free -h | grep Mem | awk '{print $7}')
check_pass "Memory: $MEM_AVAIL available / $MEM_TOTAL total"

# Check if enough memory available
MEM_AVAIL_NUM=$(free -m | grep Mem | awk '{print $7}')
if [ $MEM_AVAIL_NUM -lt 2000 ]; then
    check_warn "Low memory (<2GB available). Close other applications."
fi

echo ""
echo "[7/7] Verifying Test Scripts..."

TEST_SCRIPTS=(
    "scripts/test_bringup.sh"
    "scripts/test_mapping.sh"
    "scripts/test_navigation.sh"
)

for script in "${TEST_SCRIPTS[@]}"; do
    if [ -x "$script" ]; then
        check_pass "$(basename $script) (executable)"
    elif [ -f "$script" ]; then
        check_warn "$(basename $script) (exists but not executable - run: chmod +x $script)"
    else
        check_fail "$(basename $script) (missing)"
    fi
done

echo ""
echo "=========================================="
echo "  Pre-Test Check Summary"
echo "=========================================="
echo -e "${GREEN}Passed:${NC} $PASS"
echo -e "${RED}Failed:${NC} $FAIL"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✓ System ready for testing!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Run: ./test_bringup.sh"
    echo "  2. Check output and verify all sensors"
    echo "  3. Run: ./test_mapping.sh"
    echo ""
else
    echo -e "${RED}✗ Please fix the failed checks before testing${NC}"
    echo ""
    echo "Common fixes:"
    echo "  - ROS environment: source /opt/ros/humble/setup.bash"
    echo "  - Workspace: colcon build"
    echo "  - Network: sudo ifconfig eth0 192.168.1.50"
    echo "  - Serial: sudo chmod 666 /dev/ttyUSB*"
    echo ""
fi

echo "For detailed test procedure, see: TEST_PROCEDURE.md"
echo "For quick start, see: QUICKSTART.md"
echo "=========================================="

exit $FAIL
