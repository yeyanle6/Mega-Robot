#!/bin/bash
# Fix issues found in pre-test check

echo "=========================================="
echo "  T-Robot SLAM - Issue Fix Script"
echo "=========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "[1/3] Installing missing packages..."
echo ""

# Check if rtabmap_ros is installed
if ! ros2 pkg list | grep -q "^rtabmap_ros$"; then
    echo -e "${YELLOW}Installing rtabmap_ros...${NC}"
    echo "Run: sudo apt update && sudo apt install -y ros-humble-rtabmap-ros"
    echo ""
    echo "Press Enter after installation completes..."
    read
else
    echo -e "${GREEN}✓${NC} rtabmap_ros already installed"
fi

echo ""
echo "[2/3] Network Configuration Check..."
echo ""

# Get current IP
CURRENT_IP=$(ip addr show | grep 'inet 192.168.1' | awk '{print $2}' | cut -d'/' -f1)
echo "Current IP: ${CURRENT_IP}"
echo "Required IP: 192.168.1.50 (for MID360 communication)"
echo ""

if [ "$CURRENT_IP" != "192.168.1.50" ]; then
    echo -e "${YELLOW}⚠${NC} IP address mismatch"
    echo ""
    echo "To fix network configuration, choose one option:"
    echo ""
    echo "Option 1: Temporary change (until reboot)"
    echo "  sudo ip addr add 192.168.1.50/24 dev eth0"
    echo ""
    echo "Option 2: Permanent change (NetworkManager)"
    echo "  nmcli con mod 'Wired connection 1' ipv4.addresses 192.168.1.50/24"
    echo "  nmcli con mod 'Wired connection 1' ipv4.method manual"
    echo "  nmcli con up 'Wired connection 1'"
    echo ""
    echo "Option 3: Edit /etc/netplan/ (if using netplan)"
    echo ""
    echo "Which option do you want to use? (1/2/3/skip): "
    read choice

    case $choice in
        1)
            echo "Running temporary IP change..."
            sudo ip addr add 192.168.1.50/24 dev eth0
            echo "Verifying..."
            ip addr show eth0 | grep 192.168.1.50
            ;;
        2)
            echo "Setting up NetworkManager..."
            sudo nmcli con mod 'Wired connection 1' ipv4.addresses 192.168.1.50/24
            sudo nmcli con mod 'Wired connection 1' ipv4.method manual
            sudo nmcli con up 'Wired connection 1'
            ;;
        3)
            echo "Please manually edit /etc/netplan/*.yaml"
            echo "Add:"
            echo "  ethernets:"
            echo "    eth0:"
            echo "      addresses: [192.168.1.50/24]"
            echo "Then run: sudo netplan apply"
            ;;
        *)
            echo "Skipping network configuration"
            ;;
    esac
else
    echo -e "${GREEN}✓${NC} IP configuration correct"
fi

echo ""
echo "[3/3] Testing MID360 connectivity..."
echo ""

if ping -c 2 -W 2 192.168.1.1 &> /dev/null; then
    echo -e "${GREEN}✓${NC} MID360 is reachable at 192.168.1.1"
else
    echo -e "${RED}✗${NC} Cannot reach MID360 at 192.168.1.1"
    echo ""
    echo "Troubleshooting steps:"
    echo "1. Check MID360 power supply"
    echo "2. Check Ethernet cable connection"
    echo "3. Verify MID360 IP is set to 192.168.1.1 (default)"
    echo "4. Check network interface is up:"
    echo "   ip link show eth0"
    echo "5. Try manual ping:"
    echo "   ping 192.168.1.1"
fi

echo ""
echo "=========================================="
echo "  Fix Summary"
echo "=========================================="
echo ""
echo "After fixing all issues, run:"
echo "  ./pre_test_check.sh"
echo ""
echo "If all checks pass, start testing:"
echo "  ./test_bringup.sh"
echo "=========================================="
