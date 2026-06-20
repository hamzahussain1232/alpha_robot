#!/bin/bash
# Comprehensive diagnostic script for keyboard drive control

echo "========================================="
echo "KEYBOARD DRIVE DIAGNOSTICS"
echo "========================================="
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Check ROS2 environment
echo -e "${BLUE}[1] ROS2 Setup Check${NC}"
echo "COLCON_IGNORE: ${COLCON_IGNORE:-not set}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set (default 0)}"
echo ""

# Check if ros2 daemon is running
if pgrep -x "ros2" > /dev/null; then
    echo -e "${GREEN}✓ ROS2 daemon is running${NC}"
else
    echo -e "${YELLOW}⚠ ROS2 daemon not running (normal on first launch)${NC}"
fi
echo ""

# Check serial connection
echo -e "${BLUE}[2] Serial Port Check${NC}"
SERIAL_PORT="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
if [ -e "$SERIAL_PORT" ]; then
    echo -e "${GREEN}✓ Serial port found: $SERIAL_PORT${NC}"
    echo "  Port permissions: $(ls -l $SERIAL_PORT | awk '{print $1, $3, $4}')"
else
    echo -e "${RED}✗ Serial port NOT found${NC}"
    echo "  Expected: $SERIAL_PORT"
    echo "  Available serial ports:"
    if ls /dev/serial/by-id/ 2>/dev/null | grep -q .; then
        ls -la /dev/serial/by-id/
    else
        echo "    (none found - check USB connection)"
    fi
fi
echo ""

# Check launch files
echo -e "${BLUE}[3] Launch Files Check${NC}"
LAUNCH_FILE="launch/drive_real.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo -e "${GREEN}✓ Found: $LAUNCH_FILE${NC}"
    if grep -q "wasd_teleop" "$LAUNCH_FILE"; then
        echo "  ✓ Contains wasd_teleop node"
    fi
    if grep -q "gnome-terminal" "$LAUNCH_FILE"; then
        echo -e "  ${GREEN}✓ Uses gnome-terminal prefix (FIXED)${NC}"
    else
        echo -e "  ${YELLOW}⚠ Missing gnome-terminal prefix${NC}"
    fi
else
    echo -e "${RED}✗ Not found: $LAUNCH_FILE${NC}"
fi
echo ""

# Check Python scripts
echo -e "${BLUE}[4] Python Scripts Check${NC}"
for script in wasd_teleop.py keyboard_bridge.py serial_diffdrive_node.py; do
    if [ -f "scripts/$script" ]; then
        echo -e "${GREEN}✓ Found: scripts/$script${NC}"
        if [ "$script" = "wasd_teleop.py" ]; then
            if grep -q "except Exception" "scripts/$script"; then
                echo "  ✓ Has exception handling for stdin"
            fi
        fi
    else
        echo -e "${RED}✗ Not found: scripts/$script${NC}"
    fi
done
echo ""

# Check config files
echo -e "${BLUE}[5] Configuration Files Check${NC}"
CONFIG_FILE="config/twist_mux.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo -e "${GREEN}✓ Found: $CONFIG_FILE${NC}"
    echo "  Topics configured in mux:"
    grep "topic.*:" "$CONFIG_FILE" | head -5
else
    echo -e "${RED}✗ Not found: $CONFIG_FILE${NC}"
fi
echo ""

# Check topic subscriptions
echo -e "${BLUE}[6] Topic Flow Check${NC}"
echo "Expected topic flow:"
echo "  wasd_teleop.py → /cmd_vel"
echo "  keyboard_bridge.py: /cmd_vel → /key_vel (TwistStamped)"
echo "  twist_mux: /key_vel → /diff_cont/cmd_vel"
echo "  serial_diffdrive_node.py: /diff_cont/cmd_vel → Serial"
echo ""

# Check if we're in the right directory
echo -e "${BLUE}[7] Directory Check${NC}"
CURRENT_DIR=$(basename "$PWD")
if [ "$CURRENT_DIR" = "articubot_one" ]; then
    echo -e "${GREEN}✓ Working in correct directory: articubot_one${NC}"
else
    echo -e "${YELLOW}⚠ Current directory: $CURRENT_DIR${NC}"
    echo "  Expected to be in: articubot_one"
fi
echo ""

# Recommendations
echo -e "${BLUE}[8] Recommendations${NC}"
echo ""
echo "To test keyboard drive control, run:"
echo -e "  ${YELLOW}ros2 launch articubot_one drive_real.launch.py enable_teleop:=true${NC}"
echo ""
echo "To debug with verbose output, run:"
echo -e "  ${YELLOW}ros2 launch articubot_one serial_drive.launch.py debug_serial:=true${NC}"
echo ""
echo "To monitor topics in another terminal:"
echo -e "  ${YELLOW}ros2 topic list${NC}"
echo -e "  ${YELLOW}ros2 topic echo /cmd_vel${NC}"
echo -e "  ${YELLOW}ros2 topic echo /key_vel${NC}"
echo ""

