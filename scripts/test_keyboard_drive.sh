#!/bin/bash
# Quick test script for keyboard drive control

echo "========================================="
echo "KEYBOARD DRIVE CONTROL - TEST SCRIPT"
echo "========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ROS2 is not installed${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 is installed${NC}"
echo ""

# Check serial port
SERIAL_PORT="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
if [ -e "$SERIAL_PORT" ]; then
    echo -e "${GREEN}✓ Serial port found: $SERIAL_PORT${NC}"
else
    echo -e "${YELLOW}⚠ Serial port not found at $SERIAL_PORT${NC}"
    echo "  Available serial ports:"
    ls /dev/serial/by-id/ 2>/dev/null || echo "  (none found)"
fi

echo ""
echo "Starting drive_real launch with keyboard control..."
echo ""
echo -e "${YELLOW}INSTRUCTIONS:${NC}"
echo "1. A new terminal will open for keyboard control"
echo "2. Use WASD keys to drive the robot:"
echo "   - W: Forward"
echo "   - A: Turn Left"
echo "   - S: Backward"
echo "   - D: Turn Right"
echo "   - X or Space: Stop"
echo "   - Q: Quit"
echo ""
echo "Starting in 3 seconds..."
sleep 3

echo ""
echo -e "${GREEN}Launching drive_real.launch.py...${NC}"
ros2 launch articubot_one drive_real.launch.py enable_teleop:=true

