#!/usr/bin/env bash
set -eo pipefail

WS_DEFAULT="$HOME/ros2_ws"
WS="${1:-$WS_DEFAULT}"

if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "[ERROR] setup.bash not found at: $WS/install/setup.bash"
  echo "Build first: cd $WS && colcon build --packages-select articubot_one --symlink-install"
  exit 1
fi

echo "[INFO] Using workspace: $WS"
source "$WS/install/setup.bash"

echo "[INFO] Killing stale teleop/drive processes..."
pkill -f "ros2 run articubot_one wasd_teleop.py" 2>/dev/null || true
pkill -f "install/articubot_one/lib/articubot_one/wasd_teleop.py" 2>/dev/null || true
pkill -f "ros2 launch articubot_one drive_real.launch.py" 2>/dev/null || true
pkill -f "install/articubot_one/lib/articubot_one/serial_diffdrive_node.py" 2>/dev/null || true
pkill -f "install/articubot_one/lib/articubot_one/keyboard_bridge.py" 2>/dev/null || true

sleep 1

echo "[INFO] Starting real drive stack in background..."
ros2 launch articubot_one drive_real.launch.py > /tmp/articubot_drive_real.log 2>&1 &
LAUNCH_PID=$!

echo "[INFO] Launch PID: $LAUNCH_PID"
echo "[INFO] Log file: /tmp/articubot_drive_real.log"

cleanup() {
  echo ""
  echo "[INFO] Stopping launch PID $LAUNCH_PID"
  kill "$LAUNCH_PID" 2>/dev/null || true
  sleep 0.5
  pkill -f "ros2 launch articubot_one drive_real.launch.py" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

sleep 2

echo "[INFO] Starting keyboard teleop in this terminal..."
echo "[INFO] Controls: W/A/S/D move, X or Space stop, Q quit"
ros2 run articubot_one wasd_teleop.py
