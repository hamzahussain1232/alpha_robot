#!/usr/bin/env bash
set -euo pipefail

echo "[INFO] Stopping articubot drive and teleop processes..."
pkill -f "ros2 run articubot_one wasd_teleop.py" 2>/dev/null || true
pkill -f "install/articubot_one/lib/articubot_one/wasd_teleop.py" 2>/dev/null || true
pkill -f "ros2 launch articubot_one drive_real.launch.py" 2>/dev/null || true
pkill -f "install/articubot_one/lib/articubot_one/serial_diffdrive_node.py" 2>/dev/null || true
pkill -f "install/articubot_one/lib/articubot_one/keyboard_bridge.py" 2>/dev/null || true

echo "[INFO] Done."
