#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

exec ros2 run articubot_one pi_health_web.py --host 0.0.0.0 --port 8090
