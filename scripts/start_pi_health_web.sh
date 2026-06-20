#!/usr/bin/env bash
set -eo pipefail

cd /home/hh89669411/ros2_ws
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
source /opt/ros/jazzy/setup.bash
source /home/hh89669411/ros2_ws/install/setup.bash

exec ros2 run articubot_one pi_health_web.py --host 0.0.0.0 --port 8090
