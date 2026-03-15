#!/bin/bash

# 1. Source ROS 2 (Jazzy)
source /opt/ros/jazzy/setup.bash

# 2. Build the workspace
# (Note: In a finished robot, you usually disable 'colcon build' to make booting faster)
cd /home/ros/robot_ws
colcon build --symlink-install

# 3. Source the Local Workspace
source /home/ros/robot_ws/install/setup.bash

# 4. Play startup sound (Optional)
#aplay -D default:CARD=Set ~/wav/cat_meow.wav
#aplay ~/wav/cat_meow.wav

# ==============================================================================
# LAUNCH THE ROBOT
# We use 'ros2 launch package filename' so it finds the installed files correctly.
# ==============================================================================

# Launch the Drive System (Motors, Encoders, Robot State Publisher)
# This uses the 'drive.launch.py' file we fixed earlier.
ros2 launch articubot_one drive.launch.py

# Optional: Launch Camera in background (uncomment if needed)
# ros2 launch articubot_one camera.launch.py &