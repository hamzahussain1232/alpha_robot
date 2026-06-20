cmake_minimum_required(VERSION 3.8)
project(articubot_one)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 1. Install Python modules (if you have an 'articubot_one' folder inside 'articubot_one')
# If you do NOT have a folder named 'articubot_one' with an '__init__.py' inside your src,
# you should comment this line out, or the build will fail saying "package not found".
# ament_python_install_package(${PROJECT_NAME})

# 2. Install Directories (Launch, Config, Description, Assets)
install(
  DIRECTORY assets config description launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gps_waypoint_follower.py
  scripts/xy_waypoint_follower.py
  scripts/start_clean_keyboard_drive.sh
  scripts/stop_drive_stack.sh
  scripts/keyboard_bridge.py
  scripts/wasd_teleop.py
  scripts/pi_health_monitor.py
  scripts/pi_health_web.py
  scripts/start_pi_health_web.sh
  scripts/arm_calibration.py
  scripts/arm_web_teleop.py
  scripts/nano_arm_keyboard.py
  scripts/nano_arm_keyboard_ros.py
  scripts/nano_arm_set_pose.py
  scripts/nano_arm_sweep.py
  scripts/nano_arm_driver.py
  scripts/voice_command_node.py
  scripts/voice_mic_node.py
  scripts/voice_text_publisher.py
  scripts/object_query_node.py
  scripts/task_manager_node.py
  scripts/object_detector_node.py
  scripts/camera_safety_node.py
  scripts/marker_detector_node.py
  scripts/hand_gesture_teleop_node.py
  scripts/capture_dataset_node.py
  scripts/prepare_yolo_dataset.py
  scripts/serial_diffdrive_node.py
  scripts/serial_selftest.py
  scripts/twist_stamper.py
  scripts/fetch_bottle_node.py
  scripts/omni_voice_server.py
  scripts/omni_robot_commander.py
  scripts/red_object_picker.py
  scripts/finger_arm_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Standard ROS 2 Package Export
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
