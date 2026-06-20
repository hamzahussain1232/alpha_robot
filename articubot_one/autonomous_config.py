"""
Configuration for autonomous task executor.

This file defines:
- Available tasks and objects to fetch
- Arm positions for different actions
- Navigation parameters
- Safety limits
- Object detection settings
"""

# ==================== OBJECTS TO FETCH ====================
FETCHABLE_OBJECTS = {
    'medicine': {
        'labels': ['medicine', 'medicine box', 'pills', 'tablet'],
        'priority': 'high',
        'description': 'Medical supplies'
    },
    'water': {
        'labels': ['water', 'bottle', 'water bottle'],
        'priority': 'high',
        'description': 'Water or beverage'
    },
    'cup': {
        'labels': ['cup', 'mug', 'glass', 'drinking cup'],
        'priority': 'medium',
        'description': 'Drinking vessel'
    },
    'phone': {
        'labels': ['phone', 'mobile', 'smartphone'],
        'priority': 'high',
        'description': 'Mobile phone'
    },
    'remote': {
        'labels': ['remote', 'remote control'],
        'priority': 'low',
        'description': 'TV remote'
    }
}

# ==================== ARM POSITIONS (Radians) ====================
# Nano 6-DOF Arm: 6 joints
# Servo angles: 0-180° mapped to 0-3.14 radians

ARM_POSITIONS = {
    'home': {
        'joints': [1.57, 1.57, 1.57, 1.57, 1.57, 0.0],  # Ready position
        'gripper': 0.0,  # Open
        'description': 'Home/rest position'
    },
    'approach_stand': {
        'joints': [1.57, 0.785, 0.785, -0.785, -0.785, 0.0],
        'gripper': 0.0,
        'description': 'Arm extended at face height'
    },
    'approach_table': {
        'joints': [1.57, 1.0, 1.0, -1.0, -1.0, 0.0],
        'gripper': 0.0,
        'description': 'Arm at table height'
    },
    'approach_ground': {
        'joints': [1.57, 1.5, 1.5, -1.5, -1.5, 0.0],
        'gripper': 0.0,
        'description': 'Arm low for floor objects'
    },
    'grasp_close': {
        'joints': [1.57, 1.0, 1.0, -1.0, -1.0, 3.14],  # Joint 6 = 3.14 (close)
        'gripper': 3.14,
        'description': 'Gripper closed'
    },
    'safe_carry': {
        'joints': [1.57, 0.785, 0.785, -0.785, -0.785, 3.14],
        'gripper': 3.14,
        'description': 'Carrying position'
    },
    'release': {
        'joints': [1.57, 0.785, 0.785, -0.785, -0.785, 0.0],
        'gripper': 0.0,
        'description': 'Gripper open to release object'
    }
}

# ==================== MOTION PARAMETERS ====================
MOTION_PARAMS = {
    'max_linear_velocity': 0.3,      # m/s
    'max_angular_velocity': 0.5,     # rad/s
    'min_distance_to_object': 0.5,   # meters (approach distance)
    'arm_move_duration': 1.0,        # seconds for smooth movement
    'grasp_wait_time': 0.5,          # seconds to let gripper settle
    'approach_speed': 0.1,           # m/s slow approach speed
}

# ==================== DETECTION SETTINGS ====================
DETECTION_SETTINGS = {
    'model': 'yolov8n',                    # YOLOv8 nano (fast)
    'confidence_threshold': 0.5,            # 50% confidence minimum
    'iou_threshold': 0.45,                  # IoU for NMS
    'detection_timeout': 10.0,              # seconds to look for object
    'enable_onnx_backend': True,            # Use ONNX for speed
    'camera_device': '/dev/video0',         # USB camera device
    'camera_width': 640,
    'camera_height': 480,
    'camera_fps': 30,
}

# ==================== NAVIGATION SETTINGS ====================
NAVIGATION_PARAMS = {
    'use_nav2': True,                       # Use Nav2 stack
    'max_planning_distance': 20.0,          # meters
    'approach_distance': 0.5,               # Stop this far from goal
    'navigation_timeout': 60.0,             # seconds max for navigation
    'use_lidar_mapping': True,              # Build maps dynamically
    'map_frame': 'map',
    'robot_frame': 'base_link',
}

# ==================== SAFETY LIMITS ====================
SAFETY_LIMITS = {
    'max_arm_reach': 0.8,                   # meters from base
    'min_table_height': 0.7,                # meters
    'max_table_height': 1.2,                # meters
    'gripper_max_force': 10.0,              # Newtons (estimate)
    'temperature_limit': 80.0,              # °C for motor
    'emergency_stop_enabled': True,
    'collision_detection': True,
}

# ==================== TASK SEQUENCES ====================
TASK_SEQUENCES = {
    'fetch_medicine': [
        'move_to_idle',
        'enable_detection',
        'detect_object:medicine',
        'navigate_to_object',
        'approach_for_grasp',
        'execute_grasp',
        'move_arm_safe_carry',
        'navigate_to_home',
        'release_object',
        'return_to_idle'
    ],
    'fetch_water': [
        'move_to_idle',
        'enable_detection',
        'detect_object:water',
        'navigate_to_object',
        'approach_for_grasp',
        'execute_grasp',
        'move_arm_safe_carry',
        'navigate_to_home',
        'release_object',
        'return_to_idle'
    ],
    'pick_up_object': [
        'move_to_idle',
        'enable_detection',
        'detect_object:any',
        'navigate_to_object',
        'approach_for_grasp',
        'execute_grasp',
        'move_arm_safe_carry',
        'navigate_to_home',
        'release_object',
        'return_to_idle'
    ]
}

# ==================== VOICE COMMAND MAPPINGS ====================
VOICE_COMMAND_MAP = {
    'give me medicine': 'fetch_medicine',
    'medicine': 'fetch_medicine',
    'fetch water': 'fetch_water',
    'water bottle': 'fetch_water',
    'pick up bottle': 'fetch_water',
    'bring me cup': 'fetch_cup',
    'get object': 'pick_up_object',
    'return home': 'return_home',
    'go home': 'return_home',
    'stop': 'stop_task',
    'cancel': 'stop_task',
    'emergency stop': 'emergency_stop'
}

# ==================== ROS2 CONFIGURATION ====================
ROS2_CONFIG = {
    'namespace': '/articubot',
    'voice_command_topic': '/robot/voice_command',
    'status_topic': '/robot/command_status',
    'detection_topic': '/perception/detections',
    'arm_trajectory_topic': '/arm_controller/joint_trajectory',
    'cmd_vel_topic': '/cmd_vel',
    'joint_state_topic': '/joint_states',
    'amcl_pose_topic': '/amcl_pose',
    'map_topic': '/map',
}

# ==================== HOME POSITION ====================
HOME_POSITION = {
    'x': 0.0,           # Robot's home X coordinate (meters)
    'y': 0.0,           # Robot's home Y coordinate (meters)
    'yaw': 0.0,         # Robot's home orientation (radians)
    'frame': 'map'
}

# ==================== LOGGING ====================
LOGGING_CONFIG = {
    'log_level': 'INFO',                    # DEBUG, INFO, WARNING, ERROR
    'log_to_file': True,
    'log_file': '/tmp/autonomous_fetch.log',
    'log_rostime': True,
}

# ==================== PERFORMANCE SETTINGS ====================
PERFORMANCE = {
    'object_detection_rate': 10.0,          # Hz
    'arm_control_rate': 20.0,               # Hz
    'navigation_check_rate': 2.0,           # Hz
    'status_report_rate': 1.0,              # Hz
}

# ==================== ERROR RECOVERY ====================
ERROR_RECOVERY = {
    'max_retries': 3,
    'retry_delay': 2.0,                     # seconds
    'fallback_to_manual': True,             # Allow manual takeover after failures
    'auto_home_on_error': True,             # Return to home on error
}

# ==================== ADVANCED FEATURES ====================
FEATURES = {
    'enable_gesture_control': False,        # Hand gesture recognition
    'enable_gps_navigation': False,         # GPS-based waypoints
    'enable_multi_robot': False,            # Swarm control
    'enable_learning': False,               # ML-based improvement
    'enable_user_feedback': True,           # Learn from corrections
}


# ==================== HELPER FUNCTIONS ====================

def get_arm_position(position_name):
    """Get arm joint angles for a named position."""
    if position_name in ARM_POSITIONS:
        return ARM_POSITIONS[position_name]['joints']
    else:
        raise ValueError(f"Unknown arm position: {position_name}")


def get_object_labels(object_type):
    """Get all label variations for an object type."""
    if object_type in FETCHABLE_OBJECTS:
        return FETCHABLE_OBJECTS[object_type]['labels']
    return []


def get_task_sequence(task_name):
    """Get the sequence of steps for a task."""
    if task_name in TASK_SEQUENCES:
        return TASK_SEQUENCES[task_name]
    return None


def get_command_task(voice_command):
    """Map voice command to task name."""
    voice_command_lower = voice_command.lower().strip()
    
    for pattern, task in VOICE_COMMAND_MAP.items():
        if pattern.lower() in voice_command_lower:
            return task
    
    return None


if __name__ == '__main__':
    # Test configuration
    print("Fetchable objects:", list(FETCHABLE_OBJECTS.keys()))
    print("Arm positions:", list(ARM_POSITIONS.keys()))
    print("Home position:", HOME_POSITION)
    print("Motion limits:", MOTION_PARAMS)
