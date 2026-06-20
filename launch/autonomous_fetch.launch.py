"""
Launch autonomous task executor system.

This brings together:
- Voice command receiver (orchestrator)
- Object detection
- Navigation stack
- Arm control

Usage:
    ros2 launch articubot_one autonomous_fetch.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'
    
    # Arguments
    start_voice_server = DeclareLaunchArgument(
        'start_voice_server',
        default_value='true',
        description='Whether to start the Flask voice server'
    )
    
    enable_nav2 = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Enable Nav2 navigation stack'
    )
    
    enable_perception = DeclareLaunchArgument(
        'enable_perception',
        default_value='true',
        description='Enable object detection'
    )
    
    # Include real hardware drivers
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'drive_real.launch.py'])
        ),
        launch_arguments={'enable_teleop': 'false'}.items()  # Disable teleop for autonomous mode
    )
    
    # Include arm
    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'arm_real.launch.py'])
        )
    )
    
    # Include perception (object detection)
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'perception.launch.py'])
        ),
        condition=LaunchConfiguration('enable_perception')
    )
    
    # Include navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=LaunchConfiguration('enable_nav2')
    )
    
    # Voice command receiver node
    voice_command_receiver = Node(
        package=package_name,
        executable='voice_command_receiver_node',
        name='voice_command_receiver',
        output='screen',
        parameters=[{
            'home_pose_x': 0.0,
            'home_pose_y': 0.0,
            'home_pose_yaw': 0.0,
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        DeclareLaunchArgument('enable_perception', default_value='true'),
        DeclareLaunchArgument('start_voice_server', default_value='true'),
        
        LogInfo(msg="========== STARTING AUTONOMOUS FETCH SYSTEM =========="),
        LogInfo(msg="This system allows voice commands via mobile app"),
        LogInfo(msg="Commands: 'Give me medicine', 'Fetch water', 'Pick up bottle', etc."),
        LogInfo(msg="======================================================="),
        
        hardware,
        arm,
        perception,
        navigation,
        voice_command_receiver,
    ])
