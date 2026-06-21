from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the secure voice web server from the installed package executable.
        Node(
            package='articubot_one',
            executable='omni_voice_server.py',
            name='omni_voice_server',
            output='screen',
        ),
        # Start the ROS 2 command parser.
        Node(
            package='articubot_one',
            executable='omni_robot_commander.py',
            name='omni_robot_commander',
            output='screen'
        )
    ])
