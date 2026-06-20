import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Use explicit paths for the virtual environment and scripts
    workspace_dir = '/home/hh89669411/ros2_ws'
    venv_python = os.path.join(workspace_dir, '.venv_voice', 'bin', 'python3')
    voice_server_script = os.path.join(workspace_dir, 'src', 'articubot_one', 'scripts', 'omni_voice_server.py')

    return LaunchDescription([
        # 1. Start the Secure Voice Web Server inside its venv
        ExecuteProcess(
            cmd=[venv_python, voice_server_script],
            cwd=workspace_dir,
            output='screen'
        ),
        
        # 2. Start the ROS 2 Command Parser
        Node(
            package='articubot_one',
            executable='omni_robot_commander.py',
            name='omni_robot_commander',
            output='screen'
        )
    ])
