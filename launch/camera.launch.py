import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Allow changing the device from command line:
    # ros2 launch articubot_one camera.launch.py video_device:=/dev/video2
    video_device_arg = DeclareLaunchArgument(
        'video_device', 
        default_value='/dev/video0',
        description='Path to video device (e.g. /dev/video0, /dev/video2)'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('articubot_one'), 'config', 'camera_v4l2_params.yaml']
        ),
        description='Path to v4l2 camera params yaml'
    )

    return LaunchDescription([
        video_device_arg,
        params_file_arg,

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            # CRITICAL: Remap the topic to match the Simulation and Ball Tracker
            remappings=[
                ('/image_raw', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
            ],
            parameters=[
                LaunchConfiguration('params_file'),
                {'video_device': LaunchConfiguration('video_device')},
            ]
        )
    ])
