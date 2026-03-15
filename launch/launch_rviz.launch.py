import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    package_name = 'articubot_one'

    # ARGS
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # If true, launches the GUI sliders to move joints manually. 
    # Great for testing URDF, but turn off for Simulation/Real Robot.
    show_gui = LaunchConfiguration('show_gui', default='false') 
    
    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'map.rviz'])
    )

    # 1. RVIZ NODE
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. JOINT STATE PUBLISHER GUI
    # Only runs if 'show_gui' is true. 
    # Allows you to manually inspect the robot model.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(show_gui),
        output='screen'
    )

    # NOTE: We REMOVED 'rsp.launch.py' and 'joystick.launch.py' from here.
    # Why? Because drive_sim.launch.py and launch_robot.launch.py ALREADY run them.
    # Including them here would cause "Double Node" errors.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'show_gui',
            default_value='false',
            description='Show Joint State Publisher GUI to move arm manually'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'map.rviz']),
            description='Path to RViz config file'),

        LogInfo(msg=['============ STARTING RVIZ VISUALIZATION ============']),

        joint_state_publisher_gui,
        rviz_node,
    ])
