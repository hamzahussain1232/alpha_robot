import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'articubot_one'

    # ARGS
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. PROCESS THE URDF (Robot Description)
    # This converts the Xacro file into a raw XML format that the robot can understand
    xacro_file = PathJoinSubstitution([FindPackageShare(package_name), 'description', 'robot.urdf.xacro'])
    
    # We pass 'sim_mode:=false' because this is the real robot launch file
    robot_description_content = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # 2. CONTROLLER CONFIGURATION
    controllers_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'my_controllers_hardware.yaml'
    ])

    # 3. LAUNCH RSP (Robot State Publisher)
    # This node publishes the static transforms (TF) for the robot
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'robot_description': robot_description_content}.items()
    )

    # 4. TWIST MUX (Merges Joystick, Keyboard, and Nav2 commands)
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'twist_mux.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. KEYBOARD BRIDGE (Allows laptop keyboard to drive the robot)
    keyboard_bridge_node = Node(
        package='articubot_one',
        executable='keyboard_bridge.py',
        name='keyboard_bridge',
        output='screen'
    )

    # 6. CONTROLLER MANAGER (The Brains of the Operation)
    # This connects to the hardware (Arduino) and manages the controllers
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_params_file],
        output="screen",
        remappings=[
            # Remap standard /robot_description so the node can find it
            ("~/robot_description", "/robot_description")
        ]
    )

    # 7. SPAWNERS (Start the specific controllers)
    
    # Joint State Broadcaster (Publishes the state of all joints to TF)
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )

    # Diff Drive Controller (Handles the wheel velocity)
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen"
    )

    # 8. DELAYED STARTUP SEQUENCE
    # We delay the controller manager slightly to ensure RSP is up, 
    # then we chain the spawners to start AFTER the manager is active.

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[diff_drive_spawner],
        )
    )

    # RETURN LAUNCH DESCRIPTION
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        LogInfo(msg=['============ STARTING REAL ROBOT HARDWARE ============']),
        
        rsp,
        twist_mux,
        keyboard_bridge_node,
        
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner
    ])
