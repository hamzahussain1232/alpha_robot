from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():

    package_name='articubot_one'

    # ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Params file can be overridden by parent launch (sim vs hardware profiles).
    ekf_params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'ekf.yaml'])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'ekf.yaml']),
            description='Path to EKF params YAML'
        ),

        LogInfo(msg=['============ STARTING EKF FILTER ============']),
        LogInfo(msg=['Loading params from: ', ekf_params_file]),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
            # Remap filtered output to local odometry topic
            remappings=[("odometry/filtered", "odometry/local")],
        )
    ])
