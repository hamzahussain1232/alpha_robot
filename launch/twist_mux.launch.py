from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'articubot_one'

    # Accept namespace from parent launch or use empty default
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to config file
    twist_mux_params = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'twist_mux.yaml'])

    # TWIST MUX NODE
    twist_mux_node = Node(
        package="twist_mux",
        namespace=namespace,
        executable="twist_mux",
        output='screen',
        # CRITICAL FIX: Changed 'False' to 'True' below
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time, 'use_stamped': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true', # Changed default to true for safety
            description='Use sim time if true'),

        LogInfo(msg=['============ STARTING TWIST_MUX (STAMPED MODE) ============']),

        twist_mux_node,
    ])
