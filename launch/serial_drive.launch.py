from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    port = LaunchConfiguration('port', default='/dev/ttyACM0')
    baud = LaunchConfiguration('baud', default='115200')
    wheel_separation = LaunchConfiguration('wheel_separation', default='0.31')
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0425')
    max_linear = LaunchConfiguration('max_linear', default='0.5')
    max_angular = LaunchConfiguration('max_angular', default='1.5')
    max_pwm = LaunchConfiguration('max_pwm', default='255')
    timeout_sec = LaunchConfiguration('timeout_sec', default='0.5')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/diff_cont/cmd_vel')

    twist_mux = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'twist_mux.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    keyboard_bridge_node = Node(
        package='articubot_one',
        executable='keyboard_bridge.py',
        name='keyboard_bridge',
        output='screen'
    )

    serial_node = Node(
        package='articubot_one',
        executable='serial_diffdrive_node.py',
        name='serial_diffdrive_node',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud,
            'cmd_vel_topic': cmd_vel_topic,
            'use_stamped': True,
            'wheel_separation': wheel_separation,
            'wheel_radius': wheel_radius,
            'max_linear': max_linear,
            'max_angular': max_angular,
            'max_pwm': max_pwm,
            'timeout_sec': timeout_sec,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('wheel_separation', default_value='0.31'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0425'),
        DeclareLaunchArgument('max_linear', default_value='0.5'),
        DeclareLaunchArgument('max_angular', default_value='1.5'),
        DeclareLaunchArgument('max_pwm', default_value='255'),
        DeclareLaunchArgument('timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/diff_cont/cmd_vel'),

        LogInfo(msg=['============ STARTING SERIAL DIFF DRIVE (MEGA ADK) ============']),
        twist_mux,
        keyboard_bridge_node,
        serial_node,
    ])
