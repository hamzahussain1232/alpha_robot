from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    serial_baud = LaunchConfiguration('serial_baud', default='115200')
    wheel_separation = LaunchConfiguration('wheel_separation', default='0.236')
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0325')
    encoder_cpr = LaunchConfiguration('encoder_cpr', default='330')
    max_angular = LaunchConfiguration('max_angular', default='1.2')
    min_turn_pwm = LaunchConfiguration('min_turn_pwm', default='180')
    turn_pwm_scale = LaunchConfiguration('turn_pwm_scale', default='1.35')
    turn_in_place_threshold = LaunchConfiguration('turn_in_place_threshold', default='0.08')
    turn_assist_cmd_w_threshold = LaunchConfiguration('turn_assist_cmd_w_threshold', default='0.05')
    turn_assist_min_pwm_delta = LaunchConfiguration('turn_assist_min_pwm_delta', default='80')
    include_arm = LaunchConfiguration('include_arm', default='true')

    lidar_port = LaunchConfiguration('lidar_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    lidar_baud = LaunchConfiguration('lidar_baud', default='115200')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser_frame')
    lidar_offset_x = LaunchConfiguration('lidar_offset_x', default='-0.0885')
    lidar_offset_y = LaunchConfiguration('lidar_offset_y', default='0.0')
    lidar_offset_z = LaunchConfiguration('lidar_offset_z', default='0.0275')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='3.14159')

    map_file = LaunchConfiguration('map', default='')
    enable_teleop = LaunchConfiguration('enable_teleop', default='false')

    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'nav2_params.yaml']
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'include_arm': include_arm,
            'lidar_offset_x': lidar_offset_x,
            'lidar_offset_y': lidar_offset_y,
            'lidar_offset_z': lidar_offset_z,
            'lidar_yaw': lidar_yaw,
        }.items(),
    )

    serial_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'serial_drive.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'port': serial_port,
            'baud': serial_baud,
            'wheel_separation': wheel_separation,
            'wheel_radius': wheel_radius,
            'encoder_cpr': encoder_cpr,
            'max_angular': max_angular,
            'min_turn_pwm': min_turn_pwm,
            'turn_pwm_scale': turn_pwm_scale,
            'turn_in_place_threshold': turn_in_place_threshold,
            'turn_assist_cmd_w_threshold': turn_assist_cmd_w_threshold,
            'turn_assist_min_pwm_delta': turn_assist_min_pwm_delta,
        }.items(),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'])
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baud,
            'frame_id': lidar_frame,
        }.items(),
    )

    amcl_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localizer_type': 'amcl',
            'map': map_file,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'odom_topic': '/odom',
            'autostart': 'true',
        }.items(),
    )

    nav_cmd_stamper = Node(
        package=package_name,
        executable='twist_stamper.py',
        name='nav_cmd_stamper',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel_nav_unstamped',
            'output_topic': '/cmd_vel_nav',
            'frame_id': 'base_link',
        }],
    )

    wasd_teleop = Node(
        package='articubot_one',
        executable='wasd_teleop.py',
        name='wasd_teleop',
        output='screen',
        parameters=[{
            'cmd_topic': '/cmd_vel',
            'use_stamped': False,
            'linear_speed': 0.15,
            'angular_speed': 1.7,
            'publish_rate_hz': 20.0,
            'stop_timeout': 0.3,
        }],
        condition=IfCondition(enable_teleop),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('wheel_separation', default_value='0.236'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0325'),
        DeclareLaunchArgument('encoder_cpr', default_value='330'),
        DeclareLaunchArgument('max_angular', default_value='1.2'),
        DeclareLaunchArgument('min_turn_pwm', default_value='180'),
        DeclareLaunchArgument('turn_pwm_scale', default_value='1.35'),
        DeclareLaunchArgument('turn_in_place_threshold', default_value='0.08'),
        DeclareLaunchArgument('turn_assist_cmd_w_threshold', default_value='0.05'),
        DeclareLaunchArgument('turn_assist_min_pwm_delta', default_value='80'),
        DeclareLaunchArgument('include_arm', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        DeclareLaunchArgument('lidar_offset_x', default_value='-0.0885'),
        DeclareLaunchArgument('lidar_offset_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_offset_z', default_value='0.0275'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('enable_teleop', default_value='false'),

        LogInfo(msg=['============ STARTING REAL NAVIGATION (AMCL + NAV2) ============']),
        LogInfo(msg=['Map file: ', map_file]),
        rsp,
        serial_drive,
        lidar,
        amcl_localizer,
        navigation,
        nav_cmd_stamper,
        wasd_teleop,
    ])
