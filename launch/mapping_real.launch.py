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
    include_arm = LaunchConfiguration('include_arm', default='true')

    lidar_port = LaunchConfiguration('lidar_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    lidar_baud = LaunchConfiguration('lidar_baud', default='115200')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser_frame')
    lidar_offset_x = LaunchConfiguration('lidar_offset_x', default='-0.0885')
    lidar_offset_y = LaunchConfiguration('lidar_offset_y', default='0.0')
    lidar_offset_z = LaunchConfiguration('lidar_offset_z', default='0.0275')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='3.14159')

    enable_teleop = LaunchConfiguration('enable_teleop', default='true')

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

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers', 'slam_toolbox.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
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
        DeclareLaunchArgument('include_arm', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        DeclareLaunchArgument('lidar_offset_x', default_value='-0.0885'),
        DeclareLaunchArgument('lidar_offset_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_offset_z', default_value='0.0275'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159'),
        DeclareLaunchArgument('enable_teleop', default_value='true'),

        LogInfo(msg=['============ STARTING REAL MAPPING (SLAM) ============']),
        rsp,
        serial_drive,
        lidar,
        slam_toolbox,
        wasd_teleop,
    ])
