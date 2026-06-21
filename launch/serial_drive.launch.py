from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    port = LaunchConfiguration('port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    baud = LaunchConfiguration('baud', default='115200')
    use_stamped = LaunchConfiguration('use_stamped', default='true')
    wheel_separation = LaunchConfiguration('wheel_separation', default='0.236')
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0325')
    encoder_cpr = LaunchConfiguration('encoder_cpr', default='330')
    odom_linear_scale = LaunchConfiguration('odom_linear_scale', default='1.0')
    odom_angular_scale = LaunchConfiguration('odom_angular_scale', default='0.735')
    max_linear = LaunchConfiguration('max_linear', default='0.16')
    max_angular = LaunchConfiguration('max_angular', default='0.60')
    max_pwm = LaunchConfiguration('max_pwm', default='255')
    min_nonzero_pwm = LaunchConfiguration('min_nonzero_pwm', default='115')
    min_turn_pwm = LaunchConfiguration('min_turn_pwm', default='115')
    reverse_pwm_scale = LaunchConfiguration('reverse_pwm_scale', default='1.0')
    left_forward_pwm_scale = LaunchConfiguration('left_forward_pwm_scale', default='1.0')
    left_reverse_pwm_scale = LaunchConfiguration('left_reverse_pwm_scale', default='1.0')
    right_forward_pwm_scale = LaunchConfiguration('right_forward_pwm_scale', default='1.0')
    right_reverse_pwm_scale = LaunchConfiguration('right_reverse_pwm_scale', default='1.0')
    turn_pwm_scale = LaunchConfiguration('turn_pwm_scale', default='0.65')
    turn_in_place_threshold = LaunchConfiguration('turn_in_place_threshold', default='0.05')
    turn_assist_cmd_w_threshold = LaunchConfiguration('turn_assist_cmd_w_threshold', default='0.0')
    turn_assist_min_pwm_delta = LaunchConfiguration('turn_assist_min_pwm_delta', default='0')
    straight_trim_pwm = LaunchConfiguration('straight_trim_pwm', default='8')
    straight_trim_min_cmd_vel = LaunchConfiguration('straight_trim_min_cmd_vel', default='0.05')
    straight_trim_max_cmd_w = LaunchConfiguration('straight_trim_max_cmd_w', default='0.15')
    timeout_sec = LaunchConfiguration('timeout_sec', default='0.5')
    serial_boot_wait_sec = LaunchConfiguration('serial_boot_wait_sec', default='2.0')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/diff_cont/cmd_vel')
    debug_serial = LaunchConfiguration('debug_serial', default='true')

    direct_cmd_vel = LaunchConfiguration('direct_cmd_vel', default='false')

    twist_mux = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'twist_mux.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(direct_cmd_vel),
    )

    keyboard_bridge_node = Node(
        package='articubot_one',
        executable='keyboard_bridge.py',
        name='keyboard_bridge',
        output='screen',
        condition=UnlessCondition(direct_cmd_vel),
    )

    serial_node_mux = Node(
        package='articubot_one',
        executable='serial_diffdrive_node.py',
        name='serial_diffdrive_node',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud,
            'cmd_vel_topic': cmd_vel_topic,
            'use_stamped': use_stamped,
            'wheel_separation': wheel_separation,
            'wheel_radius': wheel_radius,
            'encoder_cpr': encoder_cpr,
            'odom_linear_scale': odom_linear_scale,
            'odom_angular_scale': odom_angular_scale,
            'max_linear': max_linear,
            'max_angular': max_angular,
            'max_pwm': max_pwm,
            'min_nonzero_pwm': min_nonzero_pwm,
            'min_turn_pwm': min_turn_pwm,
            'reverse_pwm_scale': reverse_pwm_scale,
            'left_forward_pwm_scale': left_forward_pwm_scale,
            'left_reverse_pwm_scale': left_reverse_pwm_scale,
            'right_forward_pwm_scale': right_forward_pwm_scale,
            'right_reverse_pwm_scale': right_reverse_pwm_scale,
            'turn_pwm_scale': turn_pwm_scale,
            'turn_in_place_threshold': turn_in_place_threshold,
            'turn_assist_cmd_w_threshold': turn_assist_cmd_w_threshold,
            'turn_assist_min_pwm_delta': turn_assist_min_pwm_delta,
            'straight_trim_pwm': straight_trim_pwm,
            'straight_trim_min_cmd_vel': straight_trim_min_cmd_vel,
            'straight_trim_max_cmd_w': straight_trim_max_cmd_w,
            'timeout_sec': timeout_sec,
            'serial_boot_wait_sec': serial_boot_wait_sec,
            'enable_fallback_cmd_vel': True,
            'debug': debug_serial,
        }],
        condition=UnlessCondition(direct_cmd_vel),
    )

    serial_node_direct = Node(
        package='articubot_one',
        executable='serial_diffdrive_node.py',
        name='serial_diffdrive_node',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud,
            'cmd_vel_topic': '/cmd_vel',
            'use_stamped': False,
            'wheel_separation': wheel_separation,
            'wheel_radius': wheel_radius,
            'encoder_cpr': encoder_cpr,
            'odom_linear_scale': odom_linear_scale,
            'odom_angular_scale': odom_angular_scale,
            'max_linear': max_linear,
            'max_angular': max_angular,
            'max_pwm': max_pwm,
            'min_nonzero_pwm': min_nonzero_pwm,
            'min_turn_pwm': min_turn_pwm,
            'reverse_pwm_scale': reverse_pwm_scale,
            'left_forward_pwm_scale': left_forward_pwm_scale,
            'left_reverse_pwm_scale': left_reverse_pwm_scale,
            'right_forward_pwm_scale': right_forward_pwm_scale,
            'right_reverse_pwm_scale': right_reverse_pwm_scale,
            'turn_pwm_scale': turn_pwm_scale,
            'turn_in_place_threshold': turn_in_place_threshold,
            'turn_assist_cmd_w_threshold': turn_assist_cmd_w_threshold,
            'turn_assist_min_pwm_delta': turn_assist_min_pwm_delta,
            'straight_trim_pwm': straight_trim_pwm,
            'straight_trim_min_cmd_vel': straight_trim_min_cmd_vel,
            'straight_trim_max_cmd_w': straight_trim_max_cmd_w,
            'timeout_sec': timeout_sec,
            'serial_boot_wait_sec': serial_boot_wait_sec,
            'enable_fallback_cmd_vel': False,
            'debug': debug_serial,
        }],
        condition=IfCondition(direct_cmd_vel),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('use_stamped', default_value='true'),
        DeclareLaunchArgument('direct_cmd_vel', default_value='false'),
        DeclareLaunchArgument('wheel_separation', default_value='0.236'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0325'),
        DeclareLaunchArgument('encoder_cpr', default_value='330'),
        DeclareLaunchArgument('odom_linear_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_angular_scale', default_value='0.735'),
        DeclareLaunchArgument('max_linear', default_value='0.16'),
        DeclareLaunchArgument('max_angular', default_value='0.60'),
        DeclareLaunchArgument('max_pwm', default_value='255'),
        DeclareLaunchArgument('min_nonzero_pwm', default_value='115'),
        DeclareLaunchArgument('min_turn_pwm', default_value='115'),
        DeclareLaunchArgument('reverse_pwm_scale', default_value='1.0'),
        DeclareLaunchArgument('left_forward_pwm_scale', default_value='1.0'),
        DeclareLaunchArgument('left_reverse_pwm_scale', default_value='1.0'),
        DeclareLaunchArgument('right_forward_pwm_scale', default_value='1.0'),
        DeclareLaunchArgument('right_reverse_pwm_scale', default_value='1.0'),
        DeclareLaunchArgument('turn_pwm_scale', default_value='0.65'),
        DeclareLaunchArgument('turn_in_place_threshold', default_value='0.05'),
        DeclareLaunchArgument('turn_assist_cmd_w_threshold', default_value='0.0'),
        DeclareLaunchArgument('turn_assist_min_pwm_delta', default_value='0'),
        DeclareLaunchArgument('straight_trim_pwm', default_value='8'),
        DeclareLaunchArgument('straight_trim_min_cmd_vel', default_value='0.05'),
        DeclareLaunchArgument('straight_trim_max_cmd_w', default_value='0.15'),
        DeclareLaunchArgument('timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('serial_boot_wait_sec', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/diff_cont/cmd_vel'),
        DeclareLaunchArgument('debug_serial', default_value='true'),

        LogInfo(msg=['============ STARTING SERIAL DIFF DRIVE (MEGA ADK) ============']),
        twist_mux,
        keyboard_bridge_node,
        serial_node_mux,
        serial_node_direct,
    ])
