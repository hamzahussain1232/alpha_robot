from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EqualsSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mode = LaunchConfiguration('mode', default='mapping')
    mapping_localizer_type = LaunchConfiguration('mapping_localizer_type', default='slam_toolbox')
    hardware_mode = LaunchConfiguration('hardware_mode', default='real')
    map_file = LaunchConfiguration('map', default='')
    world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), 'assets', 'worlds', 'home_assistant_room.sdf']
        ),
    )
    robot_model = LaunchConfiguration('robot_model', default='minibot')
    workflow_use_rviz = LaunchConfiguration('workflow_use_rviz', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default='')
    use_gz_gui = LaunchConfiguration('use_gz_gui', default='true')
    gz_verbosity = LaunchConfiguration('gz_verbosity', default='1')
    lidar_offset_x = LaunchConfiguration('lidar_offset_x', default='-0.0885')
    lidar_offset_y = LaunchConfiguration('lidar_offset_y', default='0.0')
    lidar_offset_z = LaunchConfiguration('lidar_offset_z', default='0.0275')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='3.14159')
    cleanup_on_start = LaunchConfiguration('cleanup_on_start', default='true')
    ros_localhost_only = LaunchConfiguration('ros_localhost_only', default='0')
    ros_discovery_range = LaunchConfiguration('ros_discovery_range', default='SUBNET')
    ros_domain_id = LaunchConfiguration('ros_domain_id', default='0')
    restart_ros_daemon = LaunchConfiguration('restart_ros_daemon', default='false')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    stack_start_delay = LaunchConfiguration('stack_start_delay', default='12.0')
    enable_voice = LaunchConfiguration('enable_voice', default='false')
    voice_use_mic = LaunchConfiguration('voice_use_mic', default='false')
    enable_phone_text = LaunchConfiguration('enable_phone_text', default='true')
    phone_text_host = LaunchConfiguration('phone_text_host', default='0.0.0.0')
    phone_text_port = LaunchConfiguration('phone_text_port', default='5000')
    voice_params = LaunchConfiguration(
        'voice_params',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'voice_params.yaml']),
    )
    mapping_slam_params_default = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'mapper_params_sim_stable.yaml']
    )
    mapping_slam_params = LaunchConfiguration(
        'mapping_slam_params',
        default=mapping_slam_params_default,
    )
    enable_perception = LaunchConfiguration('enable_perception', default='false')
    enable_detector = LaunchConfiguration('enable_detector', default='false')
    enable_marker_detector = LaunchConfiguration('enable_marker_detector', default='false')
    enable_camera_safety = LaunchConfiguration('enable_camera_safety', default='true')
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    enable_arm_driver = LaunchConfiguration('enable_arm_driver', default='false')
    camera_driver = LaunchConfiguration('camera_driver', default='libcamera')
    camera_video_device = LaunchConfiguration('camera_video_device', default='/dev/video0')
    camera_params = LaunchConfiguration(
        'camera_params',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'camera_libcamera_params.yaml']),
    )
    enable_task_manager = LaunchConfiguration('enable_task_manager', default='false')
    spawn_arm_controller = LaunchConfiguration('spawn_arm_controller', default='false')
    include_arm = LaunchConfiguration('include_arm', default='true')
    serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    serial_baud = LaunchConfiguration('serial_baud', default='115200')
    arm_port = LaunchConfiguration(
        'arm_port',
        default='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
    )
    arm_baud = LaunchConfiguration('arm_baud', default='9600')
    arm_serial_boot_wait_sec = LaunchConfiguration('arm_serial_boot_wait_sec', default='2.0')
    wheel_separation = LaunchConfiguration('wheel_separation', default='0.236')
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0325')
    encoder_cpr = LaunchConfiguration('encoder_cpr', default='330')
    nav_max_angular = LaunchConfiguration('nav_max_angular', default='0.60')
    nav_min_turn_pwm = LaunchConfiguration('nav_min_turn_pwm', default='115')
    nav_turn_pwm_scale = LaunchConfiguration('nav_turn_pwm_scale', default='0.65')
    nav_turn_in_place_threshold = LaunchConfiguration('nav_turn_in_place_threshold', default='0.08')
    nav_turn_assist_cmd_w_threshold = LaunchConfiguration(
        'nav_turn_assist_cmd_w_threshold', default='0.10'
    )
    nav_turn_assist_min_pwm_delta = LaunchConfiguration(
        'nav_turn_assist_min_pwm_delta', default='0'
    )
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    lidar_baud = LaunchConfiguration('lidar_baud', default='115200')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser_frame')
    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    enable_teleop = LaunchConfiguration('enable_teleop', default='true')
    perception_params_sim = LaunchConfiguration(
        'perception_params_sim',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'perception_params_sim.yaml']),
    )
    perception_params_real = LaunchConfiguration(
        'perception_params_real',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'perception_params_real.yaml']),
    )
    task_params_sim = LaunchConfiguration(
        'task_params_sim',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'task_params_sim.yaml']),
    )
    task_params_real = LaunchConfiguration(
        'task_params_real',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'task_params_real.yaml']),
    )
    detector_params_sim = LaunchConfiguration(
        'detector_params_sim',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'detector_params_sim.yaml']),
    )
    detector_params_real = LaunchConfiguration(
        'detector_params_real',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'detector_params_real.yaml']),
    )
    marker_params_sim = LaunchConfiguration(
        'marker_params_sim',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'marker_params_sim.yaml']),
    )
    marker_params_real = LaunchConfiguration(
        'marker_params_real',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'marker_params_real.yaml']),
    )
    camera_safety_params_sim = LaunchConfiguration(
        'camera_safety_params_sim',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'camera_safety_params_sim.yaml']),
    )
    camera_safety_params_real = LaunchConfiguration(
        'camera_safety_params_real',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'camera_safety_params_real.yaml']),
    )
    object_memory_sim = LaunchConfiguration(
        'object_memory_sim',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'assets', 'memory', 'object_memory_sim.json']),
    )
    object_memory_real = LaunchConfiguration(
        'object_memory_real',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'assets', 'memory', 'object_memory_real_template.json']),
    )

    ros_local_only = SetEnvironmentVariable(
        name='ROS_LOCALHOST_ONLY',
        value=ros_localhost_only,
    )

    ros_discovery_local = SetEnvironmentVariable(
        name='ROS_AUTOMATIC_DISCOVERY_RANGE',
        value=ros_discovery_range,
    )

    ros_domain = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=ros_domain_id,
    )

    ros_static_peers_unset = UnsetEnvironmentVariable(
        name='ROS_STATIC_PEERS',
    )

    ros_daemon_stop = ExecuteProcess(
        cmd=['ros2', 'daemon', 'stop'],
        output='screen',
        condition=IfCondition(restart_ros_daemon),
    )

    ros_daemon_start = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'daemon', 'start'],
                output='screen',
            )
        ],
        condition=IfCondition(restart_ros_daemon),
    )

    # Base simulation stack (Gazebo + robot + bridge + controllers + EKF + keyboard)
    # RViz is disabled here and launched once below with selected config.
    sim_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'drive_sim.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'world': world,
            'use_rviz': 'false',
            'use_gz_gui': use_gz_gui,
            'gz_verbosity': gz_verbosity,
            'spawn_arm_controller': spawn_arm_controller,
            'lidar_offset_x': lidar_offset_x,
            'lidar_offset_y': lidar_offset_y,
            'lidar_offset_z': lidar_offset_z,
            'lidar_yaw': lidar_yaw,
            'cleanup_on_start': cleanup_on_start,
            'ros_localhost_only': ros_localhost_only,
            'ros_domain_id': ros_domain_id,
        }.items(),
        condition=IfCondition(EqualsSubstitution(hardware_mode, 'sim')),
    )

    # Real robot stack (serial drive + RSP)
    real_rsp = IncludeLaunchDescription(
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
        condition=IfCondition(EqualsSubstitution(hardware_mode, 'real')),
    )

    real_serial_drive = IncludeLaunchDescription(
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
            'max_angular': PythonExpression(
                [
                    "'",
                    nav_max_angular,
                    "' if '",
                    mode,
                    "' == 'navigation' else '1.70'",
                ]
            ),
            'min_turn_pwm': PythonExpression(
                [
                    "'",
                    nav_min_turn_pwm,
                    "' if '",
                    mode,
                    "' == 'navigation' else '115'",
                ]
            ),
            'turn_pwm_scale': PythonExpression(
                [
                    "'",
                    nav_turn_pwm_scale,
                    "' if '",
                    mode,
                    "' == 'navigation' else '1.0'",
                ]
            ),
            'turn_in_place_threshold': PythonExpression(
                [
                    "'",
                    nav_turn_in_place_threshold,
                    "' if '",
                    mode,
                    "' == 'navigation' else '0.05'",
                ]
            ),
            'turn_assist_cmd_w_threshold': PythonExpression(
                [
                    "'",
                    nav_turn_assist_cmd_w_threshold,
                    "' if '",
                    mode,
                    "' == 'navigation' else '0.0'",
                ]
            ),
            'turn_assist_min_pwm_delta': PythonExpression(
                [
                    "'",
                    nav_turn_assist_min_pwm_delta,
                    "' if '",
                    mode,
                    "' == 'navigation' else '0'",
                ]
            ),
        }.items(),
        condition=IfCondition(EqualsSubstitution(hardware_mode, 'real')),
    )

    real_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'])
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baud,
            'frame_id': lidar_frame,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_lidar,
                    "' == 'true' and '",
                    hardware_mode,
                    "' == 'real' and '",
                    mode,
                    "' != 'drive'",
                ]
            )
        ),
    )

    real_teleop = Node(
        package='articubot_one',
        executable='wasd_teleop.py',
        name='wasd_teleop',
        output='screen',
        parameters=[{
            'cmd_topic': '/cmd_vel',
            'use_stamped': False,
            'linear_speed': 0.10,
            'angular_speed': 1.70,
            'publish_rate_hz': 20.0,
            'stop_timeout': 0.3,
        }],
        condition=IfCondition(
            PythonExpression(
                ["'", enable_teleop, "' == 'true' and '", hardware_mode, "' == 'real'"]
            )
        ),
    )

    real_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'camera.launch.py'])
        ),
        launch_arguments={
            'driver': camera_driver,
            'video_device': camera_video_device,
            'params_file': camera_params,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", enable_camera, "' == 'true' and '", hardware_mode, "' == 'real'"]
            )
        ),
    )

    real_arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'arm_nano.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_rsp': 'false',
            'port': arm_port,
            'baud': arm_baud,
            'serial_boot_wait_sec': arm_serial_boot_wait_sec,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_arm_driver,
                    "' == 'true' and '",
                    include_arm,
                    "' == 'true' and '",
                    hardware_mode,
                    "' == 'real'",
                ]
            )
        ),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'launch_rviz.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'mode': mode,
            'show_gui': 'false',
            'rviz_config': rviz_config,
        }.items(),
        condition=IfCondition(workflow_use_rviz),
    )

    mapping_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'localizer_type': mapping_localizer_type,
            'slam_params_file': mapping_slam_params,
            'map': '',
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'mapping')),
    )

    stable_scan_cloud = Node(
        package=package_name,
        executable='stable_scan_cloud.py',
        name='stable_scan_cloud',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'scan_topic': '/scan',
                'cloud_topic': '/stable_scan_cloud',
                'target_frame': 'map',
                'max_age_sec': 0.0,
                'stride': 1,
                'max_range': 5.95,
                'ignore_range_margin': 0.05,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", hardware_mode, "' == 'sim' and '", mode, "' == 'mapping'"]
            )
        ),
    )

    amcl_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'localizer_type': 'amcl',
            'map': map_file,
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'amcl')),
    )

    nav_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'localizer_type': 'amcl',
            'map': map_file,
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'navigation')),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic,
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'navigation')),
    )

    delayed_localizers = TimerAction(
        period=stack_start_delay,
        actions=[
            mapping_localizer,
            amcl_localizer,
            nav_localizer,
        ],
    )

    delayed_navigation = TimerAction(
        period=PythonExpression([stack_start_delay, ' + 5.0']),
        actions=[navigation],
    )

    # I have disabled the OLD heavy Vosk voice_control node so it doesn't thermal throttle the Pi 5
    # The new lightweight omni_voice processes at the bottom of this file will handle everything!
    voice_control = TimerAction(
        period=1.0,
        actions=[LogInfo(msg="Old Heavy Speech Recognition Nodes Disabled. Using Lightweight Mobile Browser Mic!")]
    )

    perception_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'perception.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': perception_params_sim,
            'enable_detector': enable_detector,
            'detector_params_file': detector_params_sim,
            'enable_marker_detector': enable_marker_detector,
            'marker_params_file': marker_params_sim,
            'enable_camera_safety': enable_camera_safety,
            'camera_safety_params_file': camera_safety_params_sim,
            'object_memory_file': object_memory_sim,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", enable_perception, "' == 'true' and '", hardware_mode, "' == 'sim'"]
            )
        ),
    )

    perception_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'perception.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': perception_params_real,
            'enable_detector': enable_detector,
            'detector_params_file': detector_params_real,
            'enable_marker_detector': enable_marker_detector,
            'marker_params_file': marker_params_real,
            'enable_camera_safety': enable_camera_safety,
            'camera_safety_params_file': camera_safety_params_real,
            'object_memory_file': object_memory_real,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", enable_perception, "' == 'true' and '", hardware_mode, "' == 'real'"]
            )
        ),
    )

    task_manager_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'task_manager.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': task_params_sim,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", enable_task_manager, "' == 'true' and '", hardware_mode, "' == 'sim'"]
            )
        ),
    )

    task_manager_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'task_manager.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': task_params_real,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", enable_task_manager, "' == 'true' and '", hardware_mode, "' == 'real'"]
            )
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value=''),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'mode',
                default_value='mapping',
                choices=['drive', 'mapping', 'amcl', 'navigation'],
                description='drive: base + teleop, mapping: slam_toolbox, amcl: localization with saved map, navigation: amcl + nav2',
            ),
        DeclareLaunchArgument(
            'mapping_localizer_type',
            default_value='slam_toolbox',
            choices=['slam_toolbox', 'cartographer'],
            description='Localizer backend when mode=mapping',
        ),
            DeclareLaunchArgument(
                'hardware_mode',
                default_value='real',
                choices=['sim', 'real'],
                description='sim: run Gazebo stack, real: run hardware base stack',
            ),
            DeclareLaunchArgument('map', default_value=''),
            DeclareLaunchArgument('world', default_value=world),
            DeclareLaunchArgument('robot_model', default_value='minibot'),
            DeclareLaunchArgument('workflow_use_rviz', default_value='false'),
            DeclareLaunchArgument(
                'rviz_config',
                default_value='',
                description='Optional explicit RViz config file. Leave empty to auto-select by mode',
            ),
            DeclareLaunchArgument('use_gz_gui', default_value='true'),
            DeclareLaunchArgument('gz_verbosity', default_value='1'),
            DeclareLaunchArgument('lidar_offset_x', default_value='-0.0885'),
            DeclareLaunchArgument('lidar_offset_y', default_value='0.0'),
            DeclareLaunchArgument('lidar_offset_z', default_value='0.0275'),
            DeclareLaunchArgument('lidar_yaw', default_value='3.14159'),
            DeclareLaunchArgument('cleanup_on_start', default_value='true'),
            DeclareLaunchArgument('ros_localhost_only', default_value='0'),
            DeclareLaunchArgument('ros_discovery_range', default_value='SUBNET'),
            DeclareLaunchArgument('ros_domain_id', default_value='0'),
            DeclareLaunchArgument('restart_ros_daemon', default_value='false'),
            DeclareLaunchArgument('odom_topic', default_value='/odom'),
            DeclareLaunchArgument('serial_port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
            DeclareLaunchArgument('serial_baud', default_value='115200'),
            DeclareLaunchArgument('wheel_separation', default_value='0.236'),
            DeclareLaunchArgument('wheel_radius', default_value='0.0325'),
            DeclareLaunchArgument('encoder_cpr', default_value='330'),
            DeclareLaunchArgument('nav_max_angular', default_value='0.60'),
            DeclareLaunchArgument('nav_min_turn_pwm', default_value='115'),
            DeclareLaunchArgument('nav_turn_pwm_scale', default_value='0.65'),
            DeclareLaunchArgument('nav_turn_in_place_threshold', default_value='0.08'),
            DeclareLaunchArgument('nav_turn_assist_cmd_w_threshold', default_value='0.10'),
            DeclareLaunchArgument('nav_turn_assist_min_pwm_delta', default_value='0'),
            DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
            DeclareLaunchArgument('lidar_baud', default_value='115200'),
            DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
            DeclareLaunchArgument('enable_lidar', default_value='true'),
            DeclareLaunchArgument('enable_teleop', default_value='true'),
            DeclareLaunchArgument('stack_start_delay', default_value='12.0'),
            DeclareLaunchArgument('enable_voice', default_value='false'),
            DeclareLaunchArgument('voice_use_mic', default_value='false'),
            DeclareLaunchArgument('enable_phone_text', default_value='true'),
            DeclareLaunchArgument('phone_text_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('phone_text_port', default_value='5000'),
            DeclareLaunchArgument('voice_params', default_value=voice_params),
            DeclareLaunchArgument('mapping_slam_params', default_value=mapping_slam_params_default),
            DeclareLaunchArgument('enable_perception', default_value='false'),
            DeclareLaunchArgument('enable_detector', default_value='false'),
            DeclareLaunchArgument('enable_marker_detector', default_value='false'),
            DeclareLaunchArgument('enable_camera_safety', default_value='true'),
            DeclareLaunchArgument('enable_camera', default_value='true'),
            DeclareLaunchArgument(
                'enable_arm_driver',
                default_value='false',
                description='Start Nano serial arm driver when running on real hardware',
            ),
            DeclareLaunchArgument('camera_driver', default_value='libcamera'),
            DeclareLaunchArgument('camera_video_device', default_value='/dev/video0'),
            DeclareLaunchArgument('camera_params', default_value=camera_params),
            DeclareLaunchArgument(
                'arm_port',
                default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
            ),
            DeclareLaunchArgument('arm_baud', default_value='9600'),
            DeclareLaunchArgument('arm_serial_boot_wait_sec', default_value='2.0'),
            DeclareLaunchArgument('enable_task_manager', default_value='false'),
            DeclareLaunchArgument(
                'spawn_arm_controller',
                default_value='false',
                description='Spawn arm controller in sim stack (disable for mapping stability)'
            ),
            DeclareLaunchArgument(
                'include_arm',
                default_value='true',
                description='Include the physical arm in the real robot description'
            ),
            DeclareLaunchArgument('perception_params_sim', default_value=perception_params_sim),
            DeclareLaunchArgument('perception_params_real', default_value=perception_params_real),
            DeclareLaunchArgument('task_params_sim', default_value=task_params_sim),
            DeclareLaunchArgument('task_params_real', default_value=task_params_real),
            DeclareLaunchArgument('detector_params_sim', default_value=detector_params_sim),
            DeclareLaunchArgument('detector_params_real', default_value=detector_params_real),
            DeclareLaunchArgument('marker_params_sim', default_value=marker_params_sim),
            DeclareLaunchArgument('marker_params_real', default_value=marker_params_real),
            DeclareLaunchArgument('camera_safety_params_sim', default_value=camera_safety_params_sim),
            DeclareLaunchArgument('camera_safety_params_real', default_value=camera_safety_params_real),
            DeclareLaunchArgument('object_memory_sim', default_value=object_memory_sim),
            DeclareLaunchArgument('object_memory_real', default_value=object_memory_real),
            LogInfo(msg=['============ STARTING WORKFLOW MODE: ', mode, ' ============']),
            LogInfo(msg=['============ HARDWARE MODE: ', hardware_mode, ' ============']),
            ros_local_only,
            ros_discovery_local,
            ros_domain,
            ros_static_peers_unset,
            ros_daemon_stop,
            ros_daemon_start,
            sim_stack,
            real_rsp,
            real_serial_drive,
            real_lidar,
            real_teleop,
            real_camera,
            real_arm_driver,
            rviz,
            delayed_localizers,
            stable_scan_cloud,
            delayed_navigation,
            voice_control,
            perception_sim,
            perception_real,
            task_manager_sim,
            task_manager_real,

            # ---------- NEW AI INTEGRATIONS FOR WORKFLOW ----------
            # Start Voice UI Website from the installed ROS executable so the path
            # stays portable across laptop and Pi copies of the workspace.
            Node(
                package='articubot_one',
                executable='omni_voice_server.py',
                name='omni_voice_server',
                output='screen',
                condition=IfCondition(enable_voice),
            ),

            # Start Voice Command Brain
            Node(
                package='articubot_one',
                executable='omni_robot_commander.py',
                name='omni_robot_commander',
                output='screen',
                condition=IfCondition(enable_voice),
            ),

            # -----------------------------------------------------
        ]
    )
