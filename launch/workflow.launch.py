from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EqualsSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    mode = LaunchConfiguration('mode', default='mapping')
    mapping_localizer_type = LaunchConfiguration('mapping_localizer_type', default='slam_toolbox')
    hardware_mode = LaunchConfiguration('hardware_mode', default='sim')
    map_file = LaunchConfiguration('map', default='')
    world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), 'assets', 'worlds', 'home_assistant_room.sdf']
        ),
    )
    robot_model = LaunchConfiguration('robot_model', default='minibot')
    workflow_use_rviz = LaunchConfiguration('workflow_use_rviz', default='true')
    rviz_config = LaunchConfiguration(
        'rviz_config',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'map.rviz']),
    )
    use_gz_gui = LaunchConfiguration('use_gz_gui', default='true')
    gz_verbosity = LaunchConfiguration('gz_verbosity', default='1')
    lidar_offset_x = LaunchConfiguration('lidar_offset_x', default='-0.08')
    lidar_offset_y = LaunchConfiguration('lidar_offset_y', default='0.0')
    lidar_offset_z = LaunchConfiguration('lidar_offset_z', default='0.0275')
    cleanup_on_start = LaunchConfiguration('cleanup_on_start', default='true')
    ros_localhost_only = LaunchConfiguration('ros_localhost_only', default='1')
    ros_domain_id = LaunchConfiguration('ros_domain_id', default='42')
    stack_start_delay = LaunchConfiguration('stack_start_delay', default='12.0')
    enable_voice = LaunchConfiguration('enable_voice', default='false')
    voice_use_mic = LaunchConfiguration('voice_use_mic', default='true')
    voice_params = LaunchConfiguration(
        'voice_params',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'voice_params.yaml']),
    )
    enable_perception = LaunchConfiguration('enable_perception', default='false')
    enable_detector = LaunchConfiguration('enable_detector', default='false')
    enable_marker_detector = LaunchConfiguration('enable_marker_detector', default='false')
    enable_camera_safety = LaunchConfiguration('enable_camera_safety', default='true')
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    camera_video_device = LaunchConfiguration('camera_video_device', default='/dev/video0')
    camera_params = LaunchConfiguration(
        'camera_params',
        default=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'camera_v4l2_params.yaml']),
    )
    enable_task_manager = LaunchConfiguration('enable_task_manager', default='false')
    spawn_arm_controller = LaunchConfiguration('spawn_arm_controller', default='false')
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
        value='LOCALHOST',
    )

    ros_domain = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=ros_domain_id,
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
            'cleanup_on_start': cleanup_on_start,
            'ros_localhost_only': ros_localhost_only,
            'ros_domain_id': ros_domain_id,
        }.items(),
        condition=IfCondition(EqualsSubstitution(hardware_mode, 'sim')),
    )

    real_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'drive.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(EqualsSubstitution(hardware_mode, 'real')),
    )

    real_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'camera.launch.py'])
        ),
        launch_arguments={
            'video_device': camera_video_device,
            'params_file': camera_params,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", enable_camera, "' == 'true' and '", hardware_mode, "' == 'real'"]
            )
        ),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'launch_rviz.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
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
            'map': '',
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'mapping')),
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

    voice_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'voice_control.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_mic': voice_use_mic,
            'voice_params': voice_params,
        }.items(),
        condition=IfCondition(enable_voice),
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
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'mode',
                default_value='mapping',
                choices=['mapping', 'amcl', 'navigation'],
                description='mapping: slam_toolbox, amcl: localization with saved map, navigation: amcl + nav2',
            ),
        DeclareLaunchArgument(
            'mapping_localizer_type',
            default_value='slam_toolbox',
            choices=['slam_toolbox', 'cartographer'],
            description='Localizer backend when mode=mapping',
        ),
            DeclareLaunchArgument(
                'hardware_mode',
                default_value='sim',
                choices=['sim', 'real'],
                description='sim: run Gazebo stack, real: run hardware base stack',
            ),
            DeclareLaunchArgument('map', default_value=''),
            DeclareLaunchArgument('world', default_value=world),
            DeclareLaunchArgument('robot_model', default_value='minibot'),
            DeclareLaunchArgument('workflow_use_rviz', default_value='true'),
            DeclareLaunchArgument('rviz_config', default_value=rviz_config),
            DeclareLaunchArgument('use_gz_gui', default_value='true'),
            DeclareLaunchArgument('gz_verbosity', default_value='1'),
            DeclareLaunchArgument('lidar_offset_x', default_value='-0.08'),
            DeclareLaunchArgument('lidar_offset_y', default_value='0.0'),
            DeclareLaunchArgument('lidar_offset_z', default_value='0.0275'),
            DeclareLaunchArgument('cleanup_on_start', default_value='true'),
            DeclareLaunchArgument('ros_localhost_only', default_value='1'),
            DeclareLaunchArgument('ros_domain_id', default_value='42'),
            DeclareLaunchArgument('stack_start_delay', default_value='12.0'),
            DeclareLaunchArgument('enable_voice', default_value='false'),
            DeclareLaunchArgument('voice_use_mic', default_value='true'),
            DeclareLaunchArgument('voice_params', default_value=voice_params),
            DeclareLaunchArgument('enable_perception', default_value='false'),
            DeclareLaunchArgument('enable_detector', default_value='false'),
            DeclareLaunchArgument('enable_marker_detector', default_value='false'),
            DeclareLaunchArgument('enable_camera_safety', default_value='true'),
            DeclareLaunchArgument('enable_camera', default_value='true'),
            DeclareLaunchArgument('camera_video_device', default_value='/dev/video0'),
            DeclareLaunchArgument('camera_params', default_value=camera_params),
            DeclareLaunchArgument('enable_task_manager', default_value='false'),
            DeclareLaunchArgument(
                'spawn_arm_controller',
                default_value='false',
                description='Spawn arm controller in sim stack (disable for mapping stability)'
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
            sim_stack,
            real_stack,
            real_camera,
            rviz,
            delayed_localizers,
            delayed_navigation,
            voice_control,
            perception_sim,
            perception_real,
            task_manager_sim,
            task_manager_real,
        ]
    )
