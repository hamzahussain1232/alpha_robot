from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
import subprocess
import time

from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def _cleanup_stale_sim_processes(context, *args, **kwargs):
    enabled = LaunchConfiguration('cleanup_on_start').perform(context).lower() in ('true', '1', 'yes', 'on')
    if not enabled:
        return [LogInfo(msg='Skipping stale process cleanup')]

    # Kill stale processes that commonly survive interrupted runs and cause duplicate /clock publishers.
    patterns = [
        r'[o]pt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge',
        r'[o]pt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher',
        r'[o]pt/ros/jazzy/lib/robot_localization/ekf_node',
        r'[o]pt/ros/jazzy/lib/twist_mux/twist_mux',
        r'[o]pt/ros/jazzy/lib/controller_manager/spawner',
        r'[o]pt/ros/jazzy/lib/controller_manager/ros2_control_node',
        r'[o]pt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager',
        r'[o]pt/ros/jazzy/lib/rviz2/rviz2',
        r'[o]pt/ros/jazzy/lib/ros_gz_sim/create',
        r'[a]rticubot_one/lib/articubot_one/keyboard_bridge.py',
        r'[o]pt/ros/jazzy/lib/nav2_amcl/amcl',
        r'[o]pt/ros/jazzy/lib/nav2_map_server/map_server',
        r'[o]pt/ros/jazzy/lib/nav2_controller/controller_server',
        r'[o]pt/ros/jazzy/lib/nav2_planner/planner_server',
        r'[o]pt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator',
        r'[o]pt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node',
        r'[o]pt/ros/jazzy/lib/cartographer_ros/cartographer_node',
        r'[o]pt/ros/jazzy/lib/cartographer_ros/cartographer_occupancy_grid_node',
        r'[g]z sim .*articubot_one/.*/assets/worlds/.*\.sdf',
        r'[g]z sim -g',
        r'[r]uby .*/gz_tools_vendor/bin/gz sim .*articubot_one/.*/assets/worlds/.*\.sdf',
        r'[r]uby .*/gz_tools_vendor/bin/gz sim .*--force-version 8',
    ]
    for pattern in patterns:
        subprocess.run(['pkill', '-f', pattern], check=False)

    # Gazebo may ignore SIGTERM when heavily loaded; enforce cleanup before spawning a new world.
    hard_kill_patterns = [
        r'[o]pt/ros/jazzy/lib/controller_manager/spawner',
        r'[o]pt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager',
        r'[o]pt/ros/jazzy/lib/ros_gz_sim/create',
        r'[o]pt/ros/jazzy/lib/cartographer_ros/cartographer_node',
        r'[o]pt/ros/jazzy/lib/cartographer_ros/cartographer_occupancy_grid_node',
        r'[g]z sim .*articubot_one/.*/assets/worlds/.*\.sdf',
        r'[g]z sim -g',
        r'[r]uby .*/gz_tools_vendor/bin/gz sim .*articubot_one/.*/assets/worlds/.*\.sdf',
        r'[r]uby .*/gz_tools_vendor/bin/gz sim .*--force-version 8',
    ]
    time.sleep(0.5)
    for pattern in hard_kill_patterns:
        subprocess.run(['pkill', '-9', '-f', pattern], check=False)

    # Give process table a brief moment to settle to avoid controller-manager races.
    time.sleep(0.3)

    return [LogInfo(msg='Cleaned stale Gazebo/ROS processes from previous runs')]

def generate_launch_description():

    package_name = 'articubot_one'

    # 1. ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_model = LaunchConfiguration('robot_model', default='minibot')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gz_gui = LaunchConfiguration('use_gz_gui', default='true')
    gz_verbosity = LaunchConfiguration('gz_verbosity', default='1')
    spawn_arm_controller = LaunchConfiguration('spawn_arm_controller', default='false')
    lidar_offset_x = LaunchConfiguration('lidar_offset_x', default='-0.08')
    lidar_offset_y = LaunchConfiguration('lidar_offset_y', default='0.0')
    lidar_offset_z = LaunchConfiguration('lidar_offset_z', default='0.0275')
    cleanup_on_start = LaunchConfiguration('cleanup_on_start', default='true')
    ros_localhost_only = LaunchConfiguration('ros_localhost_only', default='1')
    ros_domain_id = LaunchConfiguration('ros_domain_id', default='42')
    
    # DEFINING THE WORLD PATH
    world_file_name = 'home_assistant_room.sdf'
    world_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'assets', 'worlds', world_file_name
    ])
    
    robot_world = LaunchConfiguration('world')

    # 2. GAZEBO SETUP
    # Add assets path so Gazebo can find meshes/materials
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare(package_name), 'assets', 'worlds']),
            TextSubstitution(text=':'),
            PathJoinSubstitution([FindPackageShare(package_name), 'assets'])
        ]
    )

    # Keep Gazebo logs in a writable location to avoid startup aborts if ~/.gz has bad permissions.
    gazebo_log_path = SetEnvironmentVariable(
        name='GZ_LOG_PATH',
        value='/tmp/gz_logs'
    )

    # Avoid FastDDS shared-memory lock errors that spam terminal output.
    fastrtps_udp_only = SetEnvironmentVariable(
        name='FASTDDS_BUILTIN_TRANSPORTS',
        value='UDPv4'
    )

    # Keep this simulation isolated from other ROS 2 participants on LAN.
    ros_local_only = SetEnvironmentVariable(
        name='ROS_LOCALHOST_ONLY',
        value=ros_localhost_only
    )

    # Newer discovery control; reinforces localhost-only behavior.
    ros_discovery_local = SetEnvironmentVariable(
        name='ROS_AUTOMATIC_DISCOVERY_RANGE',
        value='LOCALHOST'
    )

    # Put this project on its own DDS domain to avoid cross-talk from other ROS graphs.
    ros_domain = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=ros_domain_id
    )

    # Launch exactly one Gazebo process: GUI or headless.
    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [robot_world, ' -r -v ', gz_verbosity],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(use_gz_gui)
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [robot_world, ' -s -r -v ', gz_verbosity],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(use_gz_gui)
    )

    # 3. ROBOT STATE PUBLISHER
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'include_arm': spawn_arm_controller,
            'lidar_offset_x': lidar_offset_x,
            'lidar_offset_y': lidar_offset_y,
            'lidar_offset_z': lidar_offset_z,
        }.items()
    )

    # 4. TWIST MUX
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'twist_mux.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. SPAWN ROBOT
    spawn_sim_robot = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=[
            '-name', robot_model,
            '-topic', '/robot_description', 
            '-x', '0.0', '-y', '0.0', '-z', '0.1', # Lifted z slightly to prevent floor clipping
            '-Y', '0.0',
            '-allow_renaming', 'false'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # 6. RVIZ
    rviz_and_joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'launch_rviz.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'show_gui': 'false'}.items(),
        condition=IfCondition(use_rviz)
    )

    # 8. BRIDGE
    bridge_config_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'gz_ros_bridge.yaml'
    ])

    gz_bridge = Node(
        package='ros_gz_bridge',
        namespace=namespace,
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # 9. CONTROLLERS
    joint_broad_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=[
            "joint_broad",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "90",
            "--switch-timeout", "90",
            "--service-call-timeout", "30",
        ],
        output="screen"
    )

    arm_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "90",
            "--switch-timeout", "90",
            "--service-call-timeout", "30",
        ],
        output="screen"
    )

    diff_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=[
            "diff_cont",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "90",
            "--switch-timeout", "90",
            "--service-call-timeout", "30",
        ],
        output="screen"
    )

    # 10. KEYBOARD BRIDGE
    keyboard_bridge_node = Node(
        package='articubot_one',
        executable='keyboard_bridge.py',
        name='keyboard_bridge',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # One-shot delayed startup (no event re-triggers), avoids duplicate spawner execution.
    delayed_spawn_robot = TimerAction(period=3.0, actions=[spawn_sim_robot])
    delayed_joint_broad_spawner = TimerAction(period=7.0, actions=[joint_broad_spawner])
    delayed_diff_spawner = TimerAction(period=9.0, actions=[diff_spawner])
    delayed_arm_spawner = TimerAction(
        period=11.0,
        actions=[arm_spawner],
        condition=IfCondition(spawn_arm_controller),
    )

    # 11. EKF (Localization)
    ekf_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'ekf_sim.yaml'
    ])
    ekf_imu_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'ekf_imu_odom.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'params_file': ekf_params_file,
        }.items()
    )

    return LaunchDescription([
        # DEFAULT ARGUMENTS
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_model', default_value='minibot'),
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gz_gui', default_value='true'),
        DeclareLaunchArgument('gz_verbosity', default_value='1'),
        DeclareLaunchArgument(
            'spawn_arm_controller',
            default_value='false',
            description='Spawn arm trajectory controller in simulation'
        ),
        DeclareLaunchArgument(
            'lidar_offset_x',
            default_value='0.0',
            description='LiDAR X offset from chassis center (meters, +front)'
        ),
        DeclareLaunchArgument(
            'lidar_offset_y',
            default_value='0.0',
            description='LiDAR Y offset from chassis center (meters, +left)'
        ),
        DeclareLaunchArgument(
            'lidar_offset_z',
            default_value='0.0275',
            description='LiDAR height offset above chassis top (meters)'
        ),
        DeclareLaunchArgument('cleanup_on_start', default_value='true'),
        DeclareLaunchArgument(
            'ros_localhost_only',
            default_value='1',
            description='Set to 1 to isolate ROS graph to this machine only'
        ),
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='42',
            description='DDS domain ID used by this simulation stack'
        ),
        
        LogInfo(msg=['============ STARTING SIMULATION ============']),
        OpaqueFunction(function=_cleanup_stale_sim_processes),
        
        # LAUNCH ORDER
        # Running from Snap shells can inject incompatible GTK/Snap libs.
        # Unsetting these keeps Gazebo GUI / RViz stable.
        UnsetEnvironmentVariable(name='GTK_PATH'),
        UnsetEnvironmentVariable(name='SNAP_LIBRARY_PATH'),
        fastrtps_udp_only,
        ros_local_only,
        ros_discovery_local,
        ros_domain,
        gazebo_resource_path,
        gazebo_log_path,
        gazebo_with_gui,
        gazebo_headless,
        gz_bridge,
        delayed_spawn_robot,
        rsp,
        rviz_and_joystick,
        twist_mux,
        delayed_joint_broad_spawner,
        delayed_arm_spawner,
        delayed_diff_spawner,
        keyboard_bridge_node,
        ekf_imu_odom
    ])
