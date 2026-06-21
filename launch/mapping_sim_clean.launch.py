from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_gz_gui = LaunchConfiguration('use_gz_gui', default='true')
    ros_domain_id = LaunchConfiguration('ros_domain_id', default='42')
    ros_localhost_only = LaunchConfiguration('ros_localhost_only', default='1')
    cleanup_on_start = LaunchConfiguration('cleanup_on_start', default='true')
    show_rviz = LaunchConfiguration('show_rviz', default='true')
    rviz_start_delay = LaunchConfiguration('rviz_start_delay', default='18.0')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    controllers_file = LaunchConfiguration('controllers_file')

    drive_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'drive_sim.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
            'use_gz_gui': use_gz_gui,
            'spawn_arm_controller': 'false',
            'use_ekf': 'true',
            'controllers_file': controllers_file,
            'cleanup_on_start': cleanup_on_start,
            'ros_domain_id': ros_domain_id,
            'ros_localhost_only': ros_localhost_only,
        }.items(),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/opt/ros/jazzy/share/slam_toolbox/launch/online_async_launch.py'
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'slam_params_file': slam_params_file,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_QPA_PLATFORM': 'xcb',
        },
        output='screen',
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gz_gui', default_value='true'),
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        DeclareLaunchArgument('ros_localhost_only', default_value='1'),
        DeclareLaunchArgument('cleanup_on_start', default_value='true'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_start_delay', default_value='18.0'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'slam_toolbox_clean.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'mapping_clean.rviz']
            ),
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'my_controllers_sim_ekf.yaml']
            ),
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', ros_localhost_only),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        drive_sim,
        TimerAction(period=12.0, actions=[slam_toolbox]),
        TimerAction(period=rviz_start_delay, actions=[rviz]),
    ])
