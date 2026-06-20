from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'articubot_one'

    # ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_composition = LaunchConfiguration('use_composition', default='False')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')

    # FIXED PATH: Point directly to config/nav2_params.yaml
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'nav2_params.yaml'
    ])

    # Define the ComposableNodeContainer for Nav2 composition
    container_nav2 = ComposableNodeContainer(
        package='rclcpp_components',
        namespace=namespace,
        executable='component_container_mt', # Multi-threaded container
        name='nav2_container',
        composable_node_descriptions=[], 
        parameters=[nav2_params_file],
        output='screen',
        condition=IfCondition(use_composition)
    )

    # Launch the main Navigation Logic
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'use_composition': use_composition,
            'container_name': 'nav2_container',
            'odom_topic': odom_topic,
            'autostart': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    # Nav2 publishes geometry_msgs/Twist. Our drive stack and twist_mux use TwistStamped.
    # Convert Nav2 command stream into stamped commands on /cmd_vel_nav.
    nav_cmd_stamper = Node(
        package=package_name,
        executable='twist_stamper.py',
        name='nav_cmd_stamper',
        namespace=namespace,
        output='screen',
        parameters=[{
            'input_topic': 'cmd_vel_nav_unstamped',
            'output_topic': 'cmd_vel_nav',
            'frame_id': 'base_link',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),

        LogInfo(msg=['============ STARTING NAV2 STACK ============']),
        LogInfo(msg=['Params file: ', nav2_params_file]),

        container_nav2,
        nav2,
        nav_cmd_stamper,
    ])
