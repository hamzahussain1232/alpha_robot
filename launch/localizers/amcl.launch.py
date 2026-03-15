from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    package_name = 'articubot_one'

    # ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default='')

    # CONFIG PATHS
    # We use nav2_params.yaml because it contains the 'amcl' section we created earlier.
    amcl_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'nav2_params.yaml'
    ])

    # INCLUDE MAP SERVER (The "Main" one from launch/)
    # AMCL cannot work without a map, so we launch the map server here.
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'map_server.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'map': map_file,
        }.items()
    )

    # AMCL NODE
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        namespace=namespace,
        name='amcl',
        output='screen',
        parameters=[amcl_params_file, {'use_sim_time': use_sim_time}],
        # Remap odom to your EKF output (odometry/local)
        remappings=[('odom', '/odometry/local'), ('scan', '/scan')]
    )

    # LIFECYCLE MANAGER (Specifically for AMCL)
    # Note: Map Server has its own lifecycle manager in its own launch file
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=namespace,
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl']
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        DeclareLaunchArgument(
            'map', 
            default_value='', 
            description='Path to map YAML file (required)'
        ),

        LogInfo(msg=['============ STARTING AMCL LOCALIZATION ============']),
        LogInfo(msg=['Params: ', amcl_params_file]),

        map_server,
        amcl_node,
        lifecycle_manager,
    ])
