from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'articubot_one'

    # ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default='')

    # 1. STATIC TF (Fake Localization)
    # Assumes 'tf_map_odom.launch.py' is in the main 'launch/' folder
    tf_map_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'tf_map_odom.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace
        }.items()
    )

    # 2. MAP SERVER (The Main Worker)
    # Assumes 'map_server.launch.py' is in the main 'launch/' folder
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

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map', 
            default_value='', 
            description='Path to map YAML file for map_server'
        ),

        LogInfo(msg=['============ STARTING MAP SERVER + STATIC TF ============']),

        tf_map_odom,
        map_server,
    ])