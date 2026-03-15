from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'articubot_one'

    # ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default='')

    # CALL THE MAIN FILE
    # This points to the file in the main 'launch/' folder
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
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),

        map_server,
    ])
