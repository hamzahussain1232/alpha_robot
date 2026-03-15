from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'articubot_one'

    # ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # CALL THE MAIN CARTOGRAPHER FILE
    # This points to 'launch/cartographer.launch.py' (the one inside the main launch folder)
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'cartographer.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            # We don't need robot_model anymore since we fixed the main file to use 'config/' directly
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        LogInfo(msg=['============ STARTING CARTOGRAPHER (Wrapper) ============']),

        cartographer,
    ])
