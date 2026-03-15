from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Keep args compatible with parent launch files.
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_params_file = PathJoinSubstitution(
        [FindPackageShare('articubot_one'), 'config', 'mapper_params_online_async.yaml']
    )

    # Use slam_toolbox's official lifecycle event launch flow.
    # This avoids Nav2 lifecycle-manager bond heartbeat resets in simulation.
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/opt/ros/jazzy/share/slam_toolbox/launch/online_async_launch.py'
        ),
        launch_arguments={
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        LogInfo(msg=['============ STARTING SLAM TOOLBOX ============']),
        LogInfo(msg=['Params: ', slam_params_file]),
        LogInfo(msg=['Namespace: ', namespace]),
        slam_toolbox_launch,
    ])

