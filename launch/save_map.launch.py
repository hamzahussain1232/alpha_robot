from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'
    map_file = LaunchConfiguration('map_file', default='my_map')
    map_server_params = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'map_server_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='my_map',
            description='Output map base name (no .yaml/.pgm extension)',
        ),
        LogInfo(msg=['============ SAVING MAP TO: ', map_file, ' ============']),
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver_cli',
            output='screen',
            arguments=['-f', map_file],
            parameters=[map_server_params],
        ),
    ])
