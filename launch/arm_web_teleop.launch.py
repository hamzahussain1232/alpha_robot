from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    start_arm_driver = LaunchConfiguration('start_arm_driver')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    arm_nano = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'arm_nano.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_rsp': 'false',
        }.items(),
        condition=IfCondition(start_arm_driver),
    )

    arm_web = Node(
        package=package_name,
        executable='arm_web_teleop.py',
        name='arm_web_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'host': host,
            'port': port,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_arm_driver', default_value='true'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8090'),
        arm_nano,
        arm_web,
    ])
