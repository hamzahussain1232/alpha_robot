from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_mic = LaunchConfiguration("use_mic", default="true")
    voice_params = LaunchConfiguration(
        "voice_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "voice_params.yaml"]),
    )

    voice_command_node = Node(
        package=package_name,
        executable="voice_command_node.py",
        name="voice_command_node",
        output="screen",
        parameters=[voice_params, {"use_sim_time": use_sim_time}],
    )

    voice_mic_node = Node(
        package=package_name,
        executable="voice_mic_node.py",
        name="voice_mic_node",
        output="screen",
        parameters=[voice_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_mic),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_mic", default_value="true"),
            DeclareLaunchArgument("voice_params", default_value=voice_params),
            voice_command_node,
            voice_mic_node,
        ]
    )
