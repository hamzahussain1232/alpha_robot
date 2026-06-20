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
    enable_phone_text = LaunchConfiguration("enable_phone_text", default="false")
    phone_text_host = LaunchConfiguration("phone_text_host", default="0.0.0.0")
    phone_text_port = LaunchConfiguration("phone_text_port", default="5000")
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

    phone_text_node = Node(
        package=package_name,
        executable="voice_text_publisher.py",
        name="phone_text_server",
        output="screen",
        arguments=["--serve", "--host", phone_text_host, "--port", phone_text_port],
        condition=IfCondition(enable_phone_text),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_mic", default_value="true"),
            DeclareLaunchArgument("enable_phone_text", default_value="false"),
            DeclareLaunchArgument("phone_text_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("phone_text_port", default_value="5000"),
            DeclareLaunchArgument("voice_params", default_value=voice_params),
            voice_command_node,
            voice_mic_node,
            phone_text_node,
        ]
    )
