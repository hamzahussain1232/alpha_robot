from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    params_file = LaunchConfiguration(
        "params_file",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "detector_params_sim.yaml"]),
    )

    detector_node = Node(
        package=package_name,
        executable="object_detector_node.py",
        name="object_detector_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=params_file),
            detector_node,
        ]
    )

