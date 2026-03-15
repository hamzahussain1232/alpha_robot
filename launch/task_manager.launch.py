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
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "task_params_sim.yaml"]),
    )

    task_manager_node = Node(
        package=package_name,
        executable="task_manager_node.py",
        name="task_manager_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=params_file),
            task_manager_node,
        ]
    )

