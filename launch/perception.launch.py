from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    params_file = LaunchConfiguration(
        "params_file",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "perception_params_sim.yaml"]),
    )
    enable_detector = LaunchConfiguration("enable_detector", default="false")
    detector_params_file = LaunchConfiguration(
        "detector_params_file",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "detector_params_sim.yaml"]),
    )
    enable_marker_detector = LaunchConfiguration("enable_marker_detector", default="false")
    marker_params_file = LaunchConfiguration(
        "marker_params_file",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "marker_params_sim.yaml"]),
    )
    enable_camera_safety = LaunchConfiguration("enable_camera_safety", default="true")
    camera_safety_params_file = LaunchConfiguration(
        "camera_safety_params_file",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "camera_safety_params_sim.yaml"]),
    )
    object_memory_file = LaunchConfiguration(
        "object_memory_file",
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), "assets", "memory", "object_memory_sim.json"]
        ),
    )

    object_query_node = Node(
        package=package_name,
        executable="object_query_node.py",
        name="object_query_node",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time, "object_memory_file": object_memory_file},
        ],
    )

    object_detector_node = Node(
        package=package_name,
        executable="object_detector_node.py",
        name="object_detector_node",
        output="screen",
        parameters=[detector_params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_detector),
    )
    marker_detector_node = Node(
        package=package_name,
        executable="marker_detector_node.py",
        name="marker_detector_node",
        output="screen",
        parameters=[marker_params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_marker_detector),
    )
    camera_safety_node = Node(
        package=package_name,
        executable="camera_safety_node.py",
        name="camera_safety_node",
        output="screen",
        parameters=[camera_safety_params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_camera_safety),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=params_file),
            DeclareLaunchArgument("enable_detector", default_value="false"),
            DeclareLaunchArgument("detector_params_file", default_value=detector_params_file),
            DeclareLaunchArgument("enable_marker_detector", default_value="false"),
            DeclareLaunchArgument("marker_params_file", default_value=marker_params_file),
            DeclareLaunchArgument("enable_camera_safety", default_value="true"),
            DeclareLaunchArgument("camera_safety_params_file", default_value=camera_safety_params_file),
            DeclareLaunchArgument("object_memory_file", default_value=object_memory_file),
            object_detector_node,
            marker_detector_node,
            camera_safety_node,
            object_query_node,
        ]
    )
