from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    launch_camera = LaunchConfiguration("launch_camera", default="true")
    video_device = LaunchConfiguration("video_device", default="/dev/video0")
    camera_params = LaunchConfiguration(
        "camera_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "camera_v4l2_params.yaml"]),
    )
    gesture_params = LaunchConfiguration(
        "gesture_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "hand_gesture_params.yaml"]),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "camera.launch.py"])
        ),
        launch_arguments={
            "video_device": video_device,
            "params_file": camera_params,
        }.items(),
        condition=IfCondition(launch_camera),
    )

    gesture_node = Node(
        package=package_name,
        executable="hand_gesture_teleop_node.py",
        name="hand_gesture_teleop_node",
        output="screen",
        parameters=[gesture_params, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_params", default_value=camera_params),
            DeclareLaunchArgument("gesture_params", default_value=gesture_params),
            camera_launch,
            gesture_node,
        ]
    )
