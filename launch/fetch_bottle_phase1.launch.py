from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "articubot_one"

    launch_camera = LaunchConfiguration("launch_camera")
    launch_detector = LaunchConfiguration("launch_detector")
    camera_driver = LaunchConfiguration("camera_driver")
    video_device = LaunchConfiguration("video_device")
    camera_params = LaunchConfiguration("camera_params")
    detector_params = LaunchConfiguration("detector_params")
    fetch_params = LaunchConfiguration("fetch_params")

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", "camera.launch.py"])
        ),
        launch_arguments={
            "driver": camera_driver,
            "video_device": video_device,
            "params_file": camera_params,
        }.items(),
        condition=IfCondition(launch_camera),
    )

    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", "detector.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": detector_params,
        }.items(),
        condition=IfCondition(launch_detector),
    )

    fetch_node = Node(
        package=pkg,
        executable="fetch_bottle_node.py",
        name="fetch_bottle_node",
        output="screen",
        parameters=[fetch_params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_camera", default_value="false"),
            DeclareLaunchArgument("launch_detector", default_value="false"),
            DeclareLaunchArgument("camera_driver", default_value="libcamera"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument(
                "camera_params",
                default_value=PathJoinSubstitution([FindPackageShare(pkg), "config", "camera_libcamera_params.yaml"]),
            ),
            DeclareLaunchArgument(
                "detector_params",
                default_value=PathJoinSubstitution([FindPackageShare(pkg), "config", "detector_params_onnx_real.yaml"]),
            ),
            DeclareLaunchArgument(
                "fetch_params",
                default_value=PathJoinSubstitution([FindPackageShare(pkg), "config", "fetch_bottle_phase1_real.yaml"]),
            ),
            camera_launch,
            detector_launch,
            fetch_node,
        ]
    )
