from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    video_device = LaunchConfiguration("video_device")
    camera_driver = LaunchConfiguration("camera_driver")
    launch_camera = LaunchConfiguration("launch_camera")
    launch_web_video = LaunchConfiguration("launch_web_video")
    camera_params = LaunchConfiguration(
        "camera_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "camera_libcamera_params.yaml"]),
    )
    detector_params = LaunchConfiguration(
        "detector_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "detector_params_bootstrap_real.yaml"]),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "camera.launch.py"])
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
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "detector.launch.py"])
        ),
        launch_arguments={"use_sim_time": "false", "params_file": detector_params}.items(),
    )

    web_video = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        condition=IfCondition(launch_web_video),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_driver", default_value="libcamera"),
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("launch_web_video", default_value="false"),
            DeclareLaunchArgument("camera_params", default_value=camera_params),
            DeclareLaunchArgument("detector_params", default_value=detector_params),
            camera_launch,
            detector_launch,
            web_video,
        ]
    )
