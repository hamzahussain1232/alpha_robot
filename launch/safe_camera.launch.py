from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    launch_camera = LaunchConfiguration("launch_camera", default="true")
    launch_web_video = LaunchConfiguration("launch_web_video", default="false")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")
    camera_driver = LaunchConfiguration("camera_driver", default="libcamera")
    video_device = LaunchConfiguration("video_device", default="/dev/video0")
    camera_params = LaunchConfiguration(
        "camera_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "camera_laptop_view.yaml"]),
    )
    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "camera_safe.rviz"]),
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

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "launch_rviz.launch.py"])
        ),
        launch_arguments={
            "mode": "camera_safe",
            "rviz_config": rviz_config,
            "show_gui": "false",
        }.items(),
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("launch_web_video", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("camera_driver", default_value="libcamera"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_params", default_value=camera_params),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            camera_launch,
            rviz_launch,
        ]
    )
