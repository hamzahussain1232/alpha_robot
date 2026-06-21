from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    launch_camera = LaunchConfiguration("launch_camera", default="true")
    launch_detector = LaunchConfiguration("launch_detector", default="true")
    camera_driver = LaunchConfiguration("camera_driver", default="libcamera")
    video_device = LaunchConfiguration("video_device", default="/dev/video0")
    camera_params = LaunchConfiguration(
        "camera_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "camera_libcamera_params.yaml"]),
    )
    detector_params = LaunchConfiguration(
        "detector_params",
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), "config", "detector_params_bootstrap_real.yaml"]
        ),
    )
    web_host = LaunchConfiguration("web_host", default="0.0.0.0")
    web_port = LaunchConfiguration("web_port", default="8091")
    camera_topic = LaunchConfiguration(
        "camera_topic",
        default="/perception/annotated_image/compressed",
    )
    detections_topic = LaunchConfiguration("detections_topic", default="/perception/detections")

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
        condition=IfCondition(launch_detector),
    )

    web_node = Node(
        package=package_name,
        executable="camera_detection_web.py",
        name="camera_detection_web",
        output="screen",
        parameters=[
            {
                "host": web_host,
                "port": web_port,
                "camera_topic": camera_topic,
                "detections_topic": detections_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("launch_detector", default_value="true"),
            DeclareLaunchArgument("camera_driver", default_value="libcamera"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_params", default_value=camera_params),
            DeclareLaunchArgument("detector_params", default_value=detector_params),
            DeclareLaunchArgument("web_host", default_value=web_host),
            DeclareLaunchArgument("web_port", default_value=web_port),
            DeclareLaunchArgument("camera_topic", default_value=camera_topic),
            DeclareLaunchArgument("detections_topic", default_value=detections_topic),
            camera_launch,
            detector_launch,
            web_node,
        ]
    )
