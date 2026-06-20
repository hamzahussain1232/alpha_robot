from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    video_device = LaunchConfiguration("video_device", default="/dev/video0")
    camera_driver = LaunchConfiguration("camera_driver", default="libcamera")
    camera_params = LaunchConfiguration(
        "camera_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "camera_libcamera_params.yaml"]),
    )

    use_mic = LaunchConfiguration("use_mic", default="true")
    voice_params = LaunchConfiguration(
        "voice_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "voice_params.yaml"]),
    )

    enable_perception = LaunchConfiguration("enable_perception", default="true")
    perception_params = LaunchConfiguration(
        "perception_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "perception_params_real.yaml"]),
    )
    enable_detector = LaunchConfiguration("enable_detector", default="false")
    detector_params = LaunchConfiguration(
        "detector_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "detector_params_real.yaml"]),
    )
    object_memory_file = LaunchConfiguration(
        "object_memory_file",
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), "assets", "memory", "object_memory_real_template.json"]
        ),
    )

    enable_task_manager = LaunchConfiguration("enable_task_manager", default="false")
    task_params = LaunchConfiguration(
        "task_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "task_params_real.yaml"]),
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
    )

    voice_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "voice_control.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_mic": use_mic,
            "voice_params": voice_params,
        }.items(),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "perception.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": perception_params,
            "enable_detector": enable_detector,
            "detector_params_file": detector_params,
            "enable_marker_detector": "false",
            "object_memory_file": object_memory_file,
        }.items(),
        condition=IfCondition(enable_perception),
    )

    task_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "task_manager.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "params_file": task_params}.items(),
        condition=IfCondition(enable_task_manager),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_driver", default_value="libcamera"),
            DeclareLaunchArgument("camera_params", default_value=camera_params),
            DeclareLaunchArgument("use_mic", default_value="true"),
            DeclareLaunchArgument("voice_params", default_value=voice_params),
            DeclareLaunchArgument("enable_perception", default_value="true"),
            DeclareLaunchArgument("perception_params", default_value=perception_params),
            DeclareLaunchArgument("enable_detector", default_value="false"),
            DeclareLaunchArgument("detector_params", default_value=detector_params),
            DeclareLaunchArgument("object_memory_file", default_value=object_memory_file),
            DeclareLaunchArgument("enable_task_manager", default_value="false"),
            DeclareLaunchArgument("task_params", default_value=task_params),
            camera_launch,
            voice_launch,
            perception_launch,
            task_manager_launch,
        ]
    )
