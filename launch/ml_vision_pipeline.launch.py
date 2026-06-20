from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    hardware_mode = LaunchConfiguration("hardware_mode", default="real")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    launch_camera = LaunchConfiguration("launch_camera", default="true")
    video_device = LaunchConfiguration("video_device", default="/dev/video0")
    camera_driver = LaunchConfiguration("camera_driver", default="libcamera")
    camera_params = LaunchConfiguration(
        "camera_params",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "camera_libcamera_params.yaml"]),
    )

    perception_params_sim = LaunchConfiguration(
        "perception_params_sim",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "perception_params_sim.yaml"]),
    )
    perception_params_real = LaunchConfiguration(
        "perception_params_real",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "perception_params_real.yaml"]),
    )
    detector_params_ml_sim = LaunchConfiguration(
        "detector_params_ml_sim",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "detector_params_ml_sim.yaml"]),
    )
    detector_params_ml_real = LaunchConfiguration(
        "detector_params_ml_real",
        default=PathJoinSubstitution([FindPackageShare(package_name), "config", "detector_params_ml_real.yaml"]),
    )
    object_memory_sim = LaunchConfiguration(
        "object_memory_sim",
        default=PathJoinSubstitution([FindPackageShare(package_name), "assets", "memory", "object_memory_sim.json"]),
    )
    object_memory_real = LaunchConfiguration(
        "object_memory_real",
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), "assets", "memory", "object_memory_real_template.json"]
        ),
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
        condition=IfCondition(
            PythonExpression(
                ["'", launch_camera, "' == 'true' and '", hardware_mode, "' == 'real'"]
            )
        ),
    )

    perception_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "perception.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": perception_params_sim,
            "enable_detector": "true",
            "detector_params_file": detector_params_ml_sim,
            "enable_marker_detector": "false",
            "object_memory_file": object_memory_sim,
        }.items(),
        condition=IfCondition(PythonExpression(["'", hardware_mode, "' == 'sim'"])),
    )

    perception_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "perception.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": perception_params_real,
            "enable_detector": "true",
            "detector_params_file": detector_params_ml_real,
            "enable_marker_detector": "false",
            "object_memory_file": object_memory_real,
        }.items(),
        condition=IfCondition(PythonExpression(["'", hardware_mode, "' == 'real'"])),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_mode", default_value="real"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_driver", default_value="libcamera"),
            DeclareLaunchArgument("camera_params", default_value=camera_params),
            DeclareLaunchArgument("perception_params_sim", default_value=perception_params_sim),
            DeclareLaunchArgument("perception_params_real", default_value=perception_params_real),
            DeclareLaunchArgument("detector_params_ml_sim", default_value=detector_params_ml_sim),
            DeclareLaunchArgument("detector_params_ml_real", default_value=detector_params_ml_real),
            DeclareLaunchArgument("object_memory_sim", default_value=object_memory_sim),
            DeclareLaunchArgument("object_memory_real", default_value=object_memory_real),
            camera_launch,
            perception_sim,
            perception_real,
        ]
    )
