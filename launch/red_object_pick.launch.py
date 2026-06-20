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
    launch_arm = LaunchConfiguration("launch_arm")
    camera_params_file = LaunchConfiguration("camera_params_file")
    picker_params_file = LaunchConfiguration("picker_params_file")
    arm_params_file = LaunchConfiguration("arm_params_file")
    arm_calibration_file = LaunchConfiguration("arm_calibration_file")

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "camera.launch.py"])
        ),
        launch_arguments={
            "driver": camera_driver,
            "video_device": video_device,
            "params_file": camera_params_file,
        }.items(),
        condition=IfCondition(launch_camera),
    )

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "arm_nano.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": arm_params_file,
            "calibration_file": arm_calibration_file,
        }.items(),
        condition=IfCondition(launch_arm),
    )

    picker_node = Node(
        package=package_name,
        executable="red_object_picker.py",
        name="red_object_picker",
        output="screen",
        parameters=[picker_params_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_driver", default_value="libcamera"),
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("launch_arm", default_value="true"),
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare(package_name), "config", "camera_libcamera_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "picker_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare(package_name), "config", "red_object_picker_params_real.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "arm_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare(package_name), "config", "arm_nano_real.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "arm_calibration_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare(package_name), "config", "arm_nano_calibration.yaml"]
                ),
            ),
            camera_launch,
            arm_launch,
            picker_node,
        ]
    )
