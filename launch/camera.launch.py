from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    driver_arg = DeclareLaunchArgument(
        "driver",
        default_value="libcamera",
        description="Deprecated compatibility argument. Only libcamera (Pi CSI camera) is supported.",
    )
    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value="/dev/video0",
        description="Deprecated compatibility argument. Ignored in libcamera mode.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Optional explicit camera params yaml. Leave empty to use driver defaults.",
    )

    def launch_setup(context, *_args, **_kwargs):
        driver = LaunchConfiguration("driver").perform(context).strip().lower()
        params_override = LaunchConfiguration("params_file").perform(context).strip()

        params_file = (
            params_override
            or PathJoinSubstitution(
                [FindPackageShare(package_name), "config", "camera_libcamera_params.yaml"]
            ).perform(context)
        )
        actions = []
        if driver != "libcamera":
            actions.append(
                LogInfo(
                    msg=[
                        "camera.launch.py: driver='",
                        driver,
                        "' requested, but only libcamera is supported. Using libcamera.",
                    ]
                )
            )
        actions.append(
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",
                remappings=[
                    ("/image_raw", "/camera/image_raw"),
                    ("/camera_info", "/camera/camera_info"),
                ],
                parameters=[params_file],
            )
        )
        return actions

    return LaunchDescription(
        [
            driver_arg,
            video_device_arg,
            params_file_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
