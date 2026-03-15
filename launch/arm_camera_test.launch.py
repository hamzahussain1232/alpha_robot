from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    mode = LaunchConfiguration("mode", default="sim")
    world = LaunchConfiguration(
        "world",
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), "assets", "worlds", "home_assistant_room.sdf"]
        ),
    )
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_gz_gui = LaunchConfiguration("use_gz_gui", default="true")
    gz_verbosity = LaunchConfiguration("gz_verbosity", default="1")
    video_device = LaunchConfiguration("video_device", default="/dev/video0")
    camera_params = LaunchConfiguration(
        "camera_params",
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), "config", "camera_v4l2_params.yaml"]
        ),
    )
    start_arm_teleop = LaunchConfiguration("start_arm_teleop", default="false")

    sim_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "drive_sim.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": use_rviz,
            "world": world,
            "use_gz_gui": use_gz_gui,
            "gz_verbosity": gz_verbosity,
        }.items(),
    )

    hardware_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "drive.launch.py"])
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    hardware_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "camera.launch.py"])
        ),
        launch_arguments={
            "video_device": video_device,
            "params_file": camera_params,
        }.items(),
    )

    hardware_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "launch_rviz.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "false",
            "show_gui": "false",
        }.items(),
        condition=IfCondition(use_rviz),
    )

    arm_teleop = Node(
        package=package_name,
        executable="arm_teleop.py",
        output="screen",
        condition=IfCondition(start_arm_teleop),
    )

    sim_group = GroupAction(
        condition=IfCondition(EqualsSubstitution(mode, "sim")),
        actions=[sim_stack],
    )

    hardware_group = GroupAction(
        condition=IfCondition(EqualsSubstitution(mode, "hardware")),
        actions=[hardware_drive, hardware_camera, hardware_rviz],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                choices=["sim", "hardware"],
                description="Choose sim for Gazebo tests, hardware for real robot + USB camera",
            ),
            DeclareLaunchArgument("world", default_value=world),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_gz_gui", default_value="true"),
            DeclareLaunchArgument("gz_verbosity", default_value="1"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_params", default_value=camera_params),
            DeclareLaunchArgument("start_arm_teleop", default_value="false"),
            LogInfo(msg=["============ ARM+CAMERA TEST MODE: ", mode, " ============"]),
            sim_group,
            hardware_group,
            arm_teleop,
        ]
    )
