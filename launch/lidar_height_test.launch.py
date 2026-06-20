from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    mode = LaunchConfiguration("mode", default="mapping")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    lidar_offset_x = LaunchConfiguration("lidar_offset_x", default="-0.08")
    lidar_offset_y = LaunchConfiguration("lidar_offset_y", default="0.0")
    lidar_offset_z = LaunchConfiguration("lidar_offset_z", default="0.0275")
    use_gz_gui = LaunchConfiguration("use_gz_gui", default="true")
    workflow_use_rviz = LaunchConfiguration("workflow_use_rviz", default="true")
    gz_verbosity = LaunchConfiguration("gz_verbosity", default="1")
    map_file = LaunchConfiguration("map", default="")
    mapping_localizer_type = LaunchConfiguration("mapping_localizer_type", default="slam_toolbox")
    world = LaunchConfiguration(
        "world",
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), "assets", "worlds", "lidar_height_test.sdf"]
        ),
    )

    workflow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", "workflow.launch.py"])
        ),
        launch_arguments={
            "mode": mode,
            "mapping_localizer_type": mapping_localizer_type,
            "hardware_mode": "sim",
            "use_sim_time": use_sim_time,
            "world": world,
            "map": map_file,
            "lidar_offset_x": lidar_offset_x,
            "lidar_offset_y": lidar_offset_y,
            "lidar_offset_z": lidar_offset_z,
            "workflow_use_rviz": workflow_use_rviz,
            "use_gz_gui": use_gz_gui,
            "gz_verbosity": gz_verbosity,
            "enable_voice": "false",
            "enable_perception": "false",
            "enable_task_manager": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mapping"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("lidar_offset_x", default_value="-0.08"),
            DeclareLaunchArgument("lidar_offset_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_offset_z", default_value="0.0275"),
            DeclareLaunchArgument("use_gz_gui", default_value="true"),
            DeclareLaunchArgument("workflow_use_rviz", default_value="true"),
            DeclareLaunchArgument("gz_verbosity", default_value="1"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("mapping_localizer_type", default_value="slam_toolbox"),
            DeclareLaunchArgument("world", default_value=world),
            LogInfo(msg=["=== LiDAR height test world ==="]),
            LogInfo(msg=["Tip center: lidar_offset_x:=-0.08 lidar_offset_y:=0.0"]),
            LogInfo(msg=["Tip test: lidar_offset_x:=-0.08 (rear), lidar_offset_y:=0.02 (left)"]),
            LogInfo(msg=["Tip height: lidar_offset_z 0.00, 0.015, 0.0275, 0.04"]),
            workflow,
        ]
    )
