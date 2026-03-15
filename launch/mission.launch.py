from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    map_file = LaunchConfiguration("map")
    world = LaunchConfiguration(
        "world",
        default=PathJoinSubstitution(
            [FindPackageShare(package_name), "assets", "worlds", "home_assistant_room.sdf"]
        ),
    )
    waypoint_file = LaunchConfiguration("waypoint_file", default="sim_waypoints")
    mission_start_delay = LaunchConfiguration("mission_start_delay", default="15.0")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    workflow_use_rviz = LaunchConfiguration("workflow_use_rviz", default="true")
    use_gz_gui = LaunchConfiguration("use_gz_gui", default="true")
    auto_start_waypoints = LaunchConfiguration("auto_start_waypoints", default="true")

    workflow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package_name), "launch", "workflow.launch.py"]
            )
        ),
        launch_arguments={
            "mode": "navigation",
            "map": map_file,
            "world": world,
            "use_sim_time": use_sim_time,
            "workflow_use_rviz": workflow_use_rviz,
            "use_gz_gui": use_gz_gui,
        }.items(),
    )

    waypoint_runner = TimerAction(
        period=mission_start_delay,
        actions=[
            Node(
                package=package_name,
                executable="xy_waypoint_follower.py",
                name="xy_waypoint_follower",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["--file", waypoint_file],
            )
        ],
        condition=IfCondition(auto_start_waypoints),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Absolute path to map yaml, e.g. /home/user/ros2_ws/src/articubot_one/assets/maps/home_map_v1.yaml",
            ),
            DeclareLaunchArgument("world", default_value=world),
            DeclareLaunchArgument("waypoint_file", default_value="sim_waypoints"),
            DeclareLaunchArgument("mission_start_delay", default_value="15.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("workflow_use_rviz", default_value="true"),
            DeclareLaunchArgument("use_gz_gui", default_value="true"),
            DeclareLaunchArgument("auto_start_waypoints", default_value="true"),
            workflow,
            waypoint_runner,
        ]
    )
