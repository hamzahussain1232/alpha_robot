from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "articubot_one"
    namespace = LaunchConfiguration("namespace", default="")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # FIXED PATHS: Point to 'config' folder, ignoring 'robot_model'
    ekf_odom_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), "config", "ekf.yaml"
    ])
    
    # You would need to create these 2 files in config/ for this to work:
    ekf_navsat_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), "config", "ekf_navsat_params.yaml"
    ])
    navsat_transform_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), "config", "navsat_transform.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        LogInfo(msg=["============ STARTING GPS LOCALIZATION (Dual EKF) ============"]),

        # Node 1: Local EKF (Odom + IMU)
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            namespace=namespace,
            parameters=[ekf_odom_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "odometry/local")],
            output="screen"
        ),

        # Node 2: Global EKF (Odom + IMU + GPS)
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_navsat",
            namespace=namespace,
            parameters=[ekf_navsat_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "odometry/global")],
            output="screen"
        ),

        # Node 3: GPS Transform
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            namespace=namespace,
            parameters=[navsat_transform_params_file, {"use_sim_time": use_sim_time}],
            remappings=[
                ('imu', 'imu/data'),
                ('gps/fix', 'gps/fix'),
                ('odometry/filtered', 'odometry/global'),
                ("odometry/gps", "odometry/gps"),
            ],
            output="screen"
        ),
    ])