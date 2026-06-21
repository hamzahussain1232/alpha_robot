from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'articubot_one'

    def launch_setup(context, *args, **kwargs):
        use_sim_time_raw = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
        use_sim_time = use_sim_time_raw in ('1', 'true', 'yes', 'on')
        show_gui = LaunchConfiguration('show_gui').perform(context)
        mode = LaunchConfiguration('mode').perform(context)
        rviz_config_override = LaunchConfiguration('rviz_config').perform(context).strip()

        if rviz_config_override:
            rviz_config_file = rviz_config_override
        elif mode in ('navigation', 'amcl'):
            rviz_config_file = PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'main.rviz']
            ).perform(context)
        elif mode == 'mapping':
            rviz_config_file = PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'mapping_laptop_compressed.rviz']
            ).perform(context)
        elif mode == 'perception':
            rviz_config_file = PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'perception_laptop.rviz']
            ).perform(context)
        elif mode == 'camera':
            rviz_config_file = PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'camera_view.rviz']
            ).perform(context)
        elif mode == 'camera_safe':
            rviz_config_file = PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'camera_safe.rviz']
            ).perform(context)
        else:
            rviz_config_file = PathJoinSubstitution(
                [FindPackageShare(package_name), 'config', 'map.rviz']
            ).perform(context)

        actions = [
            LogInfo(msg=[f'============ STARTING RVIZ VISUALIZATION ({mode}) ============']),
        ]

        if show_gui == 'true':
            actions.append(
                Node(
                    package='joint_state_publisher_gui',
                    executable='joint_state_publisher_gui',
                    output='screen',
                )
            )

        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        )

        return actions

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'show_gui',
            default_value='false',
            description='Show Joint State Publisher GUI to move arm manually'),
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            choices=['drive', 'mapping', 'amcl', 'navigation', 'perception', 'camera', 'camera_safe'],
            description='Select the RViz preset that matches the robot workflow'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional explicit RViz config file. Leave empty for auto selection by mode'),
        OpaqueFunction(function=launch_setup),
    ])
