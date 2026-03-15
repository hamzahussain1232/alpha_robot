from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    package_name = 'articubot_one'

    # ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default='')
    localizer_type = LaunchConfiguration('localizer_type', default='slam_toolbox')

    # PATHS TO INCLUDED FILES
    # Based on your screenshot, these are inside 'launch/localizers/'
    
    # 1. SLAM TOOLBOX
    # (Note: In your screenshot it is named 'slam_toolbox.launch.py', not 'online_async...')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers', 'slam_toolbox.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(EqualsSubstitution(localizer_type, 'slam_toolbox'))
    )

    # 2. CARTOGRAPHER
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers', 'cartographer.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 
                          'resolution': '0.05',
                          'publish_period_sec': '1.0'}.items(),
        condition=IfCondition(EqualsSubstitution(localizer_type, 'cartographer'))
    )

    # 3. AMCL
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers', 'amcl.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'map': map_file}.items(),
        condition=IfCondition(EqualsSubstitution(localizer_type, 'amcl'))
    )

    # 4. MAP SERVER
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'localizers', 'map_server.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'map': map_file}.items(),
        condition=IfCondition(EqualsSubstitution(localizer_type, 'map_server'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        DeclareLaunchArgument(
            'localizer_type',
            default_value='slam_toolbox',
            choices=['slam_toolbox', 'cartographer', 'amcl', 'map_server'],
            description='Choose: slam_toolbox, cartographer, amcl, or map_server'
        ),

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map.yaml (required for amcl and map_server)'
        ),

        LogInfo(msg=['============ STARTING LOCALIZER ============']),
        LogInfo(msg=['Type: ', localizer_type]),

        slam_toolbox,
        cartographer,
        amcl,
        map_server
    ])
