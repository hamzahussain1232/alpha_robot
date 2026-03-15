import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name='articubot_one'

    # ARGS
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # FIXED PATH: Point directly to the 'config' folder
    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare(package_name), 'config'
    ])

    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.25')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
            
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        LogInfo(msg=['============ STARTING CARTOGRAPHER ============']),
        LogInfo(msg=['Config Dir: ', cartographer_config_dir]),
        LogInfo(msg=['Config File: ', configuration_basename]),

        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                # Remap the laser scan topic if necessary
                # ('scan', 'scan'), 
                # Remap odom to the output of your EKF filter
                ('odom', 'odometry/local'),
                # ('imu', 'imu/data'),
            ]
        ),

        # Occupancy Grid Node (Converts map to standard ROS format)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution,
                       '-publish_period_sec', publish_period_sec]
        )
    ])
