from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    nano_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'arm_nano.launch.py'])
        )
    )

    return LaunchDescription([
        LogInfo(msg=['============ STARTING REAL ARM (NANO SERIAL) ============']),
        nano_arm,
    ])
