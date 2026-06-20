from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'articubot_one'

    default_params_file = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'arm_nano_real.yaml']
    )
    default_calibration_file = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'arm_nano_calibration.yaml']
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rsp = LaunchConfiguration('start_rsp')
    params_file = LaunchConfiguration('params_file')
    calibration_file = LaunchConfiguration('calibration_file')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    serial_boot_wait_sec = LaunchConfiguration('serial_boot_wait_sec')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'include_arm': 'true'}.items(),
        condition=IfCondition(start_rsp),
    )

    arm_driver = Node(
        package=package_name,
        executable='nano_arm_driver.py',
        name='nano_arm_driver',
        output='screen',
        parameters=[params_file, calibration_file, {
            'use_sim_time': use_sim_time,
            'port': port,
            'baud': baud,
            'serial_boot_wait_sec': serial_boot_wait_sec,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rsp', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('calibration_file', default_value=default_calibration_file),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
        ),
        DeclareLaunchArgument('baud', default_value='9600'),
        DeclareLaunchArgument('serial_boot_wait_sec', default_value='2.0'),
        LogInfo(msg=['============ STARTING REAL ARM (NANO SERIAL) ============']),
        rsp,
        arm_driver,
    ])
