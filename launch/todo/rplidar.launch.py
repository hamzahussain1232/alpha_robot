import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            # Note: If 'rplidar_composition' fails, try 'rplidar_node'
            executable='rplidar_composition', 
            name='rplidar_node',
            output='screen',
            parameters=[{
                # 1. SERIAL PORT (Your specific path is great!)
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                
                # 2. BAUDRATE (CRITICAL!)
                # A1 / A2 = 115200
                # A3 / S1 = 256000
                'serial_baudrate': 115200, 

                # 3. FRAME ID (Must match your URDF link name)
                'frame_id': 'laser_frame',

                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
    ])