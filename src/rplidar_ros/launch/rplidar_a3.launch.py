from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/CP2102',
                'serial_baudrate': 256000,  # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': False,
                'scan_mode': 'Sensitivity',
            }],
        ),
    ])
