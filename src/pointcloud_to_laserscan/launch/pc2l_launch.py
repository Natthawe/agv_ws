from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/center/rslidar_points'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'laser',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 1.0,
                'angle_min': -3.14,  # -M_PI/2
                'angle_max': 3.14,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.2,
                'range_max': 25.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
