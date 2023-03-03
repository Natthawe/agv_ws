from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/pointcloud2_xyzi'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'laser',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 1.0,
                'angle_min': -0.75921822462, #-1.57,  # -M_PI/2
                'angle_max': 0.75921822462,#1.57,  # M_PI/2
                'angle_increment': 0.01745329238474369,  # M_PI/360.0
                'scan_time': 0.1338348239660263,
                'range_min': 0.5,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])