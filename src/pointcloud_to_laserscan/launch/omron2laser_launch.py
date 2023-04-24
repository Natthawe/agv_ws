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
                'min_height': -0.2,
                'max_height': 0.3,
                'angle_min': -1.5708, #-0.75921822462, # -M_PI/2
                'angle_max': 1.5708, #0.75921822462, # M_PI/2
                'angle_increment': 0.0087, #0.01745329238474369,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.5,
                'range_max': 2.8,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])