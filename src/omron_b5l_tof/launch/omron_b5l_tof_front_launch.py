import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Create a launch description and populate
    ld = LaunchDescription()

    # Get the launch directory and create a path to the config file
    config = os.path.join(
        get_package_share_directory('omron_b5l_tof'),
        'param', 'omron_b5l_tof_front.yaml'
    )

    node = Node(
        package='omron_b5l_tof',
        name='omron_b5l_tof',
        executable='omron_b5l_tof_front',
        parameters=[config]
    )

    ld.add_action(node)
    return ld
