from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package = 'accel_decel',
        executable = 'accel_decel'
    )

    ld.add_action(node)
    return ld