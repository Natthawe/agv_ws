import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a launch description and populate
    ld = LaunchDescription()
    
    # Get the launch directory and create a path to the config file
    config = os.path.join(
        get_package_share_directory('scurve_twist'), 
        'config', 'scurve_twist.yaml'
    )
    
    node = Node(
        package='scurve_twist',
        name='scurve_twist',
        executable='scurve_twist_node',
        parameters=[config]
    )
    
    ld.add_action(node)
    return ld