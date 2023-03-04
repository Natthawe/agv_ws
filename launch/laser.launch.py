import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    # omron_b5l_a
    omron_b5l_a_node = Node(
        package='omron_b5l_a',
        executable='omron_b5l_a',
        output='both',
        name='omron_b5l_a_node'
    )

    # pointcloud_to_laserscan
    pointcloud_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    launch_pointcloud_to_laserscan_pkg = os.path.join(pointcloud_to_laserscan_dir, 'launch')

    # # static tf node
    static_tf_dir = get_package_share_directory('static_broadcaster')
    launch_static_tf_pkg = os.path.join(static_tf_dir, 'launch')


    return LaunchDescription([

    # omron_b5l_a
        omron_b5l_a_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=omron_b5l_a_node,
                on_exit=[
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),

    # pointcloud_to_laserscan        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_pointcloud_to_laserscan_pkg, 'omron2laser_launch.py'))
        ),

    # tf_static        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_static_tf_pkg, 'tf_static.launch.py'))
        ),
        
    ])