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

    # rplidar_ros node
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    launch_rplidar_ros_pkg = os.path.join(rplidar_ros_dir, 'launch')    

    # odom_wheel node
    odom_wheel_dir = get_package_share_directory('odom_wheel')
    launch_odom_wheel_pkg = os.path.join(odom_wheel_dir, 'launch')    

    # accel_decel
    accel_decel_dir = get_package_share_directory('accel_decel')
    launch_accel_decel_pkg = os.path.join(accel_decel_dir, 'launch')      


    return LaunchDescription([

    # rplidar_ros        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_rplidar_ros_pkg, 'rplidar_a3.launch.py'))
        ),

    # odom_wheel        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_odom_wheel_pkg, 'odom_wheel_node.launch.py'))
        ),

    # accel_decel        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_accel_decel_pkg, 'accel_decel_node.launch.py'))
        ),        
        
    ])