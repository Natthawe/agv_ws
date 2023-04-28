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

    # rviz2
    # rviz2_dir = get_package_share_directory('nav2_bringup')
    # launch_rviz2_pkg = os.path.join(rviz2_dir, 'launch')

    # bno055
    bno055_dir = get_package_share_directory('bno055')
    launch_bno055_pkg = os.path.join(bno055_dir, 'launch')

    # robot localization node
    robot_localization_dir = get_package_share_directory('robot_localization')
    launch_robot_localization_pkg = os.path.join(robot_localization_dir, 'launch')

    # static tf node
    static_tf_dir = get_package_share_directory('static_broadcaster')
    launch_static_tf_pkg = os.path.join(static_tf_dir, 'launch')

    # rplidar_ros node
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    launch_rplidar_ros_pkg = os.path.join(rplidar_ros_dir, 'launch')    

    # moteus_drive
    launch_moteus_drive_dir = get_package_share_directory('moteus_drive')
    launch_moteus_drive_pkg = os.path.join(launch_moteus_drive_dir, 'launch')

    # moteus_vel
    moteus_vel_dir = get_package_share_directory('moteus_vel')
    launch_moteus_vel_pkg = os.path.join(moteus_vel_dir, 'launch')

    # joy_tester
    # joy_tester_dir = get_package_share_directory('joy_tester')
    # launch_joy_tester_pkg = os.path.join(joy_tester_dir, 'launch')

    # accel_decel
    accel_decel_dir = get_package_share_directory('accel_decel')
    launch_accel_decel_pkg = os.path.join(accel_decel_dir, 'launch')      


    return LaunchDescription([

    # rviz2       
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_rviz2_pkg, 'rviz2_launch.py'))
        # ),

    # bno055       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_bno055_pkg, 'bno055.launch.py'))
        ),

    # robot_localization
        IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(launch_robot_localization_pkg, 'dual_ekf_navsat.launch.py'))
            PythonLaunchDescriptionSource(os.path.join(launch_robot_localization_pkg, 'ekf_moteus.launch.py'))
        ),


    # tf_static        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_static_tf_pkg, 'moteus_tf_static.launch.py'))
        ),

    # rplidar_ros        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_rplidar_ros_pkg, 'rplidar_a3.launch.py'))
        ),

    # moteus_drive       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_moteus_drive_pkg, 'moteus_node.launch.py'))
        ),

    # moteus_vel        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_moteus_vel_pkg, 'moteus_vel_node.launch.py'))
        ),

    # joy_tester
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_joy_tester_pkg, 'joy_stick.launch.py'))
        # ),

    # accel_decel        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_accel_decel_pkg, 'accel_decel_node.launch.py'))
        ),        
        
    ])