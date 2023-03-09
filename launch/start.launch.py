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

    # bno055
    bno055_dir = get_package_share_directory('bno055')
    launch_bno055_pkg = os.path.join(bno055_dir, 'launch')

    # omron_b5l_a
    omron_b5l_a_node = Node(
        package='omron_b5l_a',
        executable='omron_b5l_a',
        output='both',
        name='omron_b5l_a_node'
    )

    # rslidar_sdk
    # rslidar_sdk_dir = get_package_share_directory('rslidar_sdk')
    # launch_rslidar_sdk_pkg = os.path.join(rslidar_sdk_dir, 'launch')

    # pointcloud_to_laserscan
    pointcloud_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    launch_pointcloud_to_laserscan_pkg = os.path.join(pointcloud_to_laserscan_dir, 'launch')

    # ros2_laser_scan_matcher
    # laser_scan_matcher_node = Node(
    #     package='ros2_laser_scan_matcher',
    #     executable='laser_scan_matcher',
    #     output='both',
    #     name='laser_scan_matcher_node'
    # )

    # robot localization node
    robot_localization_dir = get_package_share_directory('robot_localization')
    launch_robot_localization_pkg = os.path.join(robot_localization_dir, 'launch')

    # static tf node
    static_tf_dir = get_package_share_directory('static_broadcaster')
    launch_static_tf_pkg = os.path.join(static_tf_dir, 'launch')

    # rplidar_ros node
    # rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    # launch_rplidar_ros_pkg = os.path.join(rplidar_ros_dir, 'launch')    

    # control drive node
    control_drive_dir = get_package_share_directory('odom_wheel')
    launch_control_drive_pkg = os.path.join(control_drive_dir, 'launch')    

    # accel_decel
    accel_decel_dir = get_package_share_directory('accel_decel')
    launch_accel_decel_pkg = os.path.join(accel_decel_dir, 'launch')      


    return LaunchDescription([

    # bno055       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_bno055_pkg, 'bno055.launch.py'))
        ),

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

    # rslidar_sdk        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_rslidar_sdk_pkg, 'start.py'))
        # ),

    # pointcloud_to_laserscan        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_pointcloud_to_laserscan_pkg, 'omron2laser_launch.py'))
        ),

    # ros2_laser_scan_matcher
        # laser_scan_matcher_node,

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=laser_scan_matcher_node,
        #         on_exit=[
        #             EmitEvent(event=Shutdown(
        #                 reason='Window closed'))
        #         ]
        #     )
        # ),

    # robot_localization
        IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(launch_robot_localization_pkg, 'dual_ekf_navsat.launch.py'))
            PythonLaunchDescriptionSource(os.path.join(launch_robot_localization_pkg, 'ekf.launch.py'))
        ),


    # tf_static        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_static_tf_pkg, 'tf_static.launch.py'))
        ),

    # rplidar_ros        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_rplidar_ros_pkg, 'rplidar_a3.launch.py'))
        # ),

    # control_node        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_control_drive_pkg, 'odom_wheel_node.launch.py'))
        ),

    # accel_decel        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_accel_decel_pkg, 'accel_decel_node.launch.py'))
        ),        
        
    ])