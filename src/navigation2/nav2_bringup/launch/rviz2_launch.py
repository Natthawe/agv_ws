import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration  
from launch import LaunchDescription

def generate_launch_description():
 
   rviz_config_file = PathJoinSubstitution(
           [FindPackageShare("nav2_bringup"), "rviz", "nav2_rplidar.rviz"]
   )
    
   return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),
   ])