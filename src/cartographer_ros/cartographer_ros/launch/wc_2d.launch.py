from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    ## ***** File paths ******
    # pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    # urdf_dir = os.path.join(pkg_share, 'urdf')
    # urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    ## ***** Nodes *****
    # robot_state_publisher_node = Node(
    #     package = 'robot_state_publisher',
    #     executable = 'robot_state_publisher',
    #     parameters=[
    #         {'robot_description': robot_desc},
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     output = 'screen'
    #     )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'wc_2d.lua'],
        remappings = [
            ('echoes', 'horizontal_laser_2d')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )

    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        # robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
