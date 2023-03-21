# Getting Started
    1) Clone this project to your colcon workspace src folder.
    2) Build the package: ```colcon build --symlink-install```

## Running the Package

##### Run All Sensors : Terminal1
    ros2 launch launch/start.launch.py

##### Run Navigation2 : Terminal2
    
    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=~/agv_ws/src/nav2_bringup/maps/map_1.yaml>   

## All Packages
- accel_decel
- agv_bot_description
- bno055
- cartographer_ros
- csm
- joy_tester
- nav2_bringup
- odom_wheel
- omron_b5l_a
- pointcloud_to_laserscan
- robot_localization
- ros2_laser_scan_matcher
- rplidar_ros
- rslidar_msg
- rslidar_sdk
- sam_bot_description
- slam_toolbox
- static_broadcaster
- teb_local_planner

# Create Packages Cpp
    ros2 pkg create <package_name> --build-type ament_cmake --dependencies rclcpp
    ros2 pkg create static_broadcaster --build-type ament_cmake --dependencies rclcpp

# Create Packages Python
    ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_python
    ros2 pkg create static_broadcaster --dependencies rclpy --build-type ament_python

#### ROSDEP
    rosdep install -r -y --from-path src
    rosdep install --from-paths src -y --ignore-src
    rosdep install -q -y -r --from-paths src --ignore-src

#### COLCON BUILD
    colcon build
    colcon build --symlink-install
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    colcon build --packages-select <name-of-pkg>
    colcon build --packages-select static_broadcaster
    
# Teleop_Twist_Keyboard
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel/remap


#### set up the API and then set the initial pose of the robot directly from code.
    1) sudo apt install ros-humble-nav2-simple-commander
    2) sudo apt install ros-humble-tf-transformations
    3) sudo apt install python3-transforms3d
    4) create nav2_poses.py & chmod +x nav2_poses.py
    5) ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    6) ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/natthawe/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml
    7) Run scripts ./nav2_poses.py

#### No GUI
    1) ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/home/natthawe/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml
    2) Run scripts ./nav2_poses.py

# URDF
#### Dependencies
    sudo apt install ros-humble-urdf-tutorial
    sudo apt install ros-humble-joint-state-publisher-gui
    sudo apt install ros-humble-xacro
    ros2 pkg create --build-type ament_cmake agv_bot_description
