# Getting Started
    1) Clone this project to your colcon workspace src folder.
    2) Build the package: ```colcon build --symlink-install```

## Running the Package
======================
run all sensors
    
    Terminal1: ```ros2 launch launch/start.launch.py```

run navigation2
    
    Terminal2: ```ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=~/agv_ws/src/nav2_bringup/maps/map_1.yaml> ```    


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
    



## All Packages
- accel_decel
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
