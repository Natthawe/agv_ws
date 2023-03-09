# Create Packages cpp
    ros2 pkg create <package_name> --build-type ament_cmake --dependencies rclcpp
    ros2 pkg create static_broadcaster --build-type ament_cmake --dependencies rclcpp

# Create Packages python
    ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_python
    ros2 pkg create static_broadcaster --dependencies rclpy --build-type ament_python

#### rosdep
    rosdep install -r -y --from-path src
    rosdep install --from-paths src -y --ignore-src
    rosdep install -q -y -r --from-paths src --ignore-src

#### colcon build
    colcon build
    colcon build --symlink-install
    colcon build --packages-select <name-of-pkg>
    colcon build --packages-select static_broadcaster

## add packages
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
