# Create Packages cpp
    ros2 pkg create <package_name> --build-type ament_cmake --dependencies rclcpp
    ros2 pkg create static_broadcaster --build-type ament_cmake --dependencies rclcpp

# Create Packages python
    ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_python
    ros2 pkg create static_broadcaster --dependencies rclpy --build-type ament_python

#### rosdep
    rosdep install -r -y --from-path src
    rosdep install --from-paths src -y --ignore-src

#### colcon build
    colcon build
    colcon build --symlink-install
    colcon build --packages-select <name-of-pkg>
    colcon build --packages-select static_broadcaster