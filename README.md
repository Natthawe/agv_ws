# Getting Started
1) Install [Ubuntu 22.04](https://ubuntu.com/download/desktop)
2) Setting [Remote Desktop](https://linuxhint.com/enable-remote-desktop-ubuntu-access-windows/) then remove password.
3) Install [ROS HUMBLE](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
4) Clone this project to your colcon workspace src folder.
5) Run `rosdep install -r -y --from-path src`
6) Build the package: `colcon build`
7) Ff error Click [Dependencies](#Dependencies)
8) In .bashrc file add command below file: 
    `export ROS_DOMAIN_ID=102` 
    `source /opt/ros/humble/setup.bash`
    `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

## Running the Package

##### Run All Sensors : Terminal1
    ros2 launch launch/start.launch.py

##### Run Navigation2 : Terminal2
    
    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=~/agv_ws/src/nav2_bringup/maps/map_1.yaml   

## All Packages
- accel_decel
- agv_bot_description
- bno055
- cartographer_ros
- costmap_converter
- csm
- joy_tester
- navigation2
- odom_wheel
- omron_b5l_a
- omron_b5l_tof
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
    ros2 pkg create --build-type ament_cmake agv_bot_description

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

# Dependencies
    sudo apt install ros-humble-urdf-tutorial
    sudo apt install ros-humble-joint-state-publisher-gui
    sudo apt install ros-humble-xacro
    sudo apt install openssh-server
    sudo apt install libyaml-cpp-dev
    sudo apt install libpcap-dev
    sudo apt install python3-pip
    pip install setuptools==58.2.0
    sudo apt install ros-humble-rmw-cyclonedds-cpp
    

# SLAM
#### ALL Sensors
    ros2 launch launch/start.launch.py

#### cartographer_ros
    ros2 launch cartographer_ros wc_2d.launch.py

#### save map
    ros2 run nav2_map_server map_saver_cli -f <map_name>

# UDEV   
#### 50-bno055.rules
    #KERNEL=="ttyUSB*", KERNELS=="1-6.1", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", SYMLINK+="bno055"

    ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", RUN+="/sbin/modprobe usbserial vendor=0x0403 product=0x6001", MODE="0666", GROUP="dialout"

#### 50-omron.rules
    #SUBSYSTEMS=="tty", KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="0590", ATTRS{idProduct}=="00ca", GROUP="dialout", MODE="0666"

    #KERNEL=="ttyUSB*", KERNELS=="1-5", ATTRS{idVendor}=="0590", ATTRS{idProduct}=="00ca", MODE:="0666", SYMLINK+="omron"

    ACTION=="add", ATTRS{idVendor}=="0590", ATTRS{idProduct}=="00ca", RUN+="/sbin/modprobe usbserial vendor=0x0590 product=0x00ca", MODE="0666", GROUP="dialout"    

#### 50-teensy.rules
    #KERNEL=="ttyACM*", KERNELS=="1-6.2", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", MODE:="0666", SYMLINK+="teensy"

    ACTION=="add", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", RUN+="/sbin/modprobe usbserial vendor=0x16c0 product=0x0483", MODE="0666", GROUP="dialout"

#### 50-rplidar.rules
    #KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"

    ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", RUN+="/sbin/modprobe usbserial vendor=0x10c4 product=0xea60", MODE="0666", GROUP="dialout"    

#### 50-stm32.rules
    ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", RUN+="/sbin/modprobe usbserial vendor=0x0483 product=0x374b", MODE="0666", GROUP="dialout"

#### reload udev
    sudo udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB1)
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=tty 


wheelchair001@wheelchair001:~/ros2_bridge$  `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
wheelchair001@wheelchair001:~/ros_web_interface$  `npm start`
http://10.1.10.146:3000/

# Using tf2_echo
#### `tf2_echo` reports the transform between any two frames broadcasted over ROS.
    ros2 run tf2_ros tf2_echo base_link laser

## Uninstall ROS & setuptools
    sudo apt remove ros-* or sudo apt remove ~nros-humble-* && sudo apt autoremove
    sudo rm /etc/apt/sources.list.d/ros2.list
    pip uninstall setuptools
    sudo apt update
    sudo apt autoremove
    sudo apt upgrade
    sudo reboot
