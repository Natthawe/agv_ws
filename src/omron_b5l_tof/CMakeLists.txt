cmake_minimum_required(VERSION 3.8)
project(omron_b5l_tof)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

add_executable(omron_b5l_tof_front
  src/omron_b5l_tof_front.cpp
  src/b5l_tof_device.cpp
  src/uart_linux.cpp
  src/TOFApiZ.cpp
)

add_executable(omron_b5l_tof_back
  src/omron_b5l_tof_back.cpp
  src/b5l_tof_device.cpp
  src/uart_linux.cpp
  src/TOFApiZ.cpp
)

ament_target_dependencies(omron_b5l_tof_front rclcpp std_msgs pcl_conversions)
ament_target_dependencies(omron_b5l_tof_back rclcpp std_msgs pcl_conversions)


# add pthread
# Link libraries for omron_b5l_tof_front and omron_b5l_tof_back
target_link_libraries(omron_b5l_tof_front
  pthread
  ${PCL_LIBRARIES}
)

target_link_libraries(omron_b5l_tof_back
  pthread
  ${PCL_LIBRARIES}
)

install(TARGETS
  omron_b5l_tof_front
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  omron_b5l_tof_back
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
