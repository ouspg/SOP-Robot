cmake_minimum_required(VERSION 3.5)
project(robot_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/robot_hardware.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Use hardcoded hardware_interface path because otherwise conflicts with the moveit2 hardware_interface

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  /ros2_control_ws/install/hardware_interface/include/
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
  trajectory_msgs
  yaml-cpp
  dynamixel_workbench_toolbox
)

target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)


## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries( 
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package() 
