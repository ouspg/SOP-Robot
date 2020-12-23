cmake_minimum_required(VERSION 3.5)
project(dynamixel-controllers)

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

find_package(rclcpp REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(find_dynamixel src/find_dynamixel.cpp)
ament_target_dependencies(find_dynamixel rclcpp)
ament_target_dependencies(find_dynamixel dynamixel_workbench_msgs)
ament_target_dependencies(find_dynamixel dynamixel_workbench_toolbox)
# ament_target_dependencies(find_dynamixel sensor_msgs)

add_executable(controller
  src/controller.cpp
  src/trajectory_generator.cpp
)

ament_target_dependencies(controller
  rclcpp
  Eigen3
  yaml-cpp
  dynamixel_workbench_toolbox
  dynamixel_workbench_msgs
  sensor_msgs
  trajectory_msgs
)

target_link_libraries(controller 
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  find_dynamixel
  controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
