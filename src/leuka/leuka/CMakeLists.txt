cmake_minimum_required(VERSION 2.8.3)
project(leuka)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_python
  rosserial_client
  std_msgs
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES leuka
  CATKIN_DEPENDS rosserial_arduino rosserial_client std_msgs roscpp rosserial_python rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ROS_LIB_DIR}
)
add_executable(talker servo_pub.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} ${ROS_LIB_DIR})

#add_executable(listener servo.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES} ${ROS_LIB_DIR})

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
 )

rosserial_configure_client(
   DIRECTORY firmware
   TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)


rosserial_add_client_target(firmware lservo ALL)
rosserial_add_client_target(firmware lservo-upload)

