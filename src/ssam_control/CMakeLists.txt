cmake_minimum_required(VERSION 3.0.2)
project(ssam_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  geometry_msgs
  controller_manager
  rosserial_arduino
  rosserial_client
)

catkin_package()

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)


rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

add_executable(test_control_node src/test_control_node.cpp)
target_link_libraries(test_control_node ${catkin_LIBRARIES})

add_executable(test_control_node2 src/test_control_node2.cpp)
target_link_libraries(test_control_node2 ${catkin_LIBRARIES})

add_executable(scout_control src/scout_control.cpp)
target_link_libraries(scout_control ${catkin_LIBRARIES})

rosserial_add_client_target(firmware wheel_controller ALL)
rosserial_add_client_target(firmware wheel_controller-upload)