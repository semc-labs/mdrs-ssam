cmake_minimum_required(VERSION 3.0.2)
project(ssam_control)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs controller_manager)

catkin_package()

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_control_node src/test_control_node.cpp)
target_link_libraries(test_control_node ${catkin_LIBRARIES})

add_executable(test_control_node2 src/test_control_node2.cpp)
target_link_libraries(test_control_node2 ${catkin_LIBRARIES})

add_executable(scout_control src/scout_control.cpp)
target_link_libraries(scout_control ${catkin_LIBRARIES})
