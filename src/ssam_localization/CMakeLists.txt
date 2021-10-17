cmake_minimum_required(VERSION 3.0.2)
project(ssam_localization)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs nav_msgs tf)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(localization_simulator src/localization_simulator.cpp)
target_link_libraries(localization_simulator ${catkin_LIBRARIES})
