cmake_minimum_required(VERSION 3.0.2)
project(ssam_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  gazebo_plugins
  std_msgs
  geometry_msgs
)

# Depending on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
   CATKIN_DEPENDS 
   roscpp 
   rospy 
   std_msgs 
   geometry_msgs
   gazebo_plugins
   gazebo_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(map_odometry_linker src/gazebo/map_odometry_linker.cpp)
target_link_libraries( map_odometry_linker ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} )

add_library(localization_ground_truth src/gazebo/localization_ground_truth.cpp)
target_link_libraries( localization_ground_truth ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} )

# Gazebo set flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
