cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-std=c++11)

include_directories(
  ${ROS_LIB_DIR}
  "../include"
  )

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)

#print_board_list()

generate_arduino_firmware(wheel_controller
  SRCS wheel_controller.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM0
)