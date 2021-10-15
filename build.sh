#!/bin/bash

currentDir=$(pwd)
rosDistro=$(./ros_distro.sh)

source /opt/ros/$rosDistro/setup.bash

rosdep install --from-paths src --ignore-src -r -y
catkin_make $@