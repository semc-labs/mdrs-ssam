#!/bin/bash

rosdep install --from-paths src --ignore-src -r -y
catkin_make

currentDir=$(pwd)

if ! grep -Fxq "source $currentDir/devel/setup.bash" ~/.bashrc
then
    echo "source $currentDir/devel/setup.bash" >> ~/.bashrc
    exec bash
fi