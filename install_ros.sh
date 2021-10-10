#!/bin/bash

rosDistro=$(./ros_distro.sh)

if [[ $rosDistro == "noetic" ]] ;
then
    pythonVer="python3"
fi

if [[ $rosDistro == "melodic" ]] ;
then
    pythonVer="python"
fi

echo "Using python version - $pythonVer"
echo "Preparing to install ros-$rosDistro-desktop-full"

sleep 5

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-$rosDistro-desktop-full -y
source /opt/ros/$rosDistro/setup.bash
sudo apt install $pythonVer-rosdep $pythonVer-rosinstall $pythonVer-rosinstall-generator $pythonVer-wstool build-essential -y
sudo rosdep init
rosdep update

if ! grep -Fxq "source /opt/ros/$rosDistro/setup.bash" ~/.bashrc
then
    echo "source /opt/ros/$rosDistro/setup.bash" >> ~/.bashrc
    exec bash
fi