### MDRS-SSAM

This is the software github repo for the Nexus Aurora MDRS-SSAM project.
It is a ROS+Gazebo workspace so you will need to install ROS and Gazebo.
Follow the instructions here:
http://wiki.ros.org/noetic/Installation/Ubuntu
(currently we are using the noetic version of ROS but we are open to new suggestions)

For ROS tutorial it is advised that you follow the official ones from the ROS website:
http://wiki.ros.org/ROS/Tutorials

##### Additional Required Packages
* [gmapping](http://wiki.ros.org/gmapping)
```bash
sudo apt-get install ros-noetic-gmapping
```
* [map_server](http://wiki.ros.org/map_server)
```bash
sudo apt-get install ros-noetic-map-server
```
* [move_base](http://wiki.ros.org/move_base)
```bash
sudo apt-get install ros-noetic-move-base
```
* [global_planner](http://wiki.ros.org/global_planner)
```bash
sudo apt-get install ros-noetic-global-planner
```
* [teb_local_planner](http://wiki.ros.org/teb_local_planner)
```bash
sudo apt-get install ros-noetic-teb-local-planner
```
* [controller_manager](http://wiki.ros.org/controller_manager)
```bash
sudo apt-get install ros-noetic-controller-manager
```
* [ros_control](http://wiki.ros.org/ros_control)
```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

##### Packages
The project right now contains 4 packages:
* ssam_core
* ssam_control
* ssam_navigation
* ssam_simulation

Note: Since all packages are common inside ROS along with the ones externally provided, it is recommended any newly
created packages you create to have the prefix ssam_.

After downloading the project, since this is a ROS catkin workspace, you should run "source devel/setup.bash" to get 
everything indexed properly in your terminal. Note that you will need to do this for every new terminal you open.
I also suggest you use Terminator or some other terminal which allow terminal splitting as you will be opening a lot
of different terminals while working with ROS.

##### SSAM Core
This project contains the robot URDF model, some useful xacro parameters and macros, as well as the main launch file for the robot functionality.
This launch file is the one to be used when launching the project on the actual hardware.
To start it, run "roslaunch ssam_core scout_startup"

##### SSAM Control
This package will contain all needed functionality to control the robot.
Nodes:
* test_control_node : moves the robot forward for 2 seconds, then backwards for 2 seconds. This node only publishes on
the (/cmd_vel) topic. You can start it with "rosrun ssam_control test_control_node"
* test_control_node2: This subscribes to the /odom topic to get info about the robot's position. If the robot's x
coordinate is smaller than 2 it will tell the robot to move forward. If the x coordinate is greater than 5 it will tell
the robot to move forward (using the /cmd_vel topic). You can start it with "rosrun ssam_control test_control_node2"
* scout_control: Runs the robot's hardware interface which will forward instructions from various controllers (right now only diff_drive_controller) to the actual hardware.
This node does not run in simulation mode

##### SSAM Navigation
Mainly contains configuration files for a move_base node which takes in sensor data and a goal command and outputs cmd_vel commands to the diff_drive_controller.
The launch file also launches a stereo_image_proc node (not currently used outside simulation) and a fake_localization node to complete the tf_tree when running on the actual hardware. This node will be removed.

##### SSAM Simulation
This package contains the gazebo world (/worlds), the robot model (/models/scout) and the launch file (/launch).
To start it run "roslaunch ssam_simulation scout_startup.launch". 

You should see gazebo startup, as well as rviz. You can use rviz to visualize various info about the robot, as well as issue goal commands for move_base.
Optionally you can add the argument rviz:=false if you don't wish to launch rviz.

###### Multiple Scouts
To launch multiple scouts, use the argument scout_count:=<value> to set how many scouts you want to spawn (they will be spawn 1 meter apart on the Y axis).

Note: Right now the scout uses world position and rotation provided by gazebo to navigate. This will be changed in the future.
It also uses a laser for scanning purposes. This will also be changed.

If you start the simulation and a control node you should see the robot moving forward and backwards in the gazebo
simulation.
If you use multiple scouts in the simulation you need to use the _scout_name=<string_value> to tell the node which scout
to control. For example "rosrun ssam_control test_control_node2 _scout_name:=scout3" will make the 3rd scout from the
simulation to move forward and backwards. You can spawn multiple test_control_nodes to control multiple scouts.