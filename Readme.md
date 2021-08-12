## MDRS-SSAM

This is the software github repo for the Nexus Aurora MDRS-SSAM project.
It is a ROS+Gazebo workspace so you will need to install ROS and Gazebo.
Follow the instructions here:
http://wiki.ros.org/noetic/Installation/Ubuntu
(currently we are using the noetic version of ROS but we are open to new suggestions)

For ROS tutorial it is advised that you follow the official ones from the ROS website:
http://wiki.ros.org/ROS/Tutorials

The project right now only contains 2 packages:
* ssam_simulation
* ssam_control

Note: Since all packages are common inside ROS along with the ones externally provided, it is recommended any newly
created packages you create to have the prefix ssam_.

After downloading the project, since this is a ROS catkin workspace, you should run "source devel/setup.bash" to get 
everything indexed properly in your terminal. Note that you will need to do this for every new terminal you open.
I also suggest you use Terminator or some other terminal which allow terminal splitting as you will be opening a lot
of different terminals while working with ROS.

### SSAM Simulation
This package contains the gazebo world (/worlds), the robot model (/models/scout) and the launch file (/launch).
To start it run "roslaunch ssam_simulation scout_simulation.launch". If everything is installed properly gazebo should
start up a world with some cubes and pylons and a 4 wheel robot in the middle.

You can direct the robot using messages to the /cmd_vel topic.

### SSAM Control

This package currently contains only 2 nodes (/src) which were created for testing purposes:
* test_control_node : moves the robot forward for 2 seconds, then backwards for 2 seconds. This node only publishes on
the (/cmd_vel) topic. You can start it with "rosrun ssam_control test_control_node"
* test_control_node2: This subscribes to the /odom topic to get info about the robot's position. If the robot's x
coordinate is smaller than 2 it will tell the robot to move forward. If the x coordinate is greater than 5 it will tell
the robot to move forward (using the /cmd_vel topic). You can start it with "rosrun ssam_control test_control_node2"

If you start the simulation and a control node you should see the robot moving forward and backwards in the gazebo
simulation.
