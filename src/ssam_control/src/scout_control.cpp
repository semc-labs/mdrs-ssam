#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <controller_manager/controller_manager.h>

#include "ssam_control/hardware_interface.hpp"

void ControlLoop(ScoutHardwareInterface& hardwareInterface, controller_manager::ControllerManager& controllerManager, ros::Time& lastTime)
{
    ros::Time currentTime = ros::Time::now();
    ros::Duration elapsedTime = currentTime - lastTime;
    lastTime = currentTime;

    hardwareInterface.read(currentTime, elapsedTime);
    controllerManager.update(currentTime, elapsedTime);
    hardwareInterface.write(currentTime, elapsedTime);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scout_control");

    ros::NodeHandle nodeHandle;
    ros::NodeHandle robotHardwareNodeHandle("~");

    ScoutHardwareInterface hardwareInterface;
    if(!hardwareInterface.init(nodeHandle, robotHardwareNodeHandle))
    {
        return 1;
    }

    controller_manager::ControllerManager cm(&hardwareInterface, nodeHandle);

    double controlFrequency;
    if(!robotHardwareNodeHandle.param("controlFrequency", controlFrequency, 10.0))
    {
        ROS_WARN("controlFrequency not set - defaulting to %.1f", controlFrequency);
    }

    ros::CallbackQueue callbackQueue;
    ros::AsyncSpinner my_robot_spinner(1, &callbackQueue);

    ros::Time lastTime = ros::Time::now();
    ros::TimerOptions control_timer(
        ros::Duration(1 / controlFrequency), 
        std::bind(ControlLoop, std::ref(hardwareInterface), std::ref(cm), std::ref(lastTime)), 
        &callbackQueue);
    ros::Timer control_loop = nodeHandle.createTimer(control_timer);
    my_robot_spinner.start();
    ros::spin();

    return 0;
}