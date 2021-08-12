#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_control_node");

    ROS_INFO("Booting up node");

    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate loop_rate(0.5);
    bool forward = true;

    // Sleep once to make sure all subscribers are notified of the new publisher
    loop_rate.sleep();
    ROS_INFO("All good, sending velocity messages");

    while (ros::ok())
    {
        geometry_msgs::Twist twist;
        twist.linear.x = forward ? 1 : -1;
        ROS_INFO("Sent velocity message: [%.2f %.2f %.2f] [%.2f %.2f %.2f]", twist.linear.x, twist.linear.y,
                    twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z);

        publisher.publish(twist);

        ros::spinOnce();

        loop_rate.sleep();
        forward = !forward;
    }

    return 0;
}
