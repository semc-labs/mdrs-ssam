#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

ros::NodeHandle* nodeHandle;
ros::Publisher* publisher;
ros::Subscriber* subscriber;

int last_velocity_sent = 0;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x_pos = msg->pose.pose.position.x;

    if (x_pos > 5 && last_velocity_sent != -1)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = -1;
        last_velocity_sent = -1;
        ROS_INFO("Sent velocity message: [%.2f %.2f %.2f] [%.2f %.2f %.2f]", twist.linear.x, twist.linear.y,
                    twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z);

        publisher->publish(twist);
    }
    else if (x_pos < 2 && last_velocity_sent != 1)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = 1;
        last_velocity_sent = 1;
        ROS_INFO("Sent velocity message: [%.2f %.2f %.2f] [%.2f %.2f %.2f]", twist.linear.x, twist.linear.y,
                    twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z);

        publisher->publish(twist);
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_control_node2", ros::init_options::AnonymousName);

    ROS_INFO("Booting up node");

    ros::NodeHandle local_nodeHandle = ros::NodeHandle("~");

    std::string scout_name;
    local_nodeHandle.getParam("scout_name", scout_name);

    if (scout_name.size() != 0)
    {
        ROS_INFO("Controlling scout: %s", scout_name.c_str());
    }

    ros::Publisher local_publisher = local_nodeHandle.advertise<geometry_msgs::Twist>("/" + scout_name + "/cmd_vel", 1000);
    ros::Subscriber local_subscriber = local_nodeHandle.subscribe("/" + scout_name + "/odom", 1000, odometryCallback);

    nodeHandle = &local_nodeHandle;
    publisher = &local_publisher;
    subscriber = &local_subscriber;   

    ros::spin();

    return 0;
}
