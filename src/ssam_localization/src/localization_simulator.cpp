#include <stdlib.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

ros::Publisher* noisyOdomPublisher;

void PublishNoisyOdomMessage(const nav_msgs::Odometry::ConstPtr& msg)
{
    float noise = 1.0f * (rand() % 100) / 100.0f - 0.5f;
    
    nav_msgs::Odometry noisy_msg;
    noisy_msg.child_frame_id = msg->child_frame_id;
    noisy_msg.header = msg->header;
    noisy_msg.pose = msg->pose;
    noisy_msg.twist = msg->twist;

    noisy_msg.pose.pose.position.y += noise;
    noisyOdomPublisher->publish(noisy_msg);
}

void PublishMapTransform(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf::TransformBroadcaster tfBroadcaster;
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    
    geometry_msgs::Quaternion msg_quat = msg->pose.pose.orientation;
    tf::Quaternion quaternion = tf::Quaternion(msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w);
    transform.setRotation(quaternion);

    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    PublishNoisyOdomMessage(msg);
    PublishMapTransform(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_control_node", ros::init_options::AnonymousName);

    ROS_INFO("Booting up node");   

    ros::NodeHandle nodeHandle("~");

    std::string perfect_odom_topic = "/scout/diff_drive_controller/odom";
    ros::Subscriber subscriber = nodeHandle.subscribe(perfect_odom_topic, 1000, OdometryCallback);
    
    ros::Publisher local_nodom_publisher = nodeHandle.advertise<nav_msgs::Odometry>("/scout/diff_drive_controller/noisy_odom", 1000);
    noisyOdomPublisher = &local_nodom_publisher;
    
    ros::spin();

    return 0;
}
