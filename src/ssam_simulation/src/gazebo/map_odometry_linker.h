#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
    class MapOdometryLinker : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void PublishOdometry(double step_time);
        void UpdateChild();

    private:
        physics::ModelPtr m_Parent;
        GazeboRosPtr m_GazeboRos;
        double m_UpdateRate;
        double m_UpdatePeriod;
        common::Time m_LastUpdateTime;
        std::string m_OdometryTopic;
        std::string m_OdometryFrame;
        std::string m_RobotBaseFrame;
        boost::shared_ptr<tf::TransformBroadcaster> m_TransformBroadcaster;
        ros::Publisher m_OdometryPublisher;
        event::ConnectionPtr m_UpdateConnection;
        nav_msgs::Odometry m_OdometryMessage;
    };
}