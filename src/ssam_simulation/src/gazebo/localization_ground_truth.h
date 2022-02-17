// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{
    // Publishes the true position of the scout in the map frame
    // This is used to debug with our localization system
    class LocalizationGroundTruth : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void PublishPose(double step_time);
        void Update();

    private:
        physics::ModelPtr m_Robot;
        GazeboRosPtr m_GazeboRos;
        double m_UpdatePeriod;
        common::Time m_LastUpdateTime;
        std::string m_Topic;
        ros::Publisher m_Publisher;
        event::ConnectionPtr m_UpdateConnection;
        geometry_msgs::Pose m_PoseMessage;        
    };
}
