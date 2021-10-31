#include "localization_ground_truth.h"

namespace gazebo
{
    void LocalizationGroundTruth::Load (physics::ModelPtr _robot, sdf::ElementPtr _sdf)
    {
        this->m_Robot = _robot;
        m_GazeboRos = GazeboRosPtr(new GazeboRos(_robot, _sdf, "MapOdometryLinker"));

        m_GazeboRos->isInitialized();

        double updateRate = 100.0;
        m_GazeboRos->getParameter<std::string> ( m_Topic, "topic", "truePose" );
        m_GazeboRos->getParameter<double> ( updateRate, "updateRate", 100.0 );

      
        if(updateRate > 0.0)
        {
            m_UpdatePeriod = 1.0 / updateRate;
        } 
        else 
        {
            m_UpdatePeriod = 0.0;
        }
      
        m_LastUpdateTime = m_Robot->GetWorld()->SimTime();

        m_Publisher = m_GazeboRos->node()->advertise<geometry_msgs::Pose>(m_Topic, 1);

        m_UpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LocalizationGroundTruth::Update, this));
    }

    void LocalizationGroundTruth::Update()
    {
        common::Time currentTime = m_Robot->GetWorld()->SimTime();
        double secondsSinceLastUpdate = (currentTime - m_LastUpdateTime).Double();

        if(secondsSinceLastUpdate > m_UpdatePeriod)
        {
            PublishPose(secondsSinceLastUpdate);

            m_LastUpdateTime += common::Time(m_UpdatePeriod);
        }
    }

    void LocalizationGroundTruth::PublishPose(double step_time)
    {
        ros::Time current_time = ros::Time::now();

        // getting data from gazebo world
        ignition::math::Pose3d pose = m_Robot->WorldPose();

        m_PoseMessage.position.x = pose.Pos().X();
        m_PoseMessage.position.y = pose.Pos().Y();
        m_PoseMessage.position.z = pose.Pos().Z();

        m_PoseMessage.orientation.x = pose.Rot().X();
        m_PoseMessage.orientation.y = pose.Rot().Y();
        m_PoseMessage.orientation.z = pose.Rot().Z();
        m_PoseMessage.orientation.w = pose.Rot().W();

        m_Publisher.publish(m_PoseMessage);
    }

    GZ_REGISTER_MODEL_PLUGIN(LocalizationGroundTruth)
}
