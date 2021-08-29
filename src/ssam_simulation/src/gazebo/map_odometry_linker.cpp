#include "map_odometry_linker.h"

namespace gazebo
{
  void MapOdometryLinker::Load (physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
      this->m_Parent = _parent;
      m_GazeboRos = GazeboRosPtr(new GazeboRos(_parent, _sdf, "MapOdometryLinker"));

      m_GazeboRos->isInitialized();

      m_GazeboRos->getParameter<std::string> ( m_OdometryTopic, "odometryTopic", "odom" );
      m_GazeboRos->getParameter<std::string> ( m_OdometryFrame, "odometryFrame", "odom" );
      m_GazeboRos->getParameter<std::string> ( m_RobotBaseFrame, "robotBaseFrame", "base_footprint" );
      m_GazeboRos->getParameter<double> ( m_UpdateRate, "updateRate", 100.0 );

      
      if(m_UpdateRate > 0.0)
      {
        m_UpdatePeriod = 1.0 / m_UpdateRate;
      } 
      else 
      {
        m_UpdatePeriod = 0.0;
      }
      
      m_LastUpdateTime = m_Parent->GetWorld()->SimTime();

      m_TransformBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

      m_OdometryPublisher = m_GazeboRos->node()->advertise<nav_msgs::Odometry>(m_OdometryTopic, 1);

      m_UpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MapOdometryLinker::UpdateChild, this));
  }

  void MapOdometryLinker::UpdateChild()
  {
    common::Time currentTime = m_Parent->GetWorld()->SimTime();
    double secondsSinceLastUpdate = (currentTime - m_LastUpdateTime).Double();

    if(secondsSinceLastUpdate > m_UpdatePeriod)
    {
      PublishOdometry(secondsSinceLastUpdate);

      m_LastUpdateTime += common::Time(m_UpdatePeriod);
    }
  }

  void MapOdometryLinker::PublishOdometry(double step_time)
  {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = m_GazeboRos->resolveTF(m_OdometryFrame);
    std::string base_footprint_frame = m_GazeboRos->resolveTF(m_RobotBaseFrame);

    tf::Quaternion qt;
    tf::Vector3 vt;

    // getting data from gazebo world
    ignition::math::Pose3d pose = m_Parent->WorldPose();

    qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    
    tf::Transform base_footprint_to_odom ( qt, vt );
    m_TransformBroadcaster->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
                                odom_frame, base_footprint_frame));

    m_OdometryMessage.pose.pose.position.x = vt.x();
    m_OdometryMessage.pose.pose.position.y = vt.y();
    m_OdometryMessage.pose.pose.position.z = vt.z();

    m_OdometryMessage.pose.pose.orientation.x = qt.x();
    m_OdometryMessage.pose.pose.orientation.y = qt.y();
    m_OdometryMessage.pose.pose.orientation.z = qt.z();
    m_OdometryMessage.pose.pose.orientation.w = qt.w();

    ignition::math::Vector3d linear;
    linear = m_Parent->WorldLinearVel();
    m_OdometryMessage.twist.twist.angular.z = m_Parent->WorldAngularVel().Z();

    float yaw = pose.Rot().Yaw();
    m_OdometryMessage.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    m_OdometryMessage.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

    // set covariance
    m_OdometryMessage.pose.covariance[0] = 0.00001;
    m_OdometryMessage.pose.covariance[7] = 0.00001;
    m_OdometryMessage.pose.covariance[14] = 1000000000000.0;
    m_OdometryMessage.pose.covariance[21] = 1000000000000.0;
    m_OdometryMessage.pose.covariance[28] = 1000000000000.0;
    m_OdometryMessage.pose.covariance[35] = 0.001;


    // set header
    m_OdometryMessage.header.stamp = current_time;
    m_OdometryMessage.header.frame_id = odom_frame;
    m_OdometryMessage.child_frame_id = base_footprint_frame;

    m_OdometryPublisher.publish(m_OdometryMessage);
  }

  GZ_REGISTER_MODEL_PLUGIN(MapOdometryLinker)
}