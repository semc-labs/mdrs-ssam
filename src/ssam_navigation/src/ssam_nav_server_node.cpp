#include "ssam_nav_server.h"

#include <signal.h>
#include <tf2_ros/transform_listener.h>

SSAMNavServer::Ptr navServerPtr;

void sigintHandler(int sig)
{
  ROS_INFO_STREAM("Shutdown costmap navigation server.");
  if(navServerPtr)
  {
    navServerPtr->stop();
  }
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ssam_nav_server", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double cache_time;
  private_nh.param("tf_cache_time", cache_time, 10.0);

  signal(SIGINT, sigintHandler);
  TFPtr tf_listener_ptr(new TF(ros::Duration(cache_time)));
  tf2_ros::TransformListener tf_listener(*tf_listener_ptr);

  navServerPtr = boost::make_shared<SSAMNavServer>(tf_listener_ptr);
  ros::spin();

  navServerPtr.reset();
  return EXIT_SUCCESS;
}
