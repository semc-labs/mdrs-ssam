#ifndef SSAM_NAV_SERVER
#define SSAM_NAV_SERVER

#include "global_planner/a_star_global_planner.h"
#include "local_planner/dwa_planner_ros.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <mbf_abstract_nav/abstract_navigation_server.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>

class SSAMNavServer : public mbf_abstract_nav::AbstractNavigationServer
{
  using super = mbf_abstract_nav::AbstractNavigationServer;
public:
  typedef boost::shared_ptr<SSAMNavServer> Ptr;

  SSAMNavServer(const TFPtr &tf_listener_ptr);
  virtual ~SSAMNavServer();

  void stop() final;

private:
  mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string &planner_type) final;
  bool initializePlannerPlugin(const std::string &name,const mbf_abstract_core::AbstractPlanner::Ptr &planner_ptr) final;

  mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string &controller_type) final;
  bool initializeControllerPlugin(const std::string &name, const mbf_abstract_core::AbstractController::Ptr &controller_ptr) final;

  mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string &recovery_type) final { return nullptr; }
  bool initializeRecoveryPlugin(const std::string &name, const mbf_abstract_core::AbstractRecovery::Ptr &behavior_ptr) final { return false; }

  void octomapCallback(const octomap_msgs::Octomap& msg);

  void callActionGetPath(mbf_abstract_nav::ActionServerGetPath::GoalHandle goal_handle) override;

private:
  boost::shared_ptr<AStarGlobalPlanner> m_GlobalPlanner;
  boost::shared_ptr<DWAPlannerROS> m_LocalPlanner;

  ros::Subscriber m_OctomapSubscriber;
  ros::Publisher m_GridMapPublisher;
  ros::ServiceClient m_ClearMapClient;
  grid_map::GridMap m_GridMap{ {"elevation"} };
};

#endif //SSAM_NAV_SERVER