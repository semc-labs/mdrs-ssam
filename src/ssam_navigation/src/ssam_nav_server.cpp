#include "ssam_nav_server.h"

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_srvs/Empty.h>

SSAMNavServer::SSAMNavServer(const TFPtr &tf_listener_ptr) : AbstractNavigationServer(tf_listener_ptr)
{
  initializeServerComponents();
  startActionServers();

  ros::NodeHandle nh;
  m_OctomapSubscriber = nh.subscribe("octomap_full", 1000, &SSAMNavServer::octomapCallback, this);
  m_GridMapPublisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  m_ClearMapClient = nh.serviceClient<std_srvs::Empty>("octomap/reset");

  m_GridMap.setFrameId("map");

  cv::namedWindow("OpenCV Demo", cv::WINDOW_NORMAL);
}

SSAMNavServer::~SSAMNavServer()
{
  // remove every plugin before its classLoader goes out of scope.
  controller_plugin_manager_.clearPlugins();
  planner_plugin_manager_.clearPlugins();
  recovery_plugin_manager_.clearPlugins();

  action_server_recovery_ptr_.reset();
  action_server_exe_path_ptr_.reset();
  action_server_get_path_ptr_.reset();
  action_server_move_base_ptr_.reset();
}

void SSAMNavServer::stop()
{
  AbstractNavigationServer::stop();
}

mbf_abstract_core::AbstractPlanner::Ptr SSAMNavServer::loadPlannerPlugin(const std::string &planner_type)
{
  m_GlobalPlanner.reset(new AStarGlobalPlanner(m_GridMap));
  return m_GlobalPlanner;
}

bool SSAMNavServer::initializePlannerPlugin(const std::string &name, const mbf_abstract_core::AbstractPlanner::Ptr &planner_ptr)
{
  return true;
}

mbf_abstract_core::AbstractController::Ptr SSAMNavServer::loadControllerPlugin(const std::string &controller_type)
{
  m_LocalPlanner = boost::shared_ptr<DWAPlannerROS>(new DWAPlannerROS());
  return m_LocalPlanner;
}

bool SSAMNavServer::initializeControllerPlugin(const std::string &name, const mbf_abstract_core::AbstractController::Ptr &controller_ptr)
{
  m_LocalPlanner->initialize(name, &m_GridMap);
  return true;
}

void SSAMNavServer::octomapCallback(const octomap_msgs::Octomap &msg)
{
  boost::scoped_ptr<octomap::OcTree> octomap(static_cast<octomap::OcTree *>(octomap_msgs::msgToMap(msg)));
  grid_map::GridMapOctomapConverter::fromOctomap(*octomap, "elevation", m_GridMap);

  cv::Mat mapImage;
  grid_map::GridMapCvConverter::toImage<unsigned short, 1>(m_GridMap, "elevation", CV_16UC1, 0.0, 1.0, mapImage);

  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                              cv::Size(2 * 1 + 1, 2 * 1 + 1),
                                              cv::Point(1, 1));
  /// Apply the dilation operation
  dilate(mapImage, mapImage, element);

  // cv::imshow("OpenCV Demo", mapImage);
  // cv::waitKey(1);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(mapImage, "elevation", m_GridMap, 0.0f, 1.0f, 0.0f);

  m_GridMap.setTimestamp(ros::Time::now().toNSec());

  grid_map_msgs::GridMap message;

  grid_map::GridMapRosConverter::toMessage(m_GridMap, message);
  m_GridMapPublisher.publish(message);
}

void SSAMNavServer::callActionGetPath(mbf_abstract_nav::ActionServerGetPath::GoalHandle goal_handle)
{
  super::callActionGetPath(goal_handle);
}
