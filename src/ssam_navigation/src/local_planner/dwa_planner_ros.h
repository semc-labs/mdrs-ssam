#ifndef SSAM_DWA_PLANNER_ROS
#define SSAM_DWA_PLANNER_ROS

#include "dwa_planner.h"
#include "latched_stop_rotate_controller.h"

#include <base_local_planner/odometry_helper_ros.h>
#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <mbf_abstract_core/abstract_controller.h>

class DWAPlannerROS : public mbf_abstract_core::AbstractController
{
public:
    DWAPlannerROS();
    ~DWAPlannerROS();

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) final;

    uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity, geometry_msgs::TwistStamped &cmd_vel, std::string &message) final;
    bool isGoalReached(double xy_tolerance, double yaw_tolerance) final;
    virtual bool cancel() final { return false; }

    void initialize(std::string name, grid_map::GridMap* gridmap);
    bool dwaComputeVelocityCommands(const geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist &cmd_vel);
    bool isInitialized() { return m_Initialized; }

private:
    void reconfigureCB(dwa_local_planner::DWAPlannerConfig &config, uint32_t level);
    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path);
    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path);

    tf2_ros::Buffer* m_Tf; 

    ros::Publisher m_GlobalPlanPublisher;
    ros::Publisher m_LocalPlanPublisher;

    boost::shared_ptr<DWAPlanner> m_Planner;
    base_local_planner::LocalPlannerLimits m_Limits;
    grid_map::GridMap* m_Gridmap;

    dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>* m_ReconfigureServer;
    dwa_local_planner::DWAPlannerConfig m_DefaultConfig;
    bool m_Setup;
    geometry_msgs::PoseStamped m_CurrentPose;
    geometry_msgs::PoseStamped m_CurrentGoal;
    std::vector<geometry_msgs::PoseStamped> m_CurrentPlan;

    LatchedStopRotateController m_LatchedStopRotateController;

    bool m_Initialized;
    bool m_GoalReached{ false };
    base_local_planner::OdometryHelperRos m_OdomHelper;
    std::string m_OdomTopic;
};

#endif //SSAM_DWA_PLANNER_ROS