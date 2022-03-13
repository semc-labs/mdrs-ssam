#ifndef SSAM_DWA_PLANNER
#define SSAM_DWA_PLANNER

#include "map_grid_cost_function.h"
#include "obstacle_cost_function.h"


#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/twirling_cost_function.h>

#include <dwa_local_planner/DWAPlannerConfig.h>
#include <grid_map_ros/grid_map_ros.hpp>

class DWAPlanner 
{
public:
    DWAPlanner(std::string name, grid_map::GridMap& gridmap, base_local_planner::LocalPlannerLimits& limits);
    void reconfigure(dwa_local_planner::DWAPlannerConfig& cfg);
    bool checkTrajectory(const Eigen::Vector3f pos, const Eigen::Vector3f vel, const Eigen::Vector3f vel_samples);
    base_local_planner::Trajectory findBestPath(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities);
    void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan, const std::vector<geometry_msgs::Point>& footprint_spec);
    double getSimPeriod() { return m_SimPeriod; }
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

private:
    grid_map::GridMap& m_Gridmap;
    base_local_planner::LocalPlannerLimits& m_Limits;

    double m_StopTimeBuffer; ///< @brief How long before hitting something we're going to enforce that the robot stop
    double m_PathDistanceBias, m_GoalDistanceBias, m_OccdistScale;
    Eigen::Vector3f m_VSamples;

    double m_SimPeriod;///< @brief The number of seconds to use to compute max/min vels for dwa
    base_local_planner::Trajectory m_ResultTrajectory;

    double m_ForwardPointDistance;

    std::vector<geometry_msgs::PoseStamped> m_GlobalPlan;

    boost::mutex m_ConfigurationMutex;
    std::string m_FrameId;
    ros::Publisher m_TrajCloudPublisher;
    bool m_PublishCostGridPc; ///< @brief Whether or not to build and publish a PointCloud
    bool m_PublishTrajPc;

    double m_CheatFactor;

    // see constructor body for explanations
    base_local_planner::SimpleTrajectoryGenerator m_Generator;
    base_local_planner::OscillationCostFunction m_OscillationCosts;
    ObstacleCostFunction m_ObstacleCosts;
    MapGridCostFunction m_PathCosts;
    MapGridCostFunction m_GoalCosts;
    MapGridCostFunction m_GoalFrontCosts;
    MapGridCostFunction m_AlignmentCosts;
    base_local_planner::TwirlingCostFunction m_TwirlingCosts;

    base_local_planner::SimpleScoredSamplingPlanner m_ScoredSamplingPlanner;
};

#endif //SSAM_DWA_PLANNER