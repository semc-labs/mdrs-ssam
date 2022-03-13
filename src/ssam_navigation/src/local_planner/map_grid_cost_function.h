#ifndef SSAM_MAP_GRID_COST_FUNCTION
#define SSAM_MAP_GRID_COST_FUNCTION

#include "map_grid.h"

#include <base_local_planner/trajectory_cost_function.h>
#include <grid_map_ros/grid_map_ros.hpp>

enum CostAggregationType
{
    Last,
    Sum,
    Product
};

class MapGridCostFunction : public base_local_planner::TrajectoryCostFunction
{
public:
    MapGridCostFunction(grid_map::GridMap& gridmap,
                        double xshift = 0.0,
                        double yshift = 0.0,
                        bool is_local_goal_function = false,
                        CostAggregationType aggregationType = Last);

    ~MapGridCostFunction() {}

    void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

    void setXShift(double xshift) { m_XShift = xshift; }
    void setYShift(double yshift) { m_YShift = yshift; }

    void setStopOnFailure(bool stop_on_failure) { m_StopOnFailure = stop_on_failure; }

    bool prepare();

    double scoreTrajectory(base_local_planner::Trajectory &traj);

    double obstacleCosts() { return m_Map.obstacleCosts(); }
    double unreachableCellCosts() { return m_Map.unreachableCellCosts(); }

    // used for easier debugging
    double getCellCosts(unsigned int cx, unsigned int cy);

private:
    std::vector<geometry_msgs::PoseStamped> m_TargetPoses;
    grid_map::GridMap& m_Gridmap;

    MapGrid m_Map;
    CostAggregationType m_AggregationType;

    double m_XShift;
    double m_YShift;

    bool m_IsLocalGoalFunction;
    bool m_StopOnFailure;
};

#endif //SSAM_MAP_GRID_COST_FUNCTION