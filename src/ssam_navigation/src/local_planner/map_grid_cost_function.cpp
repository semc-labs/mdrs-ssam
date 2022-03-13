#include "map_grid_cost_function.h"

MapGridCostFunction::MapGridCostFunction(grid_map::GridMap &gridmap,
                                         double xshift,
                                         double yshift,
                                         bool is_local_goal_function,
                                         CostAggregationType aggregationType)
    : m_Gridmap(gridmap)
    , m_Map(m_Gridmap.getSize()[0], m_Gridmap.getSize()[1])
    , m_AggregationType(aggregationType)
    , m_XShift(xshift), m_YShift(yshift)
    , m_IsLocalGoalFunction(is_local_goal_function)
    , m_StopOnFailure(true) 
{}

void MapGridCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses)
{
    m_TargetPoses = target_poses;
}

bool MapGridCostFunction::prepare()
{
    m_Map.resize(m_Gridmap.getSize()[0], m_Gridmap.getSize()[1]);
    m_Map.resetPathDist();

    if (m_IsLocalGoalFunction)
    {
        m_Map.setLocalGoal(m_Gridmap, m_TargetPoses);
    }
    else
    {
        m_Map.setTargetCells(m_Gridmap, m_TargetPoses);
    }
    return true;
}

double MapGridCostFunction::getCellCosts(unsigned int px, unsigned int py)
{
    double grid_dist = m_Map(px, py).target_dist;
    return grid_dist;
}

double MapGridCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
{
    double cost = 0.0;
    if (m_AggregationType == Product)
    {
        cost = 1.0;
    }
    double px, py, pth;
    double grid_dist;

    for (unsigned int i = 0; i < traj.getPointsSize(); ++i)
    {
        traj.getPoint(i, px, py, pth);

        // translate point forward if specified
        if (m_XShift != 0.0)
        {
            px = px + m_XShift * cos(pth);
            py = py + m_XShift * sin(pth);
        }
        // translate point sideways if specified
        if (m_YShift != 0.0)
        {
            px = px + m_YShift * cos(pth + M_PI_2);
            py = py + m_YShift * sin(pth + M_PI_2);
        }

        //we won't allow trajectories that go off the map... shouldn't happen that often anyways
        grid_map::Index index;
        if(!m_Gridmap.getIndex({px, py}, index))
        {
            //we're off the map
            ROS_WARN("Off Map %f, %f", px, py);
            return 0.0;
        }
        grid_dist = getCellCosts(index[0], index[1]);
        //if a point on this trajectory has no clear path to the goal... it may be invalid
        if (m_StopOnFailure)
        {
            if (grid_dist == m_Map.obstacleCosts())
            {
                return -3.0;
            }
            else if (grid_dist == m_Map.unreachableCellCosts())
            {
                return -2.0;
            }
        }

        switch (m_AggregationType)
        {
        case Last:
            cost = grid_dist;
            break;
        case Sum:
            cost += grid_dist;
            break;
        case Product:
            if (cost > 0)
            {
                cost *= grid_dist;
            }
            break;
        }
    }
    return cost;
}