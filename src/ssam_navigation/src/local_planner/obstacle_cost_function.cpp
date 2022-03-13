#include "obstacle_cost_function.h"

ObstacleCostFunction::ObstacleCostFunction(grid_map::GridMap& gridmap) 
    : m_Gridmap(gridmap)
    , m_SumScores(false)
    , m_WorldModel(gridmap)
{
}

void ObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) 
{
  // TODO: move this to prepare if possible
  m_MaxTransVel = max_trans_vel;
  m_MaxScalingFactor = max_scaling_factor;
  m_ScalingSpeed = scaling_speed;
}

void ObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) 
{
  m_FootprintSpec = footprint_spec;
}

bool ObstacleCostFunction::prepare() {
  return true;
}

double ObstacleCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) 
{
  double cost = 0;
  double scale = getScalingFactor(traj, m_ScalingSpeed, m_MaxTransVel, m_MaxScalingFactor);
  double px, py, pth;
  if (m_FootprintSpec.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) 
  {
    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth, scale, m_FootprintSpec);

    if(f_cost < 0)
    {
        return f_cost;
    }

    if(m_SumScores)
        cost +=  f_cost;
    else
        cost = std::max(cost, f_cost);
  }

  return cost;
}

double ObstacleCostFunction::getScalingFactor(base_local_planner::Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) 
{
    double vmag = hypot(traj.xv_, traj.yv_);

    //if we're over a certain speed threshold, we'll scale the robot's
    //footprint to make it either slow down or stay further from walls
    double scale = 1.0;
    if (vmag > scaling_speed) 
    {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
    }
    return scale;
}

double ObstacleCostFunction::footprintCost(const double& x, const double& y, const double& th, double scale, std::vector<geometry_msgs::Point> footprint_spec) 
{
    //check if the footprint is legal
    // TODO: Cache inscribed radius
    double footprint_cost = m_WorldModel.footprintCost(x, y, th, footprint_spec);

    if (footprint_cost < 0) 
    {
        return -6.0;
    }
    
    grid_map::Index index;
    grid_map::Position position(x, y);
    
    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if (!m_Gridmap.getIndex(position, index)) 
    {
        return 0;
    }

    double occ_cost = std::max(std::max(0.0, footprint_cost), double(m_Gridmap.at("elevation", index)));

    return occ_cost;
}