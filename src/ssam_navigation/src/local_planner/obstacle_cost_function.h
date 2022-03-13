#ifndef SSAM_OBSTACLE_COST_FUNCTION
#define SSAM_OBSTACLE_COST_FUNCTION

#include "gridmap_model.h"

#include <base_local_planner/trajectory_cost_function.h>
#include <grid_map_ros/grid_map_ros.hpp>

class ObstacleCostFunction : public base_local_planner::TrajectoryCostFunction
{
public:
  ObstacleCostFunction(grid_map::GridMap &gridmap);

  bool prepare();
  double scoreTrajectory(base_local_planner::Trajectory &traj);

  void setSumScores(bool score_sums) { m_SumScores = score_sums; }

  void setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed);
  void setFootprint(std::vector<geometry_msgs::Point> footprint_spec);

  // helper functions, made static for easy unit testing
  static double getScalingFactor(base_local_planner::Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor);
  double footprintCost(const double &x, const double &y, const double &th, double scale, std::vector<geometry_msgs::Point> footprint_spec);

private:
  double lineCost(int x0, int x1, int y0, int y1) const;
  double pointCost(int x, int y) const;

private:
  grid_map::GridMap &m_Gridmap;
  GridmapModel m_WorldModel;
  std::vector<geometry_msgs::Point> m_FootprintSpec;
  double m_MaxTransVel;
  bool m_SumScores;
  //footprint scaling with velocity;
  double m_MaxScalingFactor, m_ScalingSpeed;
};

#endif //SSAM_OBSTACLE_COST_FUNCTION