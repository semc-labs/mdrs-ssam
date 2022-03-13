#ifndef SSAM_GRIDMAP_MODEL
#define SSAM_GRIDMAP_MODEL

#include <base_local_planner/world_model.h>
#include <grid_map_ros/grid_map_ros.hpp>

class GridmapModel : public base_local_planner::WorldModel
{
public:
  using base_local_planner::WorldModel::footprintCost;

  GridmapModel(const grid_map::GridMap& gridmap) : m_Gridmap(gridmap) {}
  virtual ~GridmapModel(){}

private:
  double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint, double inscribed_radius, double circumscribed_radius) final;

  double lineCost(int x0, int x1, int y0, int y1) const;
  double pointCost(int x, int y) const;

private:
  const grid_map::GridMap& m_Gridmap;
};

#endif //SSAM_GRIDMAP_MODEL