#ifndef SSAM_MAP_GRID
#define SSAM_MAP_GRID

#include <base_local_planner/map_cell.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <queue>

class MapGrid
{
public:
    MapGrid();
    MapGrid(unsigned int size_x, unsigned int size_y);
    MapGrid(const MapGrid &mg);

    inline base_local_planner::MapCell &operator()(unsigned int x, unsigned int y) { return m_Map[m_SizeX * y + x]; }
    inline base_local_planner::MapCell operator()(unsigned int x, unsigned int y) const { return m_Map[m_SizeX * y + x]; }
    inline base_local_planner::MapCell &getCell(unsigned int x, unsigned int y) { return m_Map[m_SizeX * y + x]; }

    MapGrid &operator=(const MapGrid &mg);

    void resize(unsigned int size_x, unsigned int size_y);
    void resetPathDist();
    void sizeCheck(unsigned int size_x, unsigned int size_y);
    void commonInit();
    size_t getIndex(int x, int y);

    inline double obstacleCosts() { return m_Map.size(); }
    inline double unreachableCellCosts() { return m_Map.size() + 1; }
    inline bool updatePathCell(base_local_planner::MapCell *current_cell, base_local_planner::MapCell *check_cell, const grid_map::GridMap& gridmap);

    static void adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped> &global_plan_in, std::vector<geometry_msgs::PoseStamped> &global_plan_out, double resolution);


    void computeTargetDistance(std::queue<base_local_planner::MapCell*> &dist_queue, const grid_map::GridMap& gridmap);
    void computeGoalDistance(std::queue<base_local_planner::MapCell*> &dist_queue, const grid_map::GridMap& gridmap);

    void setTargetCells(const grid_map::GridMap& gridmap, const std::vector<geometry_msgs::PoseStamped> &global_plan);
    void setLocalGoal(const grid_map::GridMap& gridmap, const std::vector<geometry_msgs::PoseStamped> &global_plan);

    double m_GoalX, m_GoalY;

    unsigned int m_SizeX, m_SizeY;

private:
    std::vector<base_local_planner::MapCell> m_Map;
};

#endif //SSAM_MAP_GRID