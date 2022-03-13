#include "map_grid.h"

MapGrid::MapGrid()
    : m_SizeX(0), m_SizeY(0)
{
}

MapGrid::MapGrid(unsigned int size_x, unsigned int size_y)
    : m_SizeX(size_x), m_SizeY(size_y)
{
    commonInit();
}

MapGrid::MapGrid(const MapGrid &mg)
{
    m_SizeY = mg.m_SizeY;
    m_SizeX = mg.m_SizeX;
    m_Map = mg.m_Map;
}

void MapGrid::resize(unsigned int size_x, unsigned int size_y)
{
    m_SizeX = size_x;
    m_SizeY = size_y;

    commonInit();
}

void MapGrid::commonInit()
{
    //don't allow construction of zero size grid
    //ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    m_Map.resize(m_SizeY * m_SizeX);

    //make each cell aware of its location in the grid
    for (unsigned int i = 0; i < m_SizeY; ++i)
    {
        for (unsigned int j = 0; j < m_SizeX; ++j)
        {
            unsigned int id = m_SizeX * i + j;
            m_Map[id].cx = j;
            m_Map[id].cy = i;
        }
    }
}

size_t MapGrid::getIndex(int x, int y)
{
    return m_SizeX * y + x;
}

MapGrid &MapGrid::operator=(const MapGrid &mg)
{
    m_SizeY = mg.m_SizeY;
    m_SizeX = mg.m_SizeX;
    m_Map = mg.m_Map;
    return *this;
}

void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y)
{
    if (m_Map.size() != size_x * size_y)
        m_Map.resize(size_x * size_y);

    if (m_SizeX != size_x || m_SizeY != size_y)
    {
        m_SizeX = size_x;
        m_SizeY = size_y;

        for (unsigned int i = 0; i < m_SizeY; ++i)
        {
            for (unsigned int j = 0; j < m_SizeX; ++j)
            {
                int index = m_SizeX * i + j;
                m_Map[index].cx = j;
                m_Map[index].cy = i;
            }
        }
    }
}

inline bool MapGrid::updatePathCell(base_local_planner::MapCell *current_cell, base_local_planner::MapCell *check_cell, const grid_map::GridMap& gridmap)
{
    //if the cell is an obstacle set the max path distance
    float elevation = gridmap.at("elevation", {check_cell->cx, check_cell->cy});
    if(elevation > 0.5f)
    {
        check_cell->target_dist = obstacleCosts();
        return false;
    }

    double new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist)
    {
        check_cell->target_dist = new_target_dist;
    }
    return true;
}

//reset the path_dist and goal_dist fields for all cells
void MapGrid::resetPathDist()
{
    for (unsigned int i = 0; i < m_Map.size(); ++i)
    {
        m_Map[i].target_dist = unreachableCellCosts();
        m_Map[i].target_mark = false;
        m_Map[i].within_robot = false;
    }
}

void MapGrid::adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped> &global_plan_in, std::vector<geometry_msgs::PoseStamped> &global_plan_out, double resolution)
{
    if (global_plan_in.size() == 0)
    {
        return;
    }
    double last_x = global_plan_in[0].pose.position.x;
    double last_y = global_plan_in[0].pose.position.y;
    global_plan_out.push_back(global_plan_in[0]);

    double min_sq_resolution = resolution * resolution;

    for (unsigned int i = 1; i < global_plan_in.size(); ++i)
    {
        double loop_x = global_plan_in[i].pose.position.x;
        double loop_y = global_plan_in[i].pose.position.y;
        double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
        if (sqdist > min_sq_resolution)
        {
            int steps = ceil((sqrt(sqdist)) / resolution);
            // add a points in-between
            double deltax = (loop_x - last_x) / steps;
            double deltay = (loop_y - last_y) / steps;
            // TODO: Interpolate orientation
            for (int j = 1; j < steps; ++j)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = last_x + j * deltax;
                pose.pose.position.y = last_y + j * deltay;
                pose.pose.position.z = global_plan_in[i].pose.position.z;
                pose.pose.orientation = global_plan_in[i].pose.orientation;
                pose.header = global_plan_in[i].header;
                global_plan_out.push_back(pose);
            }
        }
        global_plan_out.push_back(global_plan_in[i]);
        last_x = loop_x;
        last_y = loop_y;
    }
}

//update what map cells are considered path based on the global_plan
void MapGrid::setTargetCells(const grid_map::GridMap& gridmap, const std::vector<geometry_msgs::PoseStamped> &global_plan)
{
    sizeCheck(gridmap.getSize()[0], gridmap.getSize()[1]);

    bool started_path = false;

    std::queue<base_local_planner::MapCell*> path_dist_queue;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, gridmap.getResolution());
    if (adjusted_global_plan.size() != global_plan.size())
    {
        ROS_DEBUG("Adjusted global plan resolution, added %zu points", adjusted_global_plan.size() - global_plan.size());
    }
    unsigned int i;
    // put global path points into local map until we reach the border of the local map
    for (i = 0; i < adjusted_global_plan.size(); ++i)
    {
        double g_x = adjusted_global_plan[i].pose.position.x;
        double g_y = adjusted_global_plan[i].pose.position.y;
        grid_map::Index mapIndex;
        if (gridmap.getIndex({g_x, g_y}, mapIndex))
        {
            base_local_planner::MapCell &current = getCell(mapIndex[0], mapIndex[1]);
            current.target_dist = 0.0;
            current.target_mark = true;
            path_dist_queue.push(&current);
            started_path = true;
        }
        else if (started_path)
        {
            break;
        }
    }
    if (!started_path)
    {
        ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
                  i, adjusted_global_plan.size(), global_plan.size());
        return;
    }

    computeTargetDistance(path_dist_queue, gridmap);
}

//mark the point of the costmap as local goal where global_plan first leaves the area (or its last point)
void MapGrid::setLocalGoal(const grid_map::GridMap& gridmap, const std::vector<geometry_msgs::PoseStamped> &global_plan)
{
    sizeCheck(gridmap.getSize()[0], gridmap.getSize()[1]);

    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, gridmap.getResolution());

    // skip global path points until we reach the border of the local map
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i)
    {
        double g_x = adjusted_global_plan[i].pose.position.x;
        double g_y = adjusted_global_plan[i].pose.position.y;
        grid_map::Index mapIndex;
        if (gridmap.getIndex({g_x, g_y}, mapIndex))
        {
            local_goal_x = mapIndex[0];
            local_goal_y = mapIndex[1];
            started_path = true;
        }
        else
        {
            if (started_path)
            {
                break;
            } // else we might have a non pruned path, so we just continue
        }
    }
    if (!started_path)
    {
        ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
        return;
    }

    std::queue<base_local_planner::MapCell*> path_dist_queue;
    if (local_goal_x >= 0 && local_goal_y >= 0)
    {
        base_local_planner::MapCell &current = getCell(local_goal_x, local_goal_y);
        grid_map::Position goal;
        gridmap.getPosition({local_goal_x, local_goal_y}, goal);

        m_GoalX = goal[0];
        m_GoalY = goal[1];

        current.target_dist = 0.0;
        current.target_mark = true;
        path_dist_queue.push(&current);
    }

    computeTargetDistance(path_dist_queue, gridmap);
}

void MapGrid::computeTargetDistance(std::queue<base_local_planner::MapCell*>& dist_queue, const grid_map::GridMap& gridmap)
{
    base_local_planner::MapCell *current_cell;
    base_local_planner::MapCell *check_cell;
    unsigned int last_col = m_SizeX - 1;
    unsigned int last_row = m_SizeY - 1;
    while (!dist_queue.empty())
    {
        current_cell = dist_queue.front();

        dist_queue.pop();

        if (current_cell->cx > 0)
        {
            check_cell = current_cell - 1;
            if (!check_cell->target_mark)
            {
                //mark the cell as visisted
                check_cell->target_mark = true;
                if (updatePathCell(current_cell, check_cell, gridmap))
                {
                    dist_queue.push(check_cell);
                }
            }
        }

        if (current_cell->cx < last_col)
        {
            check_cell = current_cell + 1;
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if (updatePathCell(current_cell, check_cell, gridmap))
                {
                    dist_queue.push(check_cell);
                }
            }
        }

        if (current_cell->cy > 0)
        {
            check_cell = current_cell - m_SizeX;
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if (updatePathCell(current_cell, check_cell, gridmap))
                {
                    dist_queue.push(check_cell);
                }
            }
        }

        if (current_cell->cy < last_row)
        {
            check_cell = current_cell + m_SizeX;
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if (updatePathCell(current_cell, check_cell, gridmap))
                {
                    dist_queue.push(check_cell);
                }
            }
        }
    }
}