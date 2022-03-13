#include "gridmap_model.h"

#include <base_local_planner/line_iterator.h>

const int K_OFF_MAP_COST = 0; //TODO switch to -3 later

double GridmapModel::footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint, double inscribed_radius, double circumscribed_radius)
{
    // returns:
    //  -1 if footprint covers at least a lethal obstacle cell, or
    //  -2 if footprint covers at least a no-information cell, or
    //  -3 if footprint is [partially] outside of the map, or
    //  a positive value for traversable space

    //used to put things into grid coordinates
    grid_map::Index index;
    
    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if (!m_Gridmap.getIndex({position.x, position.y}, index)) 
        return K_OFF_MAP_COST;

    //if number of points in the footprint is less than 3, we'll just assume a circular robot
    if(footprint.size() < 3)
    {
        float elevation = m_Gridmap.at("elevation", index);
        if(elevation > 0.5f)
        {
            return -1;
        }

        return 0;
    }

    //now we really have to lay down the footprint in the costmap grid
    grid_map::Index lineStart;
    grid_map::Index lineEnd;
    double line_cost = 0.0;
    double footprint_cost = 0.0;

    //we need to rasterize each line in the footprint
    for(unsigned int i = 0; i < footprint.size() - 1; ++i)
    {
        //get the cell coord of the first point
        if(!m_Gridmap.getIndex({footprint[i].x, footprint[i].y}, lineStart))
        {
            return K_OFF_MAP_COST;
        }

        //get the cell coord of the second point
        if(!m_Gridmap.getIndex({footprint[i + 1].x, footprint[i + 1].y}, lineEnd))
        {
            return K_OFF_MAP_COST;
        }

        line_cost = lineCost(lineStart[0], lineEnd[0], lineStart[1], lineEnd[1]);
        footprint_cost = std::max(line_cost, footprint_cost);

        //if there is an obstacle that hits the line... we know that we can return false right away
        if(line_cost < 0)
        {
            return line_cost;
        }
    }

    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the first point
    if(!m_Gridmap.getIndex({footprint.back().x, footprint.back().y}, lineStart))
    {
        return K_OFF_MAP_COST;
    }

    //get the cell coord of the second point
    if(!m_Gridmap.getIndex({footprint.front().x, footprint.front().y}, lineEnd))
    {
        return K_OFF_MAP_COST;
    }

    line_cost = lineCost(lineStart[0], lineEnd[0], lineStart[1], lineEnd[1]);
    footprint_cost = std::max(line_cost, footprint_cost);

    //if there is an obstacle that hits the line... we know that we can return false right away
    if(line_cost < 0)
    {
        return line_cost;
    }

    //if all line costs are legal... then we can return that the footprint is legal
    return footprint_cost;
}

//calculate the cost of a ray-traced line
double GridmapModel::lineCost(int x0, int x1, int y0, int y1) const 
{
    double line_cost = 0.0;
    double point_cost = -1.0;

    for(base_local_planner::LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance())
    {
        point_cost = pointCost( line.getX(), line.getY() ); //Score the current point

        if(point_cost < 0)
        return point_cost;

        if(line_cost < point_cost)
        line_cost = point_cost;
    }

    return line_cost;
}

double GridmapModel::pointCost(int x, int y) const 
{
    float elevation = m_Gridmap.at("elevation", {x, y});
    //if the cell is in an obstacle the path is invalid
        
    if(elevation > 0.5f)
    {
        return -1;
    }

    return 0;
}