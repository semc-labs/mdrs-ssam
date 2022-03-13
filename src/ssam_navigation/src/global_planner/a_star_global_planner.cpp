#include "a_star_global_planner.h"

uint32_t AStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
{
    std::shared_ptr<Node> startNode(new Node());
    std::shared_ptr<Node> goalNode(new Node());

    grid_map::Position startPosition(start.pose.position.x, start.pose.position.y);
    grid_map::Position goalPosition(goal.pose.position.x, goal.pose.position.y);

    goalPosition = m_GridMap.getClosestPositionInMap(goalPosition);

    grid_map::Index startIndex;
    grid_map::Index goalIndex;
    
    m_GridMap.getIndex(startPosition, startIndex);
    m_GridMap.getIndex(goalPosition, goalIndex);

    startNode->x = startIndex[0];
    startNode->y = startIndex[1];
    goalNode->x = goalIndex[0];
    goalNode->y = goalIndex[1];

    std::vector<Node> path = aStar(startNode, goalNode);
    smoothPath(path);
    
    for(Node& node : path)
    {
        geometry_msgs::PoseStamped pose = goal;
        grid_map::Index index(node.x, node.y);
        grid_map::Position position;
        m_GridMap.getPosition(index, position);

        pose.pose.position.x = position[0];
        pose.pose.position.y = position[1];

        plan.push_back(pose);
    }

    publishPlan(plan);

    return 0;
}

void AStarGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) 
{
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    m_PlanPublisher.publish(gui_path);
}

std::vector<AStarGlobalPlanner::Node> AStarGlobalPlanner::aStar(std::shared_ptr<Node>& start, std::shared_ptr<Node>& goal)
{
    std::vector<std::shared_ptr<Node>> openList;
    std::vector<std::shared_ptr<Node>> closedList;

    openList.push_back(start);

    bool foundPath = false;
    while(!foundPath && !openList.empty())
    {
        auto nodeIt = openList.end();
        float minF = std::numeric_limits<float>::max();
        for(auto it = openList.begin(); it != openList.end(); it++)
        {
            if((*it)->fCost < minF)
            {
                nodeIt = it;
                minF = (*it)->fCost;
            }
        }

        std::shared_ptr<Node> q = *nodeIt;
        openList.erase(nodeIt);

        for(int x = -1; x <= 1; x++)
        {
            for(int y = -1; y <= 1; y++)
            {
                if(x == 0 && y == 0)
                {
                    continue;
                }

                int newX = q->x + x;
                int newY = q->y + y;
                
                if(!isValid(m_GridMap, newX, newY))
                {
                    continue;
                }

                std::shared_ptr<Node> successor(new Node());
                successor->x = newX;
                successor->y = newY;
                successor->parent = q;

                if(*successor == *goal)
                {
                    goal->parent = q;
                    foundPath = true;
                    break;
                }

                successor->gCost = q->gCost + calculateH(successor->x, successor->y, *q);
                successor->hCost = calculateH(successor->x, successor->y, *goal);
                successor->fCost = successor->gCost + successor->hCost;

                bool foundBetter = false;
                for(const auto& node : openList)
                {
                    if(*node == *successor && node->fCost <= successor->fCost)
                    {
                        foundBetter = true;
                        break;
                    }
                }

                if(foundBetter)
                {
                    continue;
                }

                foundBetter = false;
                for(const auto& node : closedList)
                {
                    if(*node == *successor && node->fCost <= successor->fCost)
                    {
                        foundBetter = true;
                        break;
                    }
                }

                if(foundBetter)
                {
                    continue;
                }

                openList.push_back(successor);
            }

            if(foundPath)
            {
                break;
            }
        }

        closedList.push_back(q);
    }

    std::vector<Node> path;

    if(foundPath)
    {
        Node* current = goal.get();
        while(current != nullptr)
        {
            path.insert(path.begin(), *current);
            current = current->parent.get();
        }
    }

    return path;
}

void AStarGlobalPlanner::smoothPath(std::vector<Node>& path)
{
    if(path.empty())
    {
        return;
    }

    int currentIndex = 0;

    do
    {
        Node& from = path[currentIndex];
        Node& to = path[currentIndex + 2];

        if(isWalkable(m_GridMap, from, to))
        {
            path.erase(path.begin() + currentIndex + 1);
        }
        else
        {
            currentIndex++;
        }
    } 
    while (currentIndex < path.size() - 2);
    
}

bool AStarGlobalPlanner::isValid(const grid_map::GridMap& gridmap, int x, int y) 
{
    if(x < 0 || x >= gridmap.getSize()[0] || y < 0 || y >= gridmap.getSize()[1])
    {
        return false;
    }

    float elevation = gridmap.at("elevation", {x, y});

    return isnan(elevation) || elevation < 0.5f;
}

double AStarGlobalPlanner::calculateH(int x, int y, const Node& destination) 
{
    return sqrt((x - destination.x) * (x - destination.x) + (y - destination.y) * (y - destination.y));
}

bool AStarGlobalPlanner::isWalkable(const grid_map::GridMap& gridmap, const Node& start, const Node& end)
{    
    grid_map::Position startPosition;
    gridmap.getPosition({start.x, start.y}, startPosition);

    grid_map::Position endPosition;
    gridmap.getPosition({end.x, end.y}, endPosition);

    double granularity = gridmap.getResolution() / 2;
    int samples = (endPosition - startPosition).norm() / granularity;
    grid_map::Vector direction = (endPosition - startPosition).normalized();

    for(int i = 0; i < samples; i++)
    {
        grid_map::Position currentPos = startPosition + direction * granularity * i;

        grid_map::Index currentIndex;
        gridmap.getIndex(currentPos, currentIndex);
        if(!isValid(gridmap, currentIndex[0], currentIndex[1]))
        {
            return false;
        }
    }

    return true;
}
