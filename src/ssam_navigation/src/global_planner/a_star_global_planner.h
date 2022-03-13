#ifndef SSAM_A_STAR_GLOBAL_PLANNER
#define SSAM_A_STAR_GLOBAL_PLANNER

#include <mbf_abstract_core/abstract_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>

class AStarGlobalPlanner : public mbf_abstract_core::AbstractPlanner
{
    struct Node
    {
        Node() = default;
        Node(const Node& other)
            : x(other.x)
            , y(other.y)
        {  
        }

        int y;
        int x;
        
        std::shared_ptr<Node> parent;

        float gCost{ 0.0f };
        float hCost{ 0.0f }; 
        float fCost{ 0.0f };

        bool operator==(const Node& other) const
        {
            return x == other.x && y == other.y;
        }
    };

public:
    AStarGlobalPlanner(const grid_map::GridMap& gridMap) : m_GridMap(gridMap) 
    {
        ros::NodeHandle privateNh("~/TestGlobalPlanner");
        m_PlanPublisher = privateNh.advertise<nav_msgs::Path>("plan", 1);
    }

    uint32_t makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message) override;
    bool cancel() override { return false; }
    
private:
    std::vector<Node> aStar(std::shared_ptr<Node>& start, std::shared_ptr<Node>& goal);
    void smoothPath(std::vector<Node>& path);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path); 

    static double calculateH(int x, int y, const Node& destination);
    static bool isValid(const grid_map::GridMap& gridmap, int x, int y);
    static bool isWalkable(const grid_map::GridMap& gridmap, const Node& start, const Node& end);

private:
    bool m_Initialized{ false };
    const grid_map::GridMap& m_GridMap;

    ros::Publisher m_PlanPublisher;
};

#endif //SSAM_A_STAR_GLOBAL_PLANNER