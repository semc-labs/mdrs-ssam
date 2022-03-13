#ifndef SSAM_LATCHED_STOP_ROTATE_CONTROLLER
#define SSAM_LATCHED_STOP_ROTATE_CONTROLLER

#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_limits.h>

class LatchedStopRotateController
{
public:
    LatchedStopRotateController(base_local_planner::LocalPlannerLimits& limits, const std::string &name = "");
    virtual ~LatchedStopRotateController() {}

    bool isPositionReached(geometry_msgs::PoseStamped& goal_pose, const geometry_msgs::PoseStamped &global_pose);

    bool isGoalReached(geometry_msgs::PoseStamped& goal_pose, base_local_planner::OdometryHelperRos &odom_helper, const geometry_msgs::PoseStamped &global_pose);

    void resetLatching() { m_XyToleranceLatch = false; }

    bool stopWithAccLimits(const geometry_msgs::PoseStamped &global_pose,
                           const geometry_msgs::PoseStamped &robot_vel,
                           geometry_msgs::Twist &cmd_vel,
                           Eigen::Vector3f acc_lim,
                           double sim_period,
                           boost::function<bool(Eigen::Vector3f pos,
                                                Eigen::Vector3f vel,
                                                Eigen::Vector3f vel_samples)>
                               obstacle_check);

    bool rotateToGoal(const geometry_msgs::PoseStamped &global_pose,
                      const geometry_msgs::PoseStamped &robot_vel,
                      double goal_th,
                      geometry_msgs::Twist &cmd_vel,
                      Eigen::Vector3f acc_lim,
                      double sim_period,
                      base_local_planner::LocalPlannerLimits &limits,
                      boost::function<bool(Eigen::Vector3f pos,
                                           Eigen::Vector3f vel,
                                           Eigen::Vector3f vel_samples)>
                          obstacle_check);

    bool computeVelocityCommandsStopRotate(geometry_msgs::Twist &cmd_vel,
                                           Eigen::Vector3f acc_lim,
                                           double sim_period,
                                           geometry_msgs::PoseStamped& goal_pose,
                                           base_local_planner::OdometryHelperRos &odom_helper,
                                           const geometry_msgs::PoseStamped &global_pose,
                                           boost::function<bool(Eigen::Vector3f pos,
                                                                Eigen::Vector3f vel,
                                                                Eigen::Vector3f vel_samples)>
                                               obstacle_check);

private:
    inline double sign(double x)
    {
        return x < 0.0 ? -1.0 : 1.0;
    }

    // whether to latch at all, and whether in this turn we have already been in goal area
    bool m_LatchXyGoalTolerance;
    bool m_XyToleranceLatch;
    bool m_RotatingToGoal;

    base_local_planner::LocalPlannerLimits& m_Limits;
};

#endif //SSAM_LATCHED_STOP_ROTATE_CONTROLLER