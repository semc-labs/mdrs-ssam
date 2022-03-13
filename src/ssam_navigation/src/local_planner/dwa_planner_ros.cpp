#include "dwa_planner_ros.h"

#include <base_local_planner/goal_functions.h>
#include <tf2/utils.h>

void DWAPlannerROS::reconfigureCB(dwa_local_planner::DWAPlannerConfig &config, uint32_t level)
{
    if (m_Setup && config.restore_defaults)
    {
        config = m_DefaultConfig;
        config.restore_defaults = false;
    }
    if (!m_Setup)
    {
        m_DefaultConfig = config;
        m_Setup = true;
    }

    // update generic local planner params
    m_Limits.max_vel_trans = config.max_vel_trans;
    m_Limits.min_vel_trans = config.min_vel_trans;
    m_Limits.max_vel_x = config.max_vel_x;
    m_Limits.min_vel_x = config.min_vel_x;
    m_Limits.max_vel_y = config.max_vel_y;
    m_Limits.min_vel_y = config.min_vel_y;
    m_Limits.max_vel_theta = config.max_vel_theta;
    m_Limits.min_vel_theta = config.min_vel_theta;
    m_Limits.acc_lim_x = config.acc_lim_x;
    m_Limits.acc_lim_y = config.acc_lim_y;
    m_Limits.acc_lim_theta = config.acc_lim_theta;
    m_Limits.acc_lim_trans = config.acc_lim_trans;
    m_Limits.xy_goal_tolerance = config.xy_goal_tolerance;
    m_Limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
    m_Limits.prune_plan = config.prune_plan;
    m_Limits.trans_stopped_vel = config.trans_stopped_vel;
    m_Limits.theta_stopped_vel = config.theta_stopped_vel;

    // update dwa specific configuration
    m_Planner->reconfigure(config);
}

DWAPlannerROS::DWAPlannerROS() 
    : m_Initialized(false)
    , m_OdomHelper("odom")
    , m_Setup(false)
    , m_LatchedStopRotateController(m_Limits)
{
}

void DWAPlannerROS::initialize(std::string name, grid_map::GridMap* gridmap)
{
    if (!isInitialized())
    {
        ros::NodeHandle private_nh("~/" + name);
        m_GlobalPlanPublisher = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        m_LocalPlanPublisher = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        m_Gridmap = gridmap;

        // make sure to update the costmap we'll use for this cycle

        //create the actual planner that we'll use.. it'll configure itself from the parameter server
        m_Planner = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, *m_Gridmap, m_Limits));

        if (private_nh.getParam("odom_topic", m_OdomTopic))
        {
            m_OdomHelper.setOdomTopic(m_OdomTopic);
        }

        m_Initialized = true;

        // Warn about deprecated parameters -- remove this block in N-turtle
        
        m_ReconfigureServer = new dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
        m_ReconfigureServer->setCallback(cb);

        dwa_local_planner::DWAPlannerConfig tempConfig;
        tempConfig.max_vel_trans = 0.22;
        tempConfig.min_vel_trans = -0.22;
        tempConfig.max_vel_x = 0.22;
        tempConfig.min_vel_x = -0.22;
        tempConfig.max_vel_y = 0.0;
        tempConfig.min_vel_y = 0.0;
        tempConfig.max_vel_theta = 2.75;
        tempConfig.min_vel_theta = 1.37;
        tempConfig.acc_lim_x = 2.5;
        tempConfig.acc_lim_y = 0.0;
        tempConfig.acc_lim_theta = 3.2;
        tempConfig.acc_lim_trans = 2.5;
        //tempConfig.prune_plan;
        tempConfig.xy_goal_tolerance = 0.05;
        tempConfig.yaw_goal_tolerance = 0.17;
        //tempConfig.trans_stopped_vel;
        //tempConfig.theta_stopped_vel;
        tempConfig.sim_time = 10;
        tempConfig.sim_granularity = 0.1;
        tempConfig.angular_sim_granularity = 0.1;
        tempConfig.path_distance_bias = 32.0;
        tempConfig.goal_distance_bias = 20.0;
        tempConfig.occdist_scale = 0.02;
        //tempConfig.twirling_scale;
        tempConfig.stop_time_buffer = 0.2;
        tempConfig.oscillation_reset_dist = 0.05;
        //tempConfig.oscillation_reset_angle;
        tempConfig.forward_point_distance = 0.325;
        tempConfig.scaling_speed = 0.25;
        tempConfig.max_scaling_factor = 0.2;
        tempConfig.vx_samples = 20;
        tempConfig.vy_samples = 10;
        tempConfig.vth_samples = 40;
        tempConfig.use_dwa = true;

        reconfigureCB(tempConfig, 0);
    }
    else
    {
        ROS_WARN("This planner has already been initialized, doing nothing.");
    }
}

bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
{
    if (!isInitialized())
    {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    m_LatchedStopRotateController.resetLatching();
    m_CurrentGoal = orig_global_plan.back();
    m_CurrentPlan = orig_global_plan;

    ROS_INFO("Got new plan");
    return m_Planner->setPlan(orig_global_plan);
}

bool DWAPlannerROS::isGoalReached(double xy_tolerance, double yaw_tolerance)
{
    return m_GoalReached;
}

void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path)
{
    base_local_planner::publishPlan(path, m_LocalPlanPublisher);
}

void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
{
    base_local_planner::publishPlan(path, m_GlobalPlanPublisher);
}

DWAPlannerROS::~DWAPlannerROS()
{
    //make sure to clean things up
    delete m_ReconfigureServer;
}

bool DWAPlannerROS::dwaComputeVelocityCommands(const geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist &cmd_vel)
{
    // dynamic window sampling approach to get useful velocity commands
    if (!isInitialized())
    {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    geometry_msgs::PoseStamped robot_vel;
    m_OdomHelper.getRobotVel(robot_vel);

    //compute what trajectory to drive along
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = "base_link";

    // call with updated footprint
    base_local_planner::Trajectory path = m_Planner->findBestPath(global_pose, robot_vel, drive_cmds);

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if (path.cost_ < 0)
    {
        ROS_DEBUG_NAMED("dwa_local_planner",
                        "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
        local_plan.clear();
        publishLocalPlan(local_plan);
        return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i)
    {
        double p_x, p_y, p_th;
        path.getPoint(i, p_x, p_y, p_th);

        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.header.stamp = ros::Time::now();
        p.pose.position.x = p_x;
        p.pose.position.y = p_y;
        p.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, p_th);
        tf2::convert(q, p.pose.orientation);
        local_plan.push_back(p);
    }

    //publish information to the visualizer

    publishLocalPlan(local_plan);
    return true;
}

uint32_t DWAPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity, geometry_msgs::TwistStamped &cmd_vel, std::string &message)
{
    std::vector<geometry_msgs::PoseStamped> transformed_plan = m_CurrentPlan;

    //if the global plan passed in is empty... we won't do anything
    if (transformed_plan.empty())
    {
        ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
        return 100;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    //[[0.01, 0.01], [-0.01, 0.01], [-0.01, -0.01], [0.01, -0.01]]
    geometry_msgs::Point footprintPoint1;
    geometry_msgs::Point footprintPoint2;
    geometry_msgs::Point footprintPoint3;
    geometry_msgs::Point footprintPoint4;

    footprintPoint1.x = 0.01;
    footprintPoint1.y = 0.01;

    footprintPoint2.x = -0.01;
    footprintPoint2.y = 0.01;

    footprintPoint3.x = -0.01;
    footprintPoint3.y = -0.01;

    footprintPoint4.x = 0.01;
    footprintPoint4.y = -0.01;

    std::vector<geometry_msgs::Point> footprint;
    footprint.push_back(footprintPoint1);
    footprint.push_back(footprintPoint2);
    footprint.push_back(footprintPoint3);
    footprint.push_back(footprintPoint4);

    m_Planner->updatePlanAndLocalCosts(pose, transformed_plan, footprint);

    if (m_LatchedStopRotateController.isPositionReached(m_CurrentGoal, pose))
    {
        //publish an empty plan because we've reached our goal position
        std::vector<geometry_msgs::PoseStamped> local_plan;
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        publishGlobalPlan(transformed_plan);
        publishLocalPlan(local_plan);
        return m_LatchedStopRotateController.computeVelocityCommandsStopRotate(
            cmd_vel.twist,
            m_Limits.getAccLimits(),
            m_Planner->getSimPeriod(),
            m_CurrentGoal,
            m_OdomHelper,
            pose,
            boost::bind(&DWAPlanner::checkTrajectory, m_Planner, _1, _2, _3));
    }
    else
    {
        bool isOk = dwaComputeVelocityCommands(pose, cmd_vel.twist);
        if (isOk)
        {
            publishGlobalPlan(transformed_plan);
        }
        else
        {
            ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
            std::vector<geometry_msgs::PoseStamped> empty_plan;
            publishGlobalPlan(empty_plan);
        }
        return (isOk ? 0 : 100);
    }
}