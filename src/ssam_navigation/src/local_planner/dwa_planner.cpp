#include "dwa_planner.h"

#include <tf2/utils.h>
#include <sensor_msgs/point_cloud2_iterator.h>

void DWAPlanner::reconfigure(dwa_local_planner::DWAPlannerConfig& config)
{
    boost::mutex::scoped_lock l(m_ConfigurationMutex);

    m_Generator.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        m_SimPeriod);

    double resolution = 0.2;
    m_PathDistanceBias = resolution * config.path_distance_bias;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    m_PathCosts.setScale(m_PathDistanceBias);
    m_AlignmentCosts.setScale(m_PathDistanceBias);

    m_GoalDistanceBias = resolution * config.goal_distance_bias;
    m_GoalCosts.setScale(m_GoalDistanceBias);
    m_GoalFrontCosts.setScale(m_GoalDistanceBias);

    m_OccdistScale = config.occdist_scale;
    m_ObstacleCosts.setScale(m_OccdistScale);

    m_StopTimeBuffer = config.stop_time_buffer;
    m_OscillationCosts.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    m_ForwardPointDistance = config.forward_point_distance;
    m_GoalFrontCosts.setXShift(m_ForwardPointDistance);
    m_AlignmentCosts.setXShift(m_ForwardPointDistance);

    // obstacle costs can vary due to scaling footprint feature
    m_ObstacleCosts.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);

    m_TwirlingCosts.setScale(config.twirling_scale);

    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;

    if (vx_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
        vx_samp = 1;
        config.vx_samples = vx_samp;
    }

    if (vy_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
        vy_samp = 1;
        config.vy_samples = vy_samp;
    }

    if (vth_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
        vth_samp = 1;
        config.vth_samples = vth_samp;
    }

    m_VSamples[0] = vx_samp;
    m_VSamples[1] = vy_samp;
    m_VSamples[2] = vth_samp;
}

DWAPlanner::DWAPlanner(std::string name, grid_map::GridMap& gridmap, base_local_planner::LocalPlannerLimits& limits)
    : m_Gridmap(gridmap)
    , m_Limits(limits)
    , m_ObstacleCosts(gridmap)
    , m_PathCosts(gridmap)
    , m_GoalCosts(gridmap, 0.0, 0.0, true)
    , m_GoalFrontCosts(gridmap, 0.0, 0.0, true)
    , m_AlignmentCosts(gridmap)
{
    ros::NodeHandle private_nh("~/" + name);

    m_GoalFrontCosts.setStopOnFailure( false );
    m_AlignmentCosts.setStopOnFailure( false );

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
        m_SimPeriod = 0.05;
    } else {
        double controller_frequency = 0;
        private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        if(controller_frequency > 0) {
        m_SimPeriod = 1.0 / controller_frequency;
        } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        m_SimPeriod = 0.05;
        }
    }
    ROS_INFO("Sim period is set to %.2f", m_SimPeriod);

    m_OscillationCosts.resetOscillationFlags();

    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    m_ObstacleCosts.setSumScores(sum_scores);

    private_nh.param("publish_cost_grid_pc", m_PublishCostGridPc, false);

    private_nh.param("global_frame_id", m_FrameId, std::string("odom"));

    m_TrajCloudPublisher = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", m_PublishTrajPc, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&m_OscillationCosts); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&m_ObstacleCosts); // discards trajectories that move into obstacles
    critics.push_back(&m_GoalFrontCosts); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&m_AlignmentCosts); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&m_PathCosts); // prefers trajectories on global path
    critics.push_back(&m_GoalCosts); // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&m_TwirlingCosts); // optionally prefer trajectories that don't spin

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&m_Generator);

    m_ScoredSamplingPlanner = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", m_CheatFactor, 1.0);
}

bool DWAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) 
{
    m_OscillationCosts.resetOscillationFlags();
    
    return true;
}

bool DWAPlanner::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)
{
    m_OscillationCosts.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = m_GlobalPlan.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    m_Generator.initialise(pos, vel, goal, &m_Limits, m_VSamples);
    m_Generator.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = m_ScoredSamplingPlanner.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) 
    {
        return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
}

void DWAPlanner::updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan, const std::vector<geometry_msgs::Point>& footprint_spec) 
{
    m_GlobalPlan.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
        m_GlobalPlan[i] = new_plan[i];
    }

    m_ObstacleCosts.setFootprint(footprint_spec);

    // costs for going away from path
    m_PathCosts.setTargetPoses(m_GlobalPlan);

    // costs for not going towards the local goal as much as possible
    m_GoalCosts.setTargetPoses(m_GlobalPlan);

    // alignment costs
    geometry_msgs::PoseStamped goal_pose = m_GlobalPlan.back();

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<geometry_msgs::PoseStamped> front_global_plan = m_GlobalPlan;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
        m_ForwardPointDistance * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + m_ForwardPointDistance *
        sin(angle_to_goal);

    m_GoalFrontCosts.setTargetPoses(front_global_plan);

    // keeping the nose on the path
    if (sq_dist > m_ForwardPointDistance * m_ForwardPointDistance * m_CheatFactor) {
        m_AlignmentCosts.setScale(m_PathDistanceBias);
        // costs for robot being aligned with path (nose on path, not ju
        m_AlignmentCosts.setTargetPoses(m_GlobalPlan);
    } else {
        // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
        m_AlignmentCosts.setScale(0.0);
    }
}

/*
* given the current state of the robot, find a good trajectory
*/
base_local_planner::Trajectory DWAPlanner::findBestPath(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities) 
{
    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(m_ConfigurationMutex);

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    geometry_msgs::PoseStamped goal_pose = m_GlobalPlan.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));

    // prepare cost functions and generators for this run
    m_Generator.initialise(pos, vel, goal, &m_Limits, m_VSamples);

    m_ResultTrajectory.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<base_local_planner::Trajectory> all_explored;
    m_ScoredSamplingPlanner.findBestTrajectory(m_ResultTrajectory, &all_explored);

    if(m_PublishTrajPc)
    {
        sensor_msgs::PointCloud2 traj_cloud;
        traj_cloud.header.frame_id = m_FrameId;
        traj_cloud.header.stamp = ros::Time::now();

        sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
        cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                            "y", 1, sensor_msgs::PointField::FLOAT32,
                                            "z", 1, sensor_msgs::PointField::FLOAT32,
                                            "theta", 1, sensor_msgs::PointField::FLOAT32,
                                            "cost", 1, sensor_msgs::PointField::FLOAT32);

        unsigned int num_points = 0;
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if (t->cost_<0)
                continue;
            num_points += t->getPointsSize();
        }

        cloud_mod.resize(num_points);
        sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if(t->cost_<0)
                continue;
            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                iter_x[0] = p_x;
                iter_x[1] = p_y;
                iter_x[2] = 0.0;
                iter_x[3] = p_th;
                iter_x[4] = t->cost_;
                ++iter_x;
            }
        }
        m_TrajCloudPublisher.publish(traj_cloud);
    }

    // debrief stateful scoring functions
    m_OscillationCosts.updateOscillationFlags(pos, &m_ResultTrajectory, m_Limits.min_vel_trans);

    //if we don't have a legal trajectory, we'll just command zero
    if (m_ResultTrajectory.cost_ < 0) {
        drive_velocities.pose.position.x = 0;
        drive_velocities.pose.position.y = 0;
        drive_velocities.pose.position.z = 0;
        drive_velocities.pose.orientation.w = 1;
        drive_velocities.pose.orientation.x = 0;
        drive_velocities.pose.orientation.y = 0;
        drive_velocities.pose.orientation.z = 0;
    } else {
        drive_velocities.pose.position.x = m_ResultTrajectory.xv_;
        drive_velocities.pose.position.y = m_ResultTrajectory.yv_;
        drive_velocities.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, m_ResultTrajectory.thetav_);
        tf2::convert(q, drive_velocities.pose.orientation);
    }

    return m_ResultTrajectory;
}