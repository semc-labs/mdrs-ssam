#!/usr/bin/env python

import actionlib
import copy

import rospy
import nav_msgs.srv as nav_srvs
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped


"""
move_base legacy relay node:
Relays old move_base actions to the new mbf move_base action, similar but with richer result and feedback.
We also relay the simple goal topic published by RViz, the make_plan service and dynamic reconfiguration
calls (note that some parameters have changed names; see http://wiki.ros.org/move_base_flex for details)
"""

# keep configured base local and global planners to send to MBF
bgp = None
blp = None


def simple_goal_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg, planner=bgp, controller=blp))
    rospy.logdebug("Relaying move_base_simple/goal pose to mbf")


def mb_execute_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg.target_pose, planner=bgp, controller=blp),
                        feedback_cb=mbf_feedback_cb)
    rospy.logdebug("Relaying legacy move_base goal to mbf")
    mbf_mb_ac.wait_for_result()

    status = mbf_mb_ac.get_state()
    result = mbf_mb_ac.get_result()

    rospy.logdebug("MBF execution completed with result [%d]: %s", result.outcome, result.message)
    if result.outcome == mbf_msgs.MoveBaseResult.SUCCESS:
        mb_as.set_succeeded(mb_msgs.MoveBaseResult(), "Goal reached.")
    else:
        mb_as.set_aborted(mb_msgs.MoveBaseResult(), result.message)


def make_plan_cb(request):
    mbf_gp_ac.send_goal(mbf_msgs.GetPathGoal(start_pose=request.start, target_pose=request.goal,
                                             use_start_pose=bool(request.start.header.frame_id),
                                             planner=bgp, tolerance=request.tolerance))
    rospy.logdebug("Relaying legacy make_plan service to mbf get_path action server")
    mbf_gp_ac.wait_for_result()

    status = mbf_gp_ac.get_state()
    result = mbf_gp_ac.get_result()

    rospy.logdebug("MBF get_path execution completed with result [%d]: %s", result.outcome, result.message)
    if result.outcome == mbf_msgs.GetPathResult.SUCCESS:
        return nav_srvs.GetPlanResponse(plan=result.path)


def mbf_feedback_cb(feedback):
    mb_as.publish_feedback(mb_msgs.MoveBaseFeedback(base_position=feedback.current_pose))


if __name__ == '__main__':
    rospy.init_node("move_base")

    # TODO what happens with malformed target goal???  FAILURE  or INVALID_POSE
    # txt must be:  "Aborting on goal because it was sent with an invalid quaternion"   

    # move_base_flex get_path and move_base action clients
    mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_gp_ac = actionlib.SimpleActionClient("move_base_flex/get_path", mbf_msgs.GetPathAction)
    mbf_mb_ac.wait_for_server(rospy.Duration(20))
    mbf_gp_ac.wait_for_server(rospy.Duration(10))


    # move_base simple topic and action server
    mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)
    mb_as = actionlib.SimpleActionServer('move_base', mb_msgs.MoveBaseAction, mb_execute_cb, auto_start=False)
    mb_as.start()

    # move_base make_plan service
    mb_mps = rospy.Service('~make_plan', nav_srvs.GetPlan, make_plan_cb)

    rospy.spin()