#!/usr/bin/env python
"""This node calls the ROS GQCNN message service.
"""

## Make script both python2 and python3 compatible ##
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

## Import standard library packages ##
import sys

## Import ROS python packages ##
import rospy

## Import ROS services and messages ##
from panda_autograsp.srv import (ExecutePlan, PlanToJoint, PlanToPath, PlanToPoint, PlanToRandomPath, PlanToRandomPoint)

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Initialize ros node ##
    rospy.loginfo("Initializing moveit_planner_client node")
    rospy.init_node('moveit_planner_client', anonymous=True)

    ## Initialize grasp_planning service ##
    rospy.loginfo("Conneting to moveit_planning_server service.")
    rospy.wait_for_service("/plan_random_pose")
    try:
        plan_to_random_pose_srv = rospy.ServiceProxy(
            "/plan_random_pose", PlanToRandomPoint)
    except rospy.ServiceException as e:
        rospy.loginfo("Service initialization failed: %s" % e)
        shutdown_msg = "Shutting down %s node because %s connection failed." % (
            rospy.get_name(), plan_to_random_pose_srv)
        rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

    plan_to_random_pose_srv((0,3),(0,3),(0,3))

    ## Loop till the service is shutdown. ##
    rospy.spin()
