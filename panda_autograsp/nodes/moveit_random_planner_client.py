#!/usr/bin/env python
"""This node calls the ROS GQCNN message service.
"""

## Make script both python2 and python3 compatible ##
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

## Import standard library packages ##
import sys
import tty
import sys
import termios

## Import ROS python packages ##
import rospy

## Import ROS services and messages ##
from panda_autograsp.srv import (ExecutePlan, VisualizePlan, PlanToJoint, PlanToPath, PlanToPoint, PlanToRandomPath, PlanToRandomPoint)

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Initialize ros node ##
    rospy.loginfo("Initializing moveit_planner_client node")
    rospy.init_node('moveit_planner_client', anonymous=True)

    ## Initialize grasp_planning service ##
    rospy.loginfo("Conneting to moveit_planning_server service.")

    ## Initialize random pose service ##
    rospy.wait_for_service("moveit/plan_random_pose")
    try:
        plan_to_random_pose_srv = rospy.ServiceProxy(
            "moveit/plan_random_pose", PlanToRandomPoint)
    except rospy.ServiceException as e:
        rospy.loginfo("Service initialization failed: %s" % e)
        shutdown_msg = "Shutting down %s node because %s connection failed." % (
            rospy.get_name(), plan_to_random_pose_srv)
        rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

    ## Initialize random joint service ##
    rospy.wait_for_service("moveit/plan_random_joint")
    try:
        plan_to_random_joint_srv = rospy.ServiceProxy(
            "moveit/plan_random_joint", PlanToRandomPoint)
    except rospy.ServiceException as e:
        rospy.loginfo("Service initialization failed: %s" % e)
        shutdown_msg = "Shutting down %s node because %s connection failed." % (
            rospy.get_name(), plan_to_random_joint_srv)
        rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

    ## Initialize random cartesian path service ##
    rospy.wait_for_service("moveit/plan_random_path")
    try:
        plan_to_random_path_srv = rospy.ServiceProxy(
            "moveit/plan_random_path", PlanToRandomPath)
    except rospy.ServiceException as e:
        rospy.loginfo("Service initialization failed: %s" % e)
        shutdown_msg = "Shutting down %s node because %s connection failed." % (
            rospy.get_name(), plan_to_random_path_srv)
        rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

    ## Initialize execute plan service ##
    rospy.wait_for_service("moveit/execute_plan")
    try:
        execute_plan_srv = rospy.ServiceProxy(
            "moveit/execute_plan", ExecutePlan)
    except rospy.ServiceException as e:
        rospy.loginfo("Service initialization failed: %s" % e)
        shutdown_msg = "Shutting down %s node because %s connection failed." % (
            rospy.get_name(), execute_plan_srv)
        rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

    ## Initialize plan visualization service ##
    rospy.wait_for_service("moveit/visualize_plan")
    try:
        visualize_plan_srv = rospy.ServiceProxy(
            "moveit/visualize_plan", VisualizePlan)
    except rospy.ServiceException as e:
        rospy.loginfo("Service initialization failed: %s" % e)
        shutdown_msg = "Shutting down %s node because %s connection failed." % (
            rospy.get_name(), visualize_plan_srv)
        rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

    ## Plan random pose ##
    plan_to_random_pose_srv()
    execute_plan_srv()

    ## Plan random joint ##
    plan_to_random_joint_srv()
    execute_plan_srv()

    ## Plan random cartesian trajectory ##
    prompt_result = raw_input(
            "Do you want to start a random trajectory planning [Y/n]? ")
    while True:

        # Check user input #
        # If yes start planning
        if (prompt_result.lower() in ['y', 'yes']) or (prompt_result == ""):
            result = plan_to_random_path_srv(n_waypoints=4)

            ## Check if planning was successful ##
            if not result:
                while True:
                    prompt_result2 = raw_input("No successful trajectory was found. Do you want to try again [Y/n]? ")

                    ## Check prompt 2 ##
                    if (prompt_result2.lower() in ['y', 'yes']) or (prompt_result2 == ""):
                        break
                    elif prompt_result.lower() in ['n', 'no']:
                        print("shutdown")
                        sys.exit(0)
                    else:
                        print(
                            prompt_result + " is not a valid response please answer with Y or N to continue. ")
            else:
                break
        elif prompt_result.lower() in ['n', 'no']:
            print("shutdown")
            sys.exit(0)
        else:
            print(
                prompt_result + " is not a valid response please answer with Y or N to continue.")

    ## Visualize plan ##
    prompt_result = raw_input("A successful plan was found press enter to visualize: ")
    visualize_plan_srv()

    ## Execute plan ##
    while True:
        prompt_result = raw_input(
            "Do you want to execute the planned trajectory [Y/n]? ")
        if (prompt_result.lower() in ['y', 'yes']) or (prompt_result == ""):
            execute_plan_srv()
            break
        elif prompt_result.lower() in ['n', 'no']:
            print("shutdown")
            sys.exit(0)
        else:
            print(
                prompt_result + " is not a valid response please answer with Y or N to continue.")

    ## Loop till the service is shutdown. ##
    rospy.spin()
