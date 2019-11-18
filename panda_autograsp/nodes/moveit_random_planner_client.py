#!/usr/bin/env python
"""This ros node can be used to test out the different random planner services
present in the :py:mod:`moveit_random_planner_client.py`.
"""

#  Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

#  Main python packages
import sys

#  ROS python packages
import rospy

#  Panda_autograsp modules, msgs and srvs
from panda_autograsp.functions import yes_or_no
from panda_autograsp.srv import (
    ExecutePlan,
    VisualizePlan,
    PlanToRandomPath,
    PlanToRandomPoint,
)

#################################################
#  Main script  #################################
#################################################
if __name__ == "__main__":

    #  Initialize ros node
    rospy.init_node("moveit_planner_client", anonymous=True)
    rospy.loginfo("Initializing moveit_planner_client node")

    ###############################################
    #  Initialize moveit_planner server services ##
    ###############################################

    #  Initialize grasp_planning service
    rospy.loginfo("Connecting to moveit_planning_server service.")

    #  Initialize random pose service
    rospy.wait_for_service("moveit_planner_server/plan_random_pose")
    try:
        plan_to_random_pose_srv = rospy.ServiceProxy(
            "moveit_planner_server/plan_random_pose", PlanToRandomPoint
        )
    except rospy.ServiceException as e:
        rospy.logerr(
            "moveit_planner_server 'plan_random_pose' service initialization "
            "failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            plan_to_random_pose_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    #  Initialize random joint service
    rospy.wait_for_service("moveit_planner_server/plan_random_joint")
    try:
        plan_to_random_joint_srv = rospy.ServiceProxy(
            "moveit_planner_server/plan_random_joint", PlanToRandomPoint
        )
    except rospy.ServiceException as e:
        rospy.logerr(
            "moveit_planner_server 'plan_random_joint' service initialization "
            "failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            plan_to_random_joint_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    #  Initialize random cartesian path service
    rospy.wait_for_service("moveit_planner_server/plan_random_path")
    try:
        plan_to_random_path_srv = rospy.ServiceProxy(
            "moveit_planner_server/plan_random_path", PlanToRandomPath
        )
    except rospy.ServiceException as e:
        rospy.logerr(
            "moveit_planner_server 'plan_random_path' service initialization failed: %s"
            % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            plan_to_random_path_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    #  Initialize execute plan service
    rospy.wait_for_service("moveit_planner_server/execute_plan")
    try:
        execute_plan_srv = rospy.ServiceProxy(
            "moveit_planner_server/execute_plan", ExecutePlan
        )
    except rospy.ServiceException as e:
        rospy.logerr(
            "moveit_planner_server 'execute_plan' service initialization failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            execute_plan_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    #  Initialize plan visualization service
    rospy.wait_for_service("moveit_planner_server/visualize_plan")
    try:
        visualize_plan_srv = rospy.ServiceProxy(
            "moveit_planner_server/visualize_plan", VisualizePlan
        )
    except rospy.ServiceException as e:
        rospy.logerr(
            "Panda_autograsp 'visualize_plan' service initialization failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            visualize_plan_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    #  Print moveit services connection success message
    rospy.loginfo("Successfully connected to all moveit_planner_server services.")

    ###############################################
    #  Execute moveit plan services  ##############
    ###############################################
    rospy.loginfo("Wait for the moveit interface...")
    rospy.sleep(5)
    print(
        "\n== Random planner client ==\n"
        "This ros node can be used to test "
        "out the different random planner "
        "services present in the "
        " :py:mod:`moveit_random_planner_client.py`."
        "\n\n"
    )

    # Keep running the same protocol till exit
    while True:

        ###########################################
        #  Plan random pose  ######################
        ###########################################
        response = yes_or_no("Do you want to plan to a random pose?")
        while True:
            if not response:
                rospy.loginfo("Shutting down %s node." % rospy.get_name())
                sys.exit(0)
            else:
                result = plan_to_random_pose_srv()
                if not result:
                    yes_or_no("Random pose planning failed. Do you want to try again?")
                    if not response:
                        break
                else:
                    input(
                        "Random pose planning successful. Press enter to visualize "
                        "the plan>> "
                    )
                    visualize_plan_srv()
                    response = yes_or_no("Do you want execute the plan?")
                    if response:
                        result = execute_plan_srv()
                        if result:
                            break
                        else:
                            response = yes_or_no(
                                "Random pose execution failed. Do you want to try "
                                "again?"
                            )
                            if not response:
                                break

        ###########################################
        #  Plan random joint  #####################
        ###########################################
        response = yes_or_no("Do you want to plan to a random joint position ?")
        while True:
            if not response:
                rospy.loginfo("Shutting down %s node." % rospy.get_name())
                sys.exit(0)
            else:
                result = plan_to_random_joint_srv()
                if not result:
                    yes_or_no("Random joint planning failed. Do you want to try again?")
                    if not response:
                        break
                else:
                    input(
                        "Random joint planning successful. Press enter to visualize "
                        "the plan>> "
                    )
                    visualize_plan_srv()
                    response = yes_or_no("Do you want execute the plan?")
                    if response:
                        result = execute_plan_srv()
                        if result:
                            break
                        else:
                            response = yes_or_no(
                                "Random joint execution failed. Do you want to try "
                                "again?"
                            )
                            if not response:
                                break

        ###########################################
        #  Plan random path  ######################
        ###########################################
        response = yes_or_no("Do you want to plan to a random path?")
        while True:
            if not response:
                rospy.loginfo("Shutting down %s node." % rospy.get_name())
                sys.exit(0)
            else:
                result = plan_to_random_path_srv(n_waypoints=4)
                if not result:
                    yes_or_no("Random path planning failed. Do you want to try again?")
                    if not response:
                        break
                else:
                    input(
                        "Random path planning successful. Press enter to visualize "
                        "the plan>> "
                    )
                    visualize_plan_srv()
                    response = yes_or_no("Do you want execute the plan?")
                    if response:
                        result = execute_plan_srv()
                        if result:
                            break
                        else:
                            response = yes_or_no(
                                "Random path execution failed. Do you want to try "
                                "again?"
                            )
                            if not response:
                                break
