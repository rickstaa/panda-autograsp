#!/usr/bin/env python
"""This node sets up a number of services that can be used to control
the Panda Emika Franka robot. It uses the :py:class:`MoveitPlannerServer`
class to do this. The services that are set up by the ``moveit_planner_server``
node are listed below. It contains two types of services: services that work
on the main_move group (by default the panda_arm group) and services
that work on the gripper move group.

Services
---------

    General services
    -----------------
        - visualize_plan: Visualizes a given plan.

    Main_move_group services
    -------------------------

        - execute_plan: Executes the plan that has been computed by any of the other
        planner services.
        - plan_to_point: Plans to a given pose.
        - plan_to_joint: Plans to a sequence of joint angles.
        - plan_to_path: Plans for a given path.
        - plan_random_pose: Computes a plan to a random pose.
        - plan_random_joint: Computes a plan to a random sequence of joint angles.
        - plan_random_path: Copmutes a plan to a random path.

    Gripper move group services
    ----------------------------
        - open_gripper: Open the gripper. This service both plans and executes.
        - close_gripper: Close the gripper. This both plans and executes.
        - set_gripper_width: Set the gripper joint state targets to a
        certain width.
        - set_gripper_open: Set the gripper joint state targets to open.
        - set_gripper_closed: Set the gripper joint state targets to closed.
        - execute_gripper_plan: Execute a previously computed plan.
        - plan_gripper: Compute a plan for the currently set joint target. If no
        joint state targets are set no plan is computed.
"""

# Import ROS packages
import rospy
import sys

# Panda_autograsp modules, msgs and srvs
from panda_autograsp import MoveitPlannerServer

#################################################
# Main script####################################
#################################################
if __name__ == "__main__":

    # Init service node
    rospy.init_node("moveit_planner_server")

    # DEBUG: WAIT FOR PTVSD DEBUGGER #
    import ptvsd

    ptvsd.wait_for_attach()
    # ------------------------------ #

    # Create service object
    path_planning_service = MoveitPlannerServer(
        robot_description="robot_description",
        move_group="panda_arm",
        move_group_gripper="hand",
        pose_reference_frame="panda_link0",
        planner="TRRTkConfigDefault",
        args=sys.argv,
    )

    # Spin forever
    rospy.spin()
