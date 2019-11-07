#!/usr/bin/env python
"""This node contains the command line interface (CLI) of the panda_autograsp
autonomous grasping solution.
"""

# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# Main python packages
import rospy
import sys

# Panda_autograsp modules, msgs and srvs
from panda_autograsp.srv import (
    ComputeGrasp,
    PlanGrasp,
    VisualizeGrasp,
    ExecuteGrasp,
    CalibrateSensor,
    SetGripperOpen,
    SetGripperClosed,
    PlanGripper,
    ExecuteGripperPlan,
    CloseGripper,
    SetGripperWidth,
)
from panda_autograsp.functions import yes_or_no


#################################################
# PandaAutograspCLI class #######################
#################################################
class PandaAutograspCLI:
    """Panda_autograsp command line interface class."""

    def __init__(self):
        """Panda_autograsp command line interface initiation"""

        #########################################
        # Welcome message #######################
        #########################################
        print("----- PANDA_AUTOGRASP_PACKAGE_CLI -----")
        print("| Welcome to the panda_autograsp cli. |")
        print("---------------------------------------")
        print("| INFO: Press ctrl+D, or ctrl+c+enter |")
        print("| to close the client                 |")
        print("---------------------------------------")
        print("")

        #########################################
        # Initialize needed services ############
        #########################################

        # Initialize camera calibration service
        rospy.loginfo("Connecting to the panda_autograsp services...")
        rospy.logdebug("Connecting to 'calibrate_sensor' service...")
        rospy.wait_for_service("calibrate_sensor")
        try:
            self._calibrate_sensor_srv = rospy.ServiceProxy(
                "calibrate_sensor", CalibrateSensor
            )
            rospy.logdebug("Connected to 'calibrate_sensor' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'calibrate_sensor' service initialization failed: %s"
                % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._calibrate_sensor_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize request grasp service
        rospy.logdebug("Connecting to 'compute_grasp' service...")
        rospy.wait_for_service("compute_grasp")
        try:
            self._compute_grasp_srv = rospy.ServiceProxy("compute_grasp", ComputeGrasp)
            rospy.logdebug("Connected to 'compute_grasp' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'compute_grasp' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._compute_grasp_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner server services
        rospy.logdebug("Connecting to 'plan_grasp' service...")
        rospy.wait_for_service("plan_grasp")
        try:
            self._plan_grasp_srv = rospy.ServiceProxy("plan_grasp", PlanGrasp)
            rospy.logdebug("Connected to 'plan_grasp' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'plan_grasp' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._plan_grasp_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize plan visualization service
        rospy.logdebug("Connecting to 'visualize_grasp' service...")
        rospy.wait_for_service("visualize_grasp")
        try:
            self._visualize_grasp_srv = rospy.ServiceProxy(
                "visualize_grasp", VisualizeGrasp
            )
            rospy.logdebug("Connected to 'visualize_grasp' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'visualize_grasp' service initialization failed: %s"
                % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._visualize_grasp_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize execute plan service
        rospy.logdebug("Connecting to 'execute_grasp' service...")
        rospy.wait_for_service("execute_grasp")
        try:
            self._execute_grasp_srv = rospy.ServiceProxy("execute_grasp", ExecuteGrasp)
            rospy.logdebug("Connected to 'execute_grasp' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'execute_grasp' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._execute_grasp_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize plan_place service
        rospy.logdebug("Connecting to 'plan_place' service...")
        rospy.wait_for_service("plan_place")
        try:
            self._plan_place_srv = rospy.ServiceProxy("plan_place", CloseGripper)
            rospy.logdebug("Connected to 'plan_place' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'plan_place' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._plan_place_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner_server/set_gripper_open service
        rospy.logdebug(
            "Connecting to 'moveit_planner_server/set_gripper_open' service..."
        )
        rospy.wait_for_service("moveit_planner_server/set_gripper_open")
        try:
            self._set_gripper_open_srv = rospy.ServiceProxy(
                "moveit_planner_server/set_gripper_open", SetGripperOpen
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/set_gripper_open' service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'set_gripper_open' service initialization failed: %s"
                % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._set_gripper_open_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner_server/set_gripper_closed service
        rospy.logdebug(
            "Connecting to 'moveit_planner_server/set_gripper_closed' service..."
        )
        rospy.wait_for_service("moveit_planner_server/set_gripper_closed")
        try:
            self._set_gripper_closed_srv = rospy.ServiceProxy(
                "moveit_planner_server/set_gripper_closed", SetGripperClosed
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/set_gripper_closed' service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/set_gripper_closed' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._set_gripper_closed_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner_server/plan_gripper service
        rospy.logdebug("Connecting to 'moveit_planner_server/plan_gripper' service...")
        rospy.wait_for_service("moveit_planner_server/plan_gripper")
        try:
            self._plan_gripper_srv = rospy.ServiceProxy(
                "moveit_planner_server/plan_gripper", PlanGripper
            )
            rospy.logdebug("Connected to 'moveit_planner_server/plan_gripper' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/plan_gripper' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._plan_gripper_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner_server/execute_gripper_plan service
        rospy.logdebug(
            "Connecting to 'moveit_planner_server/execute_gripper_plan' " "service..."
        )
        rospy.wait_for_service("moveit_planner_server/execute_gripper_plan")
        try:
            self._execute_gripper_plan_srv = rospy.ServiceProxy(
                "moveit_planner_server/execute_gripper_plan", ExecuteGripperPlan
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/execute_gripper_plan' " "service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/execute_gripper_plan' service "
                "initialization "
                "failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._execute_gripper_plan_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner_server/close_gripper service
        rospy.logdebug("Connecting to 'moveit_planner_server/close_gripper' service...")
        rospy.wait_for_service("moveit_planner_server/close_gripper")
        try:
            self._close_gripper_srv = rospy.ServiceProxy(
                "moveit_planner_server/close_gripper", CloseGripper
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/close_gripper' " "service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/close_gripper' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._close_gripper_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner_server/open_gripper service
        rospy.logdebug("Connecting to 'moveit_planner_server/open_gripper' service...")
        rospy.wait_for_service("moveit_planner_server/open_gripper")
        try:
            self._open_gripper_srv = rospy.ServiceProxy(
                "moveit_planner_server/open_gripper", CloseGripper
            )
            rospy.logdebug("Connected to 'moveit_planner_server/open_gripper' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/open_gripper' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._open_gripper_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner_server/set_gripper_width service
        rospy.logdebug(
            "Connecting to 'moveit_planner_server/set_gripper_width' " "service..."
        )
        rospy.wait_for_service("moveit_planner_server/set_gripper_width")
        try:
            self._set_gripper_width_srv = rospy.ServiceProxy(
                "moveit_planner_server/set_gripper_width", SetGripperWidth
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/set_gripper_width' " "service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/set_gripper_width' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._set_gripper_width_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Log success message
        rospy.loginfo("Connected to the panda_autograsp services.")

    def start(self):
        "Start command line interface."

        # Keep alive as node is alive
        print("")
        input(
            "For the robot to know where it is relative to the camera we need "
            "to perform a calibration. Press enter to start the calibration "
            "procedure>> "
        )
        try:  # Catch user and service exceptions

            #####################################
            # Calibrate Camera frame ############
            #####################################
            while True:
                print("")
                response = yes_or_no(
                    "Is the checkerboard/arucoboard positioned on the upper left "
                    "corner of the table?"
                )
                print("Computing calibration frame transform...")
                if not response:
                    rospy.loginfo("Shutting down %s node." % rospy.get_name())
                    sys.exit(0)
                else:
                    result = self._calibrate_sensor_srv()
                    if not result.success:
                        print("")
                        response = yes_or_no(
                            "Calibration failed. Do you want to try again?"
                        )
                        if not response:
                            rospy.loginfo("Shutting down %s node." % rospy.get_name())
                            sys.exit(0)
                    else:
                        print("")
                        response = yes_or_no("Was the frame correct?")
                        if response:
                            break  # Continue to grasp planning

            #####################################
            # Panda_autograsp_cli loop ##########
            #####################################
            # Keep cli running until ros is shutdown
            while not rospy.is_shutdown():

                #################################
                # Compute grasp #################
                #################################
                print("")
                input("Click enter to compute a grasp>> ")
                while True:
                    print("Computing grasp...")
                    result = self._compute_grasp_srv()

                    # Display Grasp result message
                    if not result.success:
                        print("")
                        response = yes_or_no(
                            "Grasp computation failed. Do you want to try again?"
                        )
                        if not response:
                            rospy.loginfo("Shutting down %s node." % rospy.get_name())
                            sys.exit(0)
                    else:
                        print("")
                        response = yes_or_no(
                            "A valid grasp was found. Do you want to plan for the "
                            "computed grasp? (Y=Continue and n=Try again)>> ",
                            add_options=False,
                        )
                        if response:
                            break  # Plan grasp

                #################################
                # Plan grasp ####################
                #################################
                print("Planning grasp...")
                result1 = (
                    self._set_gripper_open_srv()
                )  # Set gripper joint states to open
                result2 = (
                    self._plan_gripper_srv()
                )  # Plan for the set gripper joint states
                result3 = self._plan_grasp_srv()

                # Check if grasp planning was successfull
                if (
                    (not result1.success)
                    or (not result2.success)
                    or (not result3.success)
                ):

                    # Display plan failed message
                    if not result3.success:
                        print("")
                        response = yes_or_no(
                            "Grasp path planning failed. Because no arm plan "
                            "was found. Do you want to compute "
                            "another grasp?"
                        )
                    elif not result2.success:
                        print("")
                        response = yes_or_no(
                            "Grasp path planning failed because no gripper plan "
                            "was found. Do you want to compute "
                            "another grasp?"
                        )
                    else:
                        print("")
                        response = yes_or_no(
                            "Grasp path planning failed because griper open target "
                            "could not be set. Do you want to compute "
                            "another grasp?"
                        )

                    # Check user response
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    input(
                        "Grasp path planning successfull. Press enter to "
                        "visualize the computed path>> "
                    )

                #################################
                # visualize grasp ###############
                #################################
                print("Visualizing grasp path...")
                result = self._visualize_grasp_srv()
                if not result.success:

                    # Display visualization failed message
                    print("")
                    response = yes_or_no(
                        "Grasp path visualization failed. Do you want to "
                        "compute another grasp?"
                    )

                    # Check response
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    response = yes_or_no(
                        "Grasp path visualization successfull. Do you want to "
                        "execute the computed grasp?"
                    )

                    # Check response
                    if not response:
                        continue  # Go back to start of while loop

                #################################
                # Execute grasp #################
                #################################
                print("Executing grasp...")
                result1 = self._execute_gripper_plan_srv()
                result2 = self._execute_grasp_srv()

                # Check if execution was successfull
                if (not result1.success) or (not result2.success):
                    print("")

                    # Display execution fail message
                    if not result1.success:
                        response = yes_or_no(
                            "Gripper plan execution failed. "
                            "Do you want to compute another grasp?"
                        )
                    else:
                        response = yes_or_no(
                            "Arm plan execution failed. "
                            "Do you want to compute another grasp?"
                        )

                    # Check user input
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    response = yes_or_no(
                        "Grasp path execution successfull. Do you want to close the "
                        "gripper?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)

                # Close gripper
                result = self._close_gripper_srv()
                if not result.success:
                    print("")
                    response = yes_or_no(
                        "Gripper close action failed. Do you want to compute another "
                        "grasp?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    response = yes_or_no(
                        "Gripper close action successfull. Do you want to pick the "
                        "object and place it somewhere else?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)

                #################################
                # Plan place ####################
                #################################
                result = self._plan_place_srv()
                if not result.success:
                    print("")
                    response = yes_or_no(
                        "Object place planning action failed. Do you want to compute "
                        "another grasp?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    response = yes_or_no(
                        "Object place planning action successfull. Do you want to "
                        "visualize the place plan?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)

                #################################
                # Place visualization ###########
                #################################
                print("Visualizing place path...")
                result = self._visualize_grasp_srv()
                if not result.success:
                    print("")
                    response = yes_or_no(
                        "Place path visualization failed. Do you want to compute "
                        "another grasp?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    response = yes_or_no(
                        "Grasp path visualization successfull. Do you want to execute "
                        "the place plan?"
                    )
                    if not response:
                        continue  # Go back to start of while loop

                #################################
                # Execute place #################
                #################################
                print("Executing grasp...")
                result = self._execute_grasp_srv()
                if not result.success:
                    print("")
                    response = yes_or_no(
                        "Place movement execution failed. Do you want to compute "
                        "another grasp?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    response = yes_or_no(
                        "Place movement path execution successfull. Do you want "
                        "to open the gripper?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)

                # Close gripper
                result = self._open_gripper_srv()
                if not result.success:
                    print("")
                    response = yes_or_no(
                        "Gripper open action failed. Do you want to compute "
                        "another grasp?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    response = yes_or_no(
                        "Gripper open action successfull. Do you want to "
                        "compute another grasp?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)

        except (
            rospy.exceptions.ROSInterruptException,
            rospy.service.ServiceException,
        ) as e:
            # Mostly means used pressed ctrl + c
            if type(e) == rospy.exceptions.ROSInterruptException:
                rospy.loginfo("Shutting down panda_autograsp client.")
                sys.exit(0)
            else:  # Could be ctrl + c inside a service thread or something else
                rospy.logerr("Panda_autograsp client shut down because: %s" % e)
                sys.exit(0)


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node("panda_autograsp_cli", anonymous=True)

    # Initialize cli object
    panda_autograsp_cli = PandaAutograspCLI()

    # Start cli object
    panda_autograsp_cli.start()

    # Spin till shutdown
    rospy.spin()
