#!/usr/bin/env python
"""This node is used as a command line interface for the panda_autograsp package.
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
)
from panda_autograsp.functions import yes_or_no


#################################################
# PandaAutograspCLI class #######################
#################################################
class PandaAutograspCLI:
    """Panda_autograsp command line interface class
    """

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
        rospy.logdebug("Conneting to 'calibrate_sensor' service.")
        rospy.wait_for_service("calibrate_sensor")
        try:
            self.calibrate_sensor_srv = rospy.ServiceProxy(
                "calibrate_sensor", CalibrateSensor
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'calibrate_sensor' service initialization failed: %s"
                % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.calibrate_sensor_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize Request grasp service
        rospy.logdebug("Conneting to 'compute_grasp' service.")
        rospy.wait_for_service("compute_grasp")
        try:
            self.compute_grasp_srv = rospy.ServiceProxy("compute_grasp", ComputeGrasp)
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'compute_grasp' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.compute_grasp_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize moveit_planner server services
        rospy.logdebug("Conneting to 'plan_grasp' service.")
        rospy.wait_for_service("plan_grasp")
        try:
            self.plan_grasp_srv = rospy.ServiceProxy("plan_grasp", PlanGrasp)
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'plan_grasp' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.plan_grasp_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize plan visualization service
        rospy.logdebug("Conneting to 'visualize_grasp' service.")
        rospy.wait_for_service("visualize_grasp")
        try:
            self.visualize_grasp_srv = rospy.ServiceProxy(
                "visualize_grasp", VisualizeGrasp
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'visualize_grasp' service initialization failed: %s"
                % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.visualize_grasp_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize execute plan service
        rospy.logdebug("Conneting to 'execute_grasp' service.")
        rospy.wait_for_service("execute_grasp")
        try:
            self.execute_grasp_srv = rospy.ServiceProxy("execute_grasp", ExecuteGrasp)
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'execute_grasp' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.execute_grasp_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize set_gripper_open service
        rospy.logdebug("Conneting to 'set_gripper_open' service.")
        rospy.wait_for_service("set_gripper_open")
        try:
            self.set_gripper_open_srv = rospy.ServiceProxy(
                "set_gripper_open", SetGripperOpen
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'set_gripper_open' service initialization failed: %s"
                % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.set_gripper_open_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize set_gripper_closed service
        rospy.logdebug("Conneting to 'set_gripper_closed' service.")
        rospy.wait_for_service("set_gripper_closed")
        try:
            self.set_gripper_closed_srv = rospy.ServiceProxy(
                "set_gripper_closed", SetGripperClosed
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'set_gripper_closed' service initialization failed: %s"
                % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.set_gripper_closed_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize plan_gripper service
        rospy.logdebug("Conneting to 'plan_gripper' service.")
        rospy.wait_for_service("plan_gripper")
        try:
            self.plan_gripper_srv = rospy.ServiceProxy("plan_gripper", PlanGripper)
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'plan_gripper' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.plan_gripper_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize execute_gripper_plan service
        rospy.logdebug("Conneting to 'execute_gripper_plan' service.")
        rospy.wait_for_service("execute_gripper_plan")
        try:
            self.execute_gripper_plan_srv = rospy.ServiceProxy(
                "execute_gripper_plan", ExecuteGripperPlan
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'execute_gripper_plan' service initialization "
                "failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.execute_gripper_plan_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize close_gripper service
        rospy.logdebug("Conneting to 'close_gripper' service.")
        rospy.wait_for_service("close_gripper")
        try:
            self.close_gripper_srv = rospy.ServiceProxy("close_gripper", CloseGripper)
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'close_gripper' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.close_gripper_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize open_gripper service
        rospy.logdebug("Conneting to 'open_gripper' service.")
        rospy.wait_for_service("open_gripper")
        try:
            self.open_gripper_srv = rospy.ServiceProxy("open_gripper", CloseGripper)
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'open_gripper' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.open_gripper_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize plan place service
        rospy.logdebug("Conneting to 'plan_place' service.")
        rospy.wait_for_service("plan_place")
        try:
            self.plan_place_srv = rospy.ServiceProxy("plan_place", CloseGripper)
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'plan_place' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self.plan_place_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Log success message
        rospy.loginfo("Connected to the panda_autograsp services.")

    def start(self):
        "Start command line interface."

        # Keep alive as node is alive
        print("")
        raw_input(
            "For the robot to know where it is relative to the camera we need "
            "to perform a calibration. Press enter to start the calibration "
            "procedure>> "
        )

        try:  # Catch user and service exceptions

            # Calibrate Camera frame
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
                    result = self.calibrate_sensor_srv()
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

            # Keep cli running until ros is shutdown
            while not rospy.is_shutdown():

                # Compute grasp
                print("")
                raw_input("Click enter to compute a grasp>> ")
                while True:
                    print("Computing grasp...")
                    result = self.compute_grasp_srv()
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

                # Grasp planning
                print("Planning grasp...")
                result1 = (
                    self.set_gripper_open_srv()
                )  # Set gripper joint states to open
                result2 = (
                    self.plan_gripper_srv()
                )  # Plan for the set gripper joint states
                if (not result1.success) or (not result2.success):
                    rospy.logwarn(
                        "Gripper could not be controlled please check the gripper "
                        "move_group."
                    )
                result = self.plan_grasp_srv()
                if not result.success:
                    print("")
                    response = yes_or_no(
                        "Grasp path planning failed. Do you want to compute "
                        "another grasp?"
                    )
                    if not response:
                        rospy.loginfo("Shutting down %s node." % rospy.get_name())
                        sys.exit(0)
                    else:
                        continue  # Go back to start of while loop
                else:
                    print("")
                    raw_input(
                        "Grasp path planning successfull. Press enter to "
                        "visualize the computed path>> "
                    )

                # Grasp visualization
                print("Visualizing grasp path...")
                result = self.visualize_grasp_srv()
                if not result.success:
                    print("")
                    response = yes_or_no(
                        "Grasp path visualization failed. Do you want to "
                        "compute another grasp?"
                    )
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
                    if not response:
                        continue  # Go back to start of while loop

                # Grasp execution
                print("Executing grasp...")
                result = self.execute_gripper_plan_srv()
                if not result.success:
                    rospy.logwarn(
                        "Gripper could not be controlled please check the gripper "
                        "move_group."
                    )
                result = self.execute_grasp_srv()
                if not result.success:
                    print("")
                    response = yes_or_no(
                        "Grasp execution failed. Do you want to compute another grasp?"
                    )
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
                result = self.close_gripper_srv()
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

                # Start place planning
                result = self.plan_place_srv()
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

                # Grasp visualization
                print("Visualizing grasp path...")
                result = self.visualize_grasp_srv()
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

                # Grasp execution
                print("Executing grasp...")
                result = self.execute_grasp_srv()
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
                result = self.open_gripper_srv()
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
