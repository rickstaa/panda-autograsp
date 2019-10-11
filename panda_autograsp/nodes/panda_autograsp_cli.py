#!/usr/bin/env python
"""This node is used as a command line interface for the panda_autograsp package.
"""

## Import python packages ##
import rospy
import sys
import traceback

## Import custom python packages ##
from panda_autograsp.srv import (
	ComputeGrasp, PlanGrasp, VisualizeGrasp, ExecuteGrasp, CalibrateSensor)
from panda_autograsp.functions import (yes_or_no)

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

	## Initialize ros node ##
	rospy.init_node('panda_autograsp_cli', anonymous=True)

	try:

		## Welcome message ##
		print("----- PANDA_AUTOGRASP_PACKAGE_CLI -----")
		print("| Welcome to the panda_autograsp cli. |")
		print("---------------------------------------")
		print("| INFO: Press ctrl+D, or ctrl+c+enter |")
		print("| to close the client                 |")
		print("---------------------------------------")
		print("")

		## Initialize camera calibration service ##
		rospy.loginfo("Connecting to the panda_autograsp services...")
		rospy.logdebug("Conneting to \'calibrate_sensor\' service.")
		rospy.wait_for_service("calibrate_sensor")
		try:
			calibrate_sensor_srv = rospy.ServiceProxy(
				"calibrate_sensor", CalibrateSensor)
		except rospy.ServiceException as e:
			rospy.logerr(
				"Panda_autograsp \'calibrate_sensor\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (
				rospy.get_name(), calibrate_sensor_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		## Initialize Request grasp service ##
		rospy.logdebug("Conneting to \'compute_grasp\' service.")
		rospy.wait_for_service("compute_grasp")
		try:
			compute_grasp_srv = rospy.ServiceProxy(
				"compute_grasp", ComputeGrasp)
		except rospy.ServiceException as e:
			rospy.logerr(
				"Panda_autograsp \'compute_grasp\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (
				rospy.get_name(), compute_grasp_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		## Initialize moveit_planner server services ##
		rospy.logdebug("Conneting to \'plan_grasp\' service.")
		rospy.wait_for_service("plan_grasp")
		try:
			plan_grasp_srv = rospy.ServiceProxy(
				"plan_grasp", PlanGrasp)
		except rospy.ServiceException as e:
			rospy.logerr(
				"Panda_autograsp \'plan_grasp\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (
				rospy.get_name(), plan_grasp_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		## Initialize plan visualization service ##
		rospy.logdebug("Conneting to \'visualize_grasp\' service.")
		rospy.wait_for_service("visualize_grasp")
		try:
			visualize_grasp_srv = rospy.ServiceProxy(
				"visualize_grasp", VisualizeGrasp)
		except rospy.ServiceException as e:
			rospy.logerr(
				"Panda_autograsp \'visualize_grasp\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (
				rospy.get_name(), visualize_grasp_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		## Initialize execute plan service ##
		rospy.logdebug("Conneting to \'execute_grasp\' service.")
		rospy.wait_for_service("execute_grasp")
		try:
			execute_grasp_srv = rospy.ServiceProxy(
				"execute_grasp", ExecuteGrasp)
		except rospy.ServiceException as e:
			rospy.logerr(
				"Panda_autograsp \'execute_grasp\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (
				rospy.get_name(), execute_grasp_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		## Log success message ##
		rospy.loginfo("Connected to the panda_autograsp services.")

		## Keep alive as node is alive ##
		print("")
		raw_input("For the robot to know where it is relative to the camera we need a external calibration. Press enter to start the calibration procedure>> ")

		## Calibrate Camera frame ##
		while True:
			print("")
			response = yes_or_no(
				"Is the checkerboard/arucoboard positioned on the upper left corner of the table?")
			print("Computing calibration frame transform...")
			if not response:
					rospy.loginfo("Shutting down %s node: " % rospy.get_name())
					sys.exit(0)
			else:
				result = calibrate_sensor_srv()
				if not result.success:
					print("")
					response = yes_or_no("Calibration failed. Do you want to try again?")
					if not response:
						rospy.loginfo("Shutting down %s node: " % rospy.get_name())
						sys.exit(0)
				else:
					print("")
					response = yes_or_no("Was the frame correct?")
					if response:
						break  # Continue to grasp planning

		## Keep cli running until ros is shutdown ##
		while not rospy.is_shutdown():

			## Compute grasp ##
			print("")
			raw_input("Click enter to compute a grasp>> ")
			while True:
				print("Computing grasp...")
				result = compute_grasp_srv()
				if not result.success:
					print("")
					response = yes_or_no(
						"Grasp computation failed. Do you want to try again?")
					if not response:
						rospy.loginfo("Shutting down %s node: " % rospy.get_name())
						sys.exit(0)
				else:
					print("")
					response = yes_or_no(
						"A valid grasp was found. Do you want to plan for the computed grasp? (Y=Continue and n=Try again)>> ", add_options=False)
					if response:
						break  # Plan grasp

			## Grasp planning ##
			print("Planning grasp...")
			result = plan_grasp_srv()
			if not result.success:
				print("")
				response = yes_or_no(
					"Grasp path planning failed. Do you want to try again?")
				if not response:
					rospy.loginfo("Shutting down %s node: " % rospy.get_name())
					sys.exit(0)
				else:
					continue  # Go back to start of while loop
			else:
				print("")
				raw_input(
					"Grasp path planning successfull. Press enter to visualize the computed path>> ")

			## Grasp visualization ##
			print("Visualizing grasp path...")
			result = visualize_grasp_srv()
			if not result.success:
				print("")
				response = yes_or_no(
					"Grasp path visualization failed. Do you want to try again?")
				if not response:
					rospy.loginfo("Shutting down %s node: " % rospy.get_name())
					sys.exit(0)
				else:
					continue  # Go back to start of while loop
			else:
				print("")
				response = yes_or_no(
					"Grasp path visualization successfull. Do you want to execute the computed grasp?")
				if not response:
					continue  # Go back to start of while loop

			## Grasp execution ##
			print("Executing grasp...")
			result = execute_grasp_srv()
			if not result.success:
				print("")
				response = yes_or_no("Grasp execution failed. Do you want to try again?")
				if not response:
					rospy.loginfo("Shutting down %s node: " % rospy.get_name())
					sys.exit(0)
				else:
					continue  # Go back to start of while loop
			else:
				print("")
				raw_input(
					"Grasp path execution successfull. Press enter to compute another grasp>> ")

	except (rospy.exceptions.ROSInterruptException, rospy.service.ServiceException) as e:
		if type(e) == rospy.exceptions.ROSInterruptException: # Mostly means used pressed ctrl + c
			rospy.loginfo("Shutting down panda_autograsp client.")
			sys.exit(0)
		else: # Could be ctrl + c inside a service thread or something else
			rospy.logerr("Panda_autograsp client shut down because: %s" % e)
			sys.exit(0)