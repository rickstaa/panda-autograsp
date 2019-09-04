#!/usr/bin/env python
"""This node is used as a command line interface for the panda_autograsp package.
"""

import rospy
import sys

from panda_autograsp.srv import (ComputeGrasp, PlanGrasp, VisualizePlan, ExecutePlan, CalibrateSensor)
from panda_autograsp.functions import yes_or_no

## TODO: Put in class
#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

	## Initialize ros node ##
	rospy.init_node('panda_autograsp_server', anonymous=True)

	## Welcome message ##
	print("----- PANDA_AUTOGRASP_PACKAGE_CLI -----")
	print("| Welcome to the panda_autograsp cli. |")
	print("---------------------------------------")

	## Initialize panda_autograsp services ##
	## TODO: Catch unsuccefull grasps

	## Initilazie camera calibration sercice ##
	rospy.logdebug("Conneting to \'calibrate_sensor\' service.")
	rospy.wait_for_service("calibrate_sensor")
	try:
		calibrate_sensor_srv = rospy.ServiceProxy(
			"calibrate_sensor", CalibrateSensor)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), calibrate_sensor_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Initialize Request grasp service ##
	rospy.logdebug("Conneting to \'compute_grasp\' service.")
	rospy.wait_for_service("compute_grasp")
	try:
		compute_grasp_srv = rospy.ServiceProxy(
			"compute_grasp", ComputeGrasp)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), compute_grasp_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Initialize moveit_planner server services ##
	rospy.logdebug("Conneting to \'plan_grasp\' service.")
	rospy.wait_for_service("/plan_grasp")
	try:
		plan_grasp_srv = rospy.ServiceProxy(
			"/plan_grasp", PlanGrasp)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), plan_grasp_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Initialize plan visualization service ##
	rospy.logdebug("Conneting to \'visualize_grasp\' service.")
	rospy.wait_for_service("/visualize_grasp")
	try:
		visualize_grasp_srv = rospy.ServiceProxy(
			"/visualize_grasp", VisualizePlan)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), visualize_grasp_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Initialize execute plan service ##
	rospy.logdebug("Conneting to \'execute_grasp\' service.")
	rospy.wait_for_service("/execute_grasp")
	try:
		execute_grasp_srv = rospy.ServiceProxy(
			"/execute_grasp", ExecutePlan)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), execute_grasp_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Keep alive as node is alive ##
	while  not rospy.is_shutdown():

		## Calibrate Camera frame ##
		while True:
			raw_input("For the robot to know where it is relative to the camera we need a quick external calibration. Press enter to start the calibration procedure.> ")
			response = yes_or_no(
					"Is the checkerboard positioned on the upper left corner of the table?")
			if not response:
					rospy.logdebug("Shutting down %s node>" % rospy.get_name())
					sys.exit(0)
			else:
				result = calibrate_sensor_srv()
				if not result:
					response = yes_or_no("Calibration not successfully do you want to try again?")
					if not response:
						rospy.logdebug("Shutting down %s node>" % rospy.get_name())
						sys.exit(0)
				else:
					response = yes_or_no("Is the frame correct?")
					if not response:
						continue
					else:
						break #  Continue to grasp lanning

		## Compute grasp ##
		## TODO: Add ablity for users to accept the grasp
		raw_input("Click enter to compute a grasp> ")
		while True:
			print("Computing grasp...")
			result = compute_grasp_srv()
			if not result:
				response = yes_or_no("Grasp computation not successfully do you want to try again?> ")
				if not response:
					rospy.logdebug("Shutting down %s node>" % rospy.get_name())
					sys.exit(0)
			else:
				break
		raw_input("Grasp computed successfully. Press enter to compute the grasp path planning> ")

		## Grasp planning ##
		while True:
			print("Planning grasp...")
			result = plan_grasp_srv()
			if not result:
				response = yes_or_no("Grasp path computation not successfully do you want to try again?> ")
				if not response:
					rospy.logdebug("Shutting down %s node>" % rospy.get_name())
					sys.exit(0)
			else:
				break
		raw_input("Grasp path planning computed successfully. press enter to visualize the computed path> ")

		## Grasp visualization ##
		while True:
			print("Visualizing grasp path...")
			result = visualize_grasp_srv()
			if not result:
				response = yes_or_no("Grasp path visualization not successfully do you want to try again?> ")
				if not response:
					rospy.logdebug("Shutting down %s node>" % rospy.get_name())
					sys.exit(0)
			else:
				break
		raw_input("Grasp path visualized successfully. press enter to execute the computed path.")

		## Grasp execution ##
		while True:
			print("Executing grasp...")
			result = execute_grasp_srv()
			if not result:
				response = yes_or_no("Grasp did not execute successfully do you want to try again?> ")
				if not response:
					rospy.logdebug("Shutting down %s node>" % rospy.get_name())
					sys.exit(0)
			else:
				break
		raw_input("Grasp path executed successfully. press enter to compute another grasp>")

		## TODO: Create CLI break key
