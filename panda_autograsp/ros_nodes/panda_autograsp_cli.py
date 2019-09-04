#!/usr/bin/env python
"""This node is used as a command line interface for the panda_autograsp package.
"""

import rospy
import sys

from panda_autograsp.srv import (RequestGraspPlanning, PlanToPoint, VisualizePlan, ExecutePlan)
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

	## Initialize Request grasp service ##
	rospy.logdebug("Conneting to \'request_grasp_planning\' service.")
	rospy.wait_for_service("request_grasp_planning")
	try:
		request_grasp_srv = rospy.ServiceProxy(
			"request_grasp_planning", RequestGraspPlanning)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), request_grasp_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node


	## Initialize moveit_planner server services ##
	rospy.logdebug("Conneting to \'plan_to_point\' service.")
	rospy.wait_for_service("/plan_to_point")
	try:
		plan_to_pose_srv = rospy.ServiceProxy(
			"/plan_to_point", PlanToPoint)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), plan_to_pose_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Initialize plan visualization service ##
	rospy.logdebug("Conneting to \'visualize_plan\' service.")
	rospy.wait_for_service("/visualize_plan")
	try:
		visualize_plan_srv = rospy.ServiceProxy(
			"/visualize_plan", VisualizePlan)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), visualize_plan_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Initialize execute plan service ##
	rospy.logdebug("Conneting to \'execute_plan\' service.")
	rospy.wait_for_service("/execute_plan")
	try:
		execute_plan_srv = rospy.ServiceProxy(
			"/execute_plan", ExecutePlan)
	except rospy.ServiceException as e:
		rospy.logdebug("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), execute_plan_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Keep alive as node is alive ##
	raw_input("Click enter to compute a grasp> ")
	while  not rospy.is_shutdown():

		## Compute grasp ##
		## TODO: Add ablity for users to accept the grasp
		while True:
			print("Computing grasp...")
			result = request_grasp_srv()
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
			print("Computing grasp path...")
			result = plan_to_pose_srv()
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
			result = plan_to_pose_srv()
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
			result = execute_plan_srv()
			if not result:
				response = yes_or_no("Grasp did not execute successfully do you want to try again?> ")
				if not response:
					rospy.logdebug("Shutting down %s node>" % rospy.get_name())
					sys.exit(0)
			else:
				break
		raw_input("Grasp path executed successfully. press enter to compute another grasp>")

		## TODO: Create CLI break key
