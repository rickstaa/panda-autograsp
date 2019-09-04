#!/usr/bin/env python
"""This node is used as a command line interface for the panda_autograsp package.
"""

import rospy

from panda_autograsp.srv import ComputeGrasp

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

	## Welcome message ##
	rospy.loginfo("==========Panda_autograsp============")
	rospy.loginfo("Welcome to the panda_autograsp solution.")
	rospy.loginfo(" ")

	## Initialize plan visualization service ##
	rospy.loginfo("Conneting to \'compute_grasp_srv\' service.")
	rospy.wait_for_service("compute_grasp")
	try:
		compute_grasp_srv = rospy.ServiceProxy(
			"compute_grasp", ComputeGrasp)
	except rospy.ServiceException as e:
		rospy.loginfo("Service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s connection failed." % (
			rospy.get_name(), compute_grasp_srv)
		rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

	## Compute grasp ##
	raw_input("Click enter to compute a grasp: ")

	## Call GQCNN grasp planning service ##
	rospy.loginfo("Computting grasp using the GQCNN grasp planning service...")
	result = compute_grasp_srv()
	if result:
		rospy.loginfo("Grasp pose computation successful.")
	else:
		rospy.loginfo("Grasp planning failed please try again.")