#!/usr/bin/env python
"""This is the main node of the panda_autograsp autonomous grasping solution.
This node is responsible for the communication between the
:py:mod:`panda_autograsp_cli` command line interface (CLI)
and the other components of the ``panda_autograsp`` solution.
It sets up a number of services which can be called from the
:py:mod:`panda_autograsp_cli` script. It uses the
:py:class:`PandaAutograspServer` to do this. The services that are
set up by the ``panda_autograsp_server`` node are listed below:

**Services:**

    - gqcnn_grasp_planner: Computes a grasp pose out of RGB-D images
    - gqcnn_grasp_planner_bounding_box: Also computes the grasp but allows you to supply a bounding box.
    - gqcnn_grasp_planner_segmask: Also computes the grasp but allows you to supply a segmask.
"""

# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# Import ROS packages
import rospy

# Panda_autograsp modules, msgs and srvs
from panda_autograsp import PandaAutograspServer

# Pass in camera type

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ros node
    rospy.loginfo("Initializing panda_autograsp_server")
    rospy.init_node("panda_autograsp_server")

    # Get private parameters specified in the launch file
    try:
        sensor_base_frame = rospy.get_param("~camera_link")
    except KeyError:
        sensor_base_frame = "camera_link"
    try:  # Check calib type
        pose_Calib_method = rospy.get_param("~calib_type")
    except KeyError:
        pose_Calib_method = ""
    try:  # Check if wer are in a gazebo simulation
        gazebo = rospy.get_param("~gazebo")
    except KeyError:
        gazebo = False
    try:
        bounding_box_enabled = rospy.get_param("~use_bounding_box")
    except KeyError:
        bounding_box_enabled = False

    # Create GraspPlannerClient object
    grasp_planner_client = PandaAutograspServer(
        pose_calib_method=pose_Calib_method,
        gazebo=gazebo,
        bounding_box_enabled=bounding_box_enabled,
        sensor_base_frame=sensor_base_frame,
    )

    # Loop till the service is shutdown
    rospy.spin()
