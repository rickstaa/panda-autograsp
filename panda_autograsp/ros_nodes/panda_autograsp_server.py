#!/usr/bin/env python
"""This node calls the ROS GQCNN message service.
"""

## Make script both python2 and python3 compatible ##
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

## Import standard library packages ##
import sys
import six

## Import ROS python packages ##
import rospy
import sensor_msgs
from message_filters import (ApproximateTimeSynchronizer, Subscriber)

## Import messages and services ##
from gqcnn.srv import GQCNNGraspPlanner
from panda_autograsp.srv import (ComputeGrasp, PlanGrasp, PlanToPoint, VisualizePlan, VisualizeGrasp, ExecutePlan, ExecuteGrasp)

## Import custom packages ##
from panda_autograsp.functions import yes_or_no

#################################################
## Script settings ##############################
#################################################

## Message filter settings ##
MSG_FILTER_QUEUE_SIZE = 5  # Max queue size
MSG_FILTER_SLOP = 0.1  # Max sync delay (in seconds)

#################################################
## GraspPlannerClient Class #####################
#################################################
class ComputeGraspServer():

	def __init__(self, grasp_detection_srv, color_topic, color_rect_topic, depth_topic, camera_info_topic, queue_size=5, slop=0.1):

		## Initialize ros node ##
		rospy.loginfo("Initializing panda_autograsp_server")
		rospy.init_node('panda_autograsp_server')

		## Initialize grasp_computation service ##
		rospy.loginfo("Conneting to %s service." % grasp_detection_srv)
		rospy.wait_for_service(grasp_detection_srv)
		try:
			self.planning_scene_srv = rospy.ServiceProxy(
				grasp_detection_srv, GQCNNGraspPlanner)
		except rospy.ServiceException as e:
			rospy.loginfo("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.planning_scene_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node
			return

		## Initialize moveit_planner server services ##

		## Initialize Plan to point service ##
		rospy.logdebug("Conneting to \'plan_to_point\' service.")
		rospy.wait_for_service("/plan_to_point")
		try:
			self.plan_to_pose_srv = rospy.ServiceProxy(
				"/plan_to_point", PlanToPoint)
		except rospy.ServiceException as e:
			rospy.logdebug("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.plan_to_pose_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Initialize plan visualization service ##
		rospy.logdebug("Conneting to \'visualize_plan\' service.")
		rospy.wait_for_service("/visualize_plan")
		try:
			self.visualize_plan_srv = rospy.ServiceProxy(
				"/visualize_plan", VisualizePlan)
		except rospy.ServiceException as e:
			rospy.logdebug("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.visualize_plan_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Initialize execute plan service ##
		rospy.logdebug("Conneting to \'execute_plan\' service.")
		rospy.wait_for_service("/execute_plan")
		try:
			self.execute_plan_srv = rospy.ServiceProxy(
				"/execute_plan", ExecutePlan)
		except rospy.ServiceException as e:
			rospy.logdebug("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.execute_plan_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Create msg filter subscribers ##
		rospy.loginfo("Creating camera sensor message_filter.")
		color_image_sub = Subscriber(color_topic, sensor_msgs.msg.Image)
		color_image_rect_sub = Subscriber(color_rect_topic, sensor_msgs.msg.Image)
		depth_image_sub = Subscriber(depth_topic, sensor_msgs.msg.Image)
		camera_info_sub = Subscriber(
			camera_info_topic, sensor_msgs.msg.CameraInfo)

		## Create msg filter ##
		ats = ApproximateTimeSynchronizer(
			[color_image_sub, color_image_rect_sub, depth_image_sub, camera_info_sub], queue_size, slop)
		ats.registerCallback(self.msg_filter_callback)
		rospy.loginfo("Camera sensor message_filter created.")

		## Create panda_autograsp_server services ##

		## Compute grasp service ##
		rospy.loginfo("Initializing %s services.", rospy.get_name())
		self.compute_grasp_srv = rospy.Service('compute_grasp',ComputeGrasp , self.compute_grasp_service)

		## Plan grasp service ##
		rospy.loginfo("Initializing %s services.", rospy.get_name())
		self.plan_grasp_srv = rospy.Service('plan_grasp', PlanGrasp, self.plan_grasp_service)

		## Visualize grasp service ##
		rospy.loginfo("Initializing %s services.", rospy.get_name())
		self.visualize_grasp_srv = rospy.Service('visualize_grasp', VisualizeGrasp, self.visualize_grasp_service)

		## execute grasp service ##
		rospy.loginfo("Initializing %s services.", rospy.get_name())
		self.execute_grasp_srv = rospy.Service('execute_grasp', ExecutePlan, self.execute_grasp_service)

		## Service initiation succes messag ##
		rospy.loginfo(
			"\'%s\' services initialized successfully. Waiting for requests.", rospy.get_name())

		## Loop till the service is shutdown. ##
		rospy.spin()

	## TODO: Docstring
	def msg_filter_callback(self, color_image, color_image_rect, depth_image_rect, camera_info):

		## Call the grasp_planner_service ##
		self.color_image = color_image
		self.color_image_rect = color_image_rect
		self.depth_image_rect = depth_image_rect
		self.camera_info = camera_info

	def compute_grasp_service(self, req):

		## Call grasp computation service ##
		self.grasp = self.planning_scene_srv(
			self.color_image_rect, self.depth_image_rect, self.camera_info)

		## Test if successful ##
		if self.grasp:
			return True
		else:
			return False

	def plan_grasp_service(self, req):

		## Call grasp computation service ##
		result = self.plan_to_pose_srv(self.grasp.grasp.pose)

		## Test if successful ##
		if result:
			return True
		else:
			return False

	def visualize_grasp_service(self, req):

		## Call grasp computation service ##
		result = self.visualize_plan_srv()

		## Test if successful ##
		if result:
			return True
		else:
			return False

	def execute_grasp_service(self, req):

		## Call grasp computation service ##
		result = self.execute_plan_srv()

		## Test if successful ##
		if result:
			return True
		else:
			return False

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

	## Argument parser ##
	try:
		img_quality = rospy.get_param("~grasp_img_quality")
	except KeyError:
		img_quality = 'sd'
	try:
		grasp_detection_srv = rospy.get_param("~grasp_detection_srv")
	except KeyError:
		grasp_detection_srv = 'grasp_planner'

	## Create topics ##
	kinect_color_image_topic ="/kinect2/%s/image_color" % "hd"
	kinect_color_image_rect_topic = "/kinect2/%s/image_color_rect" % img_quality
	kinect_depth_image_rect_topic = "/kinect2/%s/image_depth_rect_32FC1" % img_quality
	kinect_camera_info_topic = "/kinect2/%s/camera_info" % img_quality

	## Create GraspPlannerClient object ##
	grasp_planner_client = ComputeGraspServer(grasp_detection_srv, kinect_color_image_topic, kinect_color_image_rect_topic,
										   kinect_depth_image_rect_topic, kinect_camera_info_topic, MSG_FILTER_QUEUE_SIZE, MSG_FILTER_SLOP)
