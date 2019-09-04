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
from panda_autograsp.srv import RequestGraspPlanning

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

		## Initialize grasp_planning service ##
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
		rospy.loginfo("Initializing %s services.", rospy.get_name())
		self.request_grasp_planning_srv = rospy.Service('request_grasp_planning',RequestGraspPlanning, self.request_grasp_planning_service)
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

	def request_grasp_planning_service(self, req):

		## Call grasp computation service ##
		self.grasp = self.planning_scene_srv(
			self.color_image_rect, self.depth_image_rect, self.camera_info)

		## Test if successful ##
		if self.grasp:
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
