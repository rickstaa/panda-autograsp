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
import copy
import os

## Import third party packages ##
import cv2
import numpy as np
import matplotlib.pyplot as plt

## Import ROS python packages ##
import rospy
import sensor_msgs
from message_filters import (ApproximateTimeSynchronizer, Subscriber)

## Import ROS services and messages ##
from panda_autograsp.srv import (PlanToPoint, VisualizePlan, ExecutePlan)
from cv_bridge import CvBridge, CvBridgeError

## Import BerkeleyAutomation package ##
from gqcnn.srv import (
	GQCNNGraspPlanner, GQCNNGraspPlannerBoundingBox, GQCNNGraspPlannerSegmask)

#################################################
## Script settings ##############################
#################################################

## Message filter settings ##
MSG_FILTER_QUEUE_SIZE = 5  # Max queue size
MSG_FILTER_SLOP = 0.1  # Max sync delay (in seconds)

## Script settings ##
vis = True  # Visualize results
factory = False  # Use libfreenect2 camera parameters

## Big board ##
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
			30, 0.001)  # termination criteria
N_FRAMES = 10
# Row size -1 (see https://stackoverflow.com/questions/17993522/opencv-findchessboardcorners-function-is-failing-in-a-apparently-simple-scenar)
N_ROWS = 7
# Column size -1 (see https://stackoverflow.com/questions/17993522/opencv-findchessboardcorners-function-is-failing-in-a-apparently-simple-scenar)
N_COLMNS = 10
SQUARE_SIZE = 34  # The square size in mm

## Small Board ##
N_FRAMES = 10
# Row size -1 (see https://stackoverflow.com/questions/17993522/opencv-findchessboardcorners-function-is-failing-in-a-apparently-simple-scenar)
N_ROWS = 5
# Column size -1 (see https://stackoverflow.com/questions/17993522/opencv-findchessboardcorners-function-is-failing-in-a-apparently-simple-scenar)
N_COLMNS = 7
SQUARE_SIZE = 30  # The square size in mm

## Text default settings ##
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.90
fontColor = (0, 0, 0)
lineType = 1
rectangle_bgr = (255, 255, 255)

## Load calib file path ##
LOAD_CALIB = os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "..", "data", "calib","calib_results.npz"))

#################################################
## Functions ####################################
#################################################


def draw_axis(img, corners, imgpts):
	"""Takes the corners in the chessboard (obtained using cv2.findChessboardCorners())
	and axis points to draw a 3D axis.

	Parameters
	----------
	img : numpy.ndarray
		Image
	corners : corners2
		Corners
	imgpts : corners2
		Axis points

	Returns
	-------
	[type]
		[description]
	"""
	corner = tuple(corners[0].ravel())
	img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
	img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
	img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
	return img

#################################################
## GraspPlannerClient Class #####################
#################################################


class GraspPlannerClient():

	def __init__(self, grasp_detection_srv, color_topic, color_rect_topic, depth_rect_topic, camera_info_topic, queue_size=5, slop=0.1):

		## Setup member variables ##
		self.firstcallback = True

		## Setup cv_bridge ##
		self.bridge = CvBridge()

		## Initialize GQCNN grasp_planning service ##
		rospy.loginfo("Conneting to %s service." % grasp_detection_srv)
		rospy.wait_for_service(grasp_detection_srv)
		try:
			self.grasp_compuation_srv = rospy.ServiceProxy(
				grasp_detection_srv, GQCNNGraspPlanner)
		except rospy.ServiceException as e:
			rospy.loginfo("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.grasp_compuation_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node
			return

		## Initialize moveit_planner server services ##
		# Initialize plan_to_point_service
		rospy.loginfo("Conneting to \'plan_to_point\' service.")
		rospy.wait_for_service("/plan_to_point")
		try:
			self.plan_to_pose_srv = rospy.ServiceProxy(
				"/plan_to_point", PlanToPoint)
		except rospy.ServiceException as e:
			rospy.loginfo("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.plan_to_pose_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Initialize plan visualization service ##
		rospy.loginfo("Conneting to \'visualize_plan\' service.")
		rospy.wait_for_service("/visualize_plan")
		try:
			self.visualize_plan_srv = rospy.ServiceProxy(
				"/visualize_plan", VisualizePlan)
		except rospy.ServiceException as e:
			rospy.loginfo("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.visualize_plan_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Initialize execute plan service ##
		rospy.loginfo("Conneting to \'execute_plan\' service.")
		rospy.wait_for_service("/execute_plan")
		try:
			self.execute_plan_srv = rospy.ServiceProxy(
				"/execute_plan", ExecutePlan)
		except rospy.ServiceException as e:
			rospy.loginfo("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.execute_plan_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Create msg filter subscribers ##
		rospy.loginfo("Creating camera sensor message_filter.")
		color_image_sub = Subscriber(color_topic, sensor_msgs.msg.Image)
		color_rect_image_sub = Subscriber(color_rect_topic, sensor_msgs.msg.Image)
		depth_rect_image_sub = Subscriber(depth_rect_topic, sensor_msgs.msg.Image)
		camera_info_sub = Subscriber(
			camera_info_topic, sensor_msgs.msg.CameraInfo)

		## Create msg filter ##
		ats = ApproximateTimeSynchronizer(
			[color_image_sub, color_rect_image_sub, depth_rect_image_sub, camera_info_sub], queue_size, slop)
		ats.registerCallback(self.msg_filter_callback)
		rospy.loginfo("Camera sensor message_filter created.")

	def msg_filter_callback(self, color_image, color_rect_image, depth_rect_image, camera_info):

		# Perform the calibration while first entered
		if self.firstcallback:
			## Perform external world_camera calibration
			rospy.loginfo(
			"For the robot to know where it is relative to the camera we need a quick external calibration.")

			while True:
				prompt_result = raw_input(
					"Is the checkerboard positioned on the upper left corner of the table [Y/N]? ")
				if (prompt_result.lower() in ['y', 'yes']) or (prompt_result == ""):
					rvecs, tvecs, inliers = self.camera_world_calibration(color_image, camera_info)
					if not rvecs: # When fails rvecs = None
						break # Break if successful
				elif prompt_result.lower() in ['n', 'no']:
					rospy.loginfo(
						"Please place the chessboard in the upper left corner of the robot table.")
				else:
					print(prompt_result +
						  " is not a valid response please answer with Y or N to continue.")

			## Ask users if they want to compute a grasp ##
			raw_input("Click enter to compute a grasp: ")

			## Call GQCNN grasp planning service ##
			rospy.loginfo("Computting grasp using the GQCNN grasp planning service...")
			grasp = self.grasp_compuation_srv(color_rect_image, depth_rect_image, camera_info)
			if grasp:
				rospy.loginfo("Grasp pose computation successful.")
				raw_input(
					"Click enter to perform the path planning: ")
			else:
				rospy.loginfo("Grasp planning failed please try again.")

			## Send grasp to moveit planning service ##
			rospy.loginfo("Computing grasp planning using moveit...")
			result = self.plan_to_pose_srv(grasp.grasp.pose)
			if result:
				rospy.loginfo("Grasp planning successful.")
				raw_input(
					"Click enter to perform a grasp visualization: ")
			else:
				rospy.loginfo("Grasp planning failed please try again.")

			## Call visualize grasp service ##
			result = self.visualize_plan_srv()
			if result:
				rospy.loginfo("Grasp visualization succesfull.")
			else:
				rospy.loginfo("Grasp planning failed please try again.")

			## Call grasp execution service ##
			prompt_result = raw_input(
				"Do you want to execute the planned trajectory [Y/n]? ")
			if (prompt_result.lower() in ['y', 'yes']) or (prompt_result == ""):
				self.execute_plan_srv()
			elif prompt_result.lower() in ['n', 'no']:
				print("shutdown")
				sys.exit(0)
			else:
				print(prompt_result +
					  " is not a valid response please answer with Y or N to continue.")

		## Publish the camera and calibration board frames
		

	def camera_world_calibration(self, color_image, camera_info):

		## Convert color image to opencv format ##
		color_image_cv = self.bridge.imgmsg_to_cv2(color_image, desired_encoding="passthrough")

		## Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) ##
		objp = np.zeros((N_COLMNS*N_ROWS, 3), np.float32)
		objp[:, :2] = np.mgrid[0:N_ROWS, 0:N_COLMNS].T.reshape(-1, 2) * SQUARE_SIZE # Multiply by chessboard scale factor to get results in mm
		axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3) * SQUARE_SIZE # Coordinate axis

		## unpack camera information ##
		# Get calibration parameters if file exists
		if os.path.exists(LOAD_CALIB):
			a = np.load(LOAD_CALIB)
			mtx = a['mtx']
			dist = a['dst']
		else:
			mtx = np.array(camera_info.K).reshape(3,3)
			dist = camera_info.D # Default distortion parameters are 0

		## Get color image ##
		gray = cv2.cvtColor(color_image_cv, cv2.COLOR_BGR2GRAY)

		## Create screen display image ##
		screen_img = cv2.cvtColor(copy.copy(color_image_cv), cv2.COLOR_RGB2BGR)

		## Find the chess board corners ##
		ret, corners = cv2.findChessboardCorners(
			gray, (N_ROWS, N_COLMNS), None)

		## Find external matrix ##
		if ret == True:

			## Find corners ##
			corners2 = cv2.cornerSubPix(
			gray, corners, (11, 11), (-1, -1), criteria)

			## Find the rotation and translation vectors. ##
			ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(
				objp, corners2, mtx, dist)

			## project 3D points to image plane ##
			imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

			## Show projection to user ##
			screen_img = draw_axis(screen_img, corners2, imgpts)
			plt.figure("Reference frame")
			plt.imshow(screen_img)
			plt.show()
			if ret:
				return rvecs, tvecs, inliers
			else:
				return None, None, None
		else:
			return None # Chessboard calibration failed

# TODO: Cleanup ugly nonetype return statements

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

	## Welcome message ##
	rospy.loginfo("==========Panda_autograsp============")
	rospy.loginfo("Welcome to the panda_autograsp solution.")
	rospy.loginfo(" ")

	## Initialize ros node ##
	rospy.loginfo("Initializing grasp_planner_client node")
	rospy.init_node('grasp_planner_client', anonymous=True)

	## Argument parser ##
	try:
		img_quality = rospy.get_param("~grasp_img_quality")
	except KeyError:
		img_quality = 'qhd'
	try:
		grasp_detection_srv = rospy.get_param("~grasp_detection_srv")
	except KeyError:
		grasp_detection_srv = '/grasp_planner'

	## Create topics ##
	kinect_color_topic = "/kinect2/hd/image_color"
	kinect_color_rect_topic = "/kinect2/%s/image_color_rect" % img_quality
	kinect_depth_rect_topic = "/kinect2/%s/image_depth_rect_32FC1" % img_quality
	kinect_camera_info_topic = "/kinect2/%s/camera_info" % img_quality

	## Create GraspPlannerClient object ##
	grasp_planner_client = GraspPlannerClient(grasp_detection_srv, kinect_color_topic, kinect_color_rect_topic,
										   kinect_depth_rect_topic, kinect_camera_info_topic, MSG_FILTER_QUEUE_SIZE, MSG_FILTER_SLOP)

	## Loop till the service is shutdown. ##
	rospy.spin()
