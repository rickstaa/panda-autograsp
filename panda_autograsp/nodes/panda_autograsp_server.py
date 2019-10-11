#!/usr/bin/env python
"""This node calls the ROS GQCNN message service.
"""

## Make script both python2 and python3 compatible ##
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

## Import main library packages ##
import sys
import six
import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import os
import copy
import transforms3d
from pyquaternion import Quaternion
import pickle
import copy

## Import ROS python packages ##
import rospy
import sensor_msgs
from message_filters import (ApproximateTimeSynchronizer, Subscriber)
from cv_bridge import CvBridge, CvBridgeError
import tf_conversions
import tf2_ros
import dynamic_reconfigure.client

## Import third berkeley automation packages ##
from autolab_core import YamlConfig

## Import messages and services ##
import tf2_geometry_msgs
from gqcnn.srv import GQCNNGraspPlanner
from panda_autograsp.srv import (ComputeGrasp, PlanGrasp, PlanToPoint, VisualizePlan, VisualizeGrasp, ExecutePlan, ExecuteGrasp, CalibrateSensor, SetSensorPose)
import geometry_msgs.msg
from std_msgs.msg import Header

## Import custom packages ##
from panda_autograsp.functions import yes_or_no
from panda_autograsp.functions import conversions

#################################################
## Script settings ##############################
#################################################
CALIB_TRY_DURATION = 30 # [s]

###############################
## Chessboard settings ########
###############################

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

###############################
## Aruco settings #############
###############################
POSE_ARROW_SIZE = 0.2 # [M]

## Initialize board parameters ##
load_dir_path = os.path.abspath(os.path.join(os.path.dirname(
	os.path.realpath(__file__)), "../cfg/_cfg/aruco_config.dict"))
with open(load_dir_path, 'rb') as config_dict_file:
	config_dict = pickle.load(config_dict_file) # Load the aruco board settings

## Overwrite settings based on measurements ##
config_dict["MARKER_LENGTH"] =0.032       # [M]
config_dict["MARKER_SEPERATION"] = 0.009  #[M]
ARUCO_DICT = aruco.Dictionary_get(config_dict["ARUCO_DICT_TYPE"])
aruco_board = aruco.GridBoard_create(
	markersX=config_dict["MARKERS_X"],
	markersY=config_dict["MARKERS_Y"],
	markerLength=config_dict["MARKER_LENGTH"],
	markerSeparation=config_dict["MARKER_SEPARATION"],
	dictionary=ARUCO_DICT)
ARUCO_PARAMETERS = aruco.DetectorParameters_create()

## Initialize rvec and tvec vectors ##
# These will be used as an initial guess in the pose estimation.
RVEC, TVEC = None, None

###############################
## Read main config ###########
###############################
## Read panda_autograsp configuration file ##
MAIN_CFG = YamlConfig(os.path.abspath(os.path.join(os.path.dirname(
	os.path.realpath(__file__)), "../cfg/main_config.yaml")))

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
class ComputeGraspServer():
	def __init__(self):

		## Setup cv_bridge ##
		self.bridge = CvBridge()

		## Setup member variables ##
		self.rvec = RVEC
		self.tvec = TVEC

		###############################################
		## Initialize grasp computation services ######
		###############################################

		rospy.loginfo("Conneting to %s service." % "gqcnn_grasp_planner")
		rospy.wait_for_service("gqcnn_grasp_planner")
		try:
			self.gqcnn_grasp_planning_srv = rospy.ServiceProxy(
				"gqcnn_grasp_planner", GQCNNGraspPlanner)
			rospy.loginfo("Grasp_planner service found!")
		except rospy.ServiceException as e:
   			rospy.logerr("Panda_autograsp \'gqcnn_grasp_planner\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(), self.gqcnn_grasp_planning_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		###############################################
		## Initialize moveit_planner server services ##
		###############################################

		## Initialize Plan to point service ##
		rospy.loginfo("Initializing moveit_planner services.")
		rospy.loginfo("Conneting to \'moveit/plan_to_point\' service.")
		rospy.wait_for_service("moveit/plan_to_point")
		try:
			self.plan_to_pose_srv = rospy.ServiceProxy(
				"moveit/plan_to_point", PlanToPoint)
		except rospy.ServiceException as e:
   			rospy.logerr("Panda_autograsp \'moveit/plan_to_point\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(), self.plan_to_pose_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		## Initialize plan visualization service ##
		rospy.loginfo("Conneting to \'moveit/visualize_plan\' service.")
		rospy.wait_for_service("moveit/visualize_plan")
		try:
			self.visualize_plan_srv = rospy.ServiceProxy(
				"moveit/visualize_plan", VisualizePlan)
		except rospy.ServiceException as e:
   			rospy.logerr("Panda_autograsp \'moveit/visualize_plan\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(), self.visualize_plan_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		## Initialize execute plan service ##
		rospy.loginfo("Conneting to \'moveit/execute_plan\' service.")
		rospy.wait_for_service("moveit/execute_plan")
		try:
			self.execute_plan_srv = rospy.ServiceProxy(
				"moveit/execute_plan", ExecutePlan)
		except rospy.ServiceException as e:
  			rospy.logerr("Panda_autograsp \'moveit/execute_plan\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(), self.execute_plan_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		## Initialize send sensor pose service ##
		rospy.loginfo("Conneting to \'send_sensor_pose\' service.")
		rospy.wait_for_service("set_sensor_pose")
		try:
			self.set_sensor_pose_srv = rospy.ServiceProxy(
				"set_sensor_pose", SetSensorPose)
		except rospy.ServiceException as e:
  			rospy.logerr("Panda_autograsp \'set_sensor_pose\' service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(), self.set_sensor_pose_srv.resolved_name)
			rospy.logerr(shutdown_msg)
			sys.exit(0)

		###############################################
		## Create panda_autograsp_server services #####
		###############################################

		## Calibrate sensor ##
		rospy.loginfo("Initializing %s services.", rospy.get_name())
		rospy.loginfo("Initializing panda_autograsp/calibrate_sensor service.")
		self.calibrate_sensor_srv = rospy.Service('calibrate_sensor', CalibrateSensor , self.calibrate_sensor_service)

		## Compute grasp service ##
		rospy.loginfo("Initializing panda_autograsp/compute_grasp services.")
		self.compute_grasp_srv = rospy.Service('compute_grasp',ComputeGrasp , self.compute_grasp_service)

		## Plan grasp service ##
		rospy.loginfo("Initializing panda_autograsp/plan_grasp services.")
		self.plan_grasp_srv = rospy.Service('plan_grasp', PlanGrasp, self.plan_grasp_service)

		## Visualize grasp service ##
		rospy.loginfo("Initializing panda_autograsp/visualize_grasp services.")
		self.visualize_grasp_srv = rospy.Service('visualize_grasp', VisualizeGrasp, self.visualize_grasp_service)

		## execute grasp service ##
		rospy.loginfo("Initializing panda_autograsp/execute_grasp services.")
		self.execute_grasp_srv = rospy.Service('execute_grasp', ExecuteGrasp, self.execute_grasp_service)

		## Service initiation success messag ##
		rospy.loginfo(
			"\'%s\' services initialized successfully. Waiting for requests.", rospy.get_name())

		###############################################
		## Create subscribers and publishers ##########
		###############################################

		## Create msg filter subscribers ##
		rospy.loginfo("Creating camera sensor message_filter.")
		self.color_image_sub = Subscriber("image_color", sensor_msgs.msg.Image)
		self.color_image_rect_sub = Subscriber("image_color_rect", sensor_msgs.msg.Image)
		self.depth_image_rect_sub = Subscriber("image_depth_rect_32FC1", sensor_msgs.msg.Image)
		self.camera_info_hd_sub = Subscriber("hd/camera_info", sensor_msgs.msg.CameraInfo)
		self.camera_info_qhd_sub = Subscriber("qhd/camera_info", sensor_msgs.msg.CameraInfo)
		self.camera_info_sd_sub = Subscriber("sd/camera_info", sensor_msgs.msg.CameraInfo)

		## Create msg filter ##
		self.ats = ApproximateTimeSynchronizer([self.color_image_sub, self.color_image_rect_sub, self.depth_image_rect_sub, self.camera_info_hd_sub, self.camera_info_qhd_sub, self.camera_info_sd_sub], queue_size=5, slop=0.1)
		self.ats.registerCallback(self.msg_filter_callback)
		rospy.loginfo("Camera sensor message_filter created.")

		## Create dynamic reconfigure client ##
		self.dyn_client = dynamic_reconfigure.client.Client("tf2_broadcaster", timeout=30)

		## Create static publisher ##
		self.camera_frame_br = tf2_ros.StaticTransformBroadcaster()

		## Create state listener ##
		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		## Create publisher to publish pose of translated grasp ##
		self.pose_pub = rospy.Publisher("move_group/pose",
										   geometry_msgs.msg.PoseStamped,
										   queue_size=10)

		## Create pose subscriber ##
		self.pose_sub = rospy.Subscriber("gqcnn_grasp/pose", geometry_msgs.msg.PoseStamped, self.get_pose_callback)

	def get_pose_callback(self, pose_msg):
		"""Callback function of the 'gqcnn_graps/pose` subsriber. This function updates the
		self.pose_msg member variable.

		Parameters
		----------
		pose_msg : geometry_msgs.PosedStamed
			Grasp pose msgs.
		"""

		## Update pose_msg ##
		rospy.loginfo("Received grasp pose.")
		self.pose_msg = pose_msg

	def msg_filter_callback(self, color_image, color_image_rect, depth_image_rect, camera_info_hd, camera_info_qhd, camera_info_sd):

		## Call the grasp_planner_service ##
		self.color_image = color_image
		self.color_image_rect = color_image_rect
		self.depth_image_rect = depth_image_rect
		self.camera_info_hd = camera_info_hd
		self.camera_info_qhd = camera_info_qhd
		self.camera_info_sd = camera_info_sd

	def compute_grasp_service(self, req):

		## Call grasp computation service ##
		self.grasp = self.gqcnn_grasp_planning_srv(
			self.color_image_rect, self.depth_image_rect, self.camera_info_sd)

		## Print grasp ##
		position = self.grasp.grasp.pose.position
		orientation = self.grasp.grasp.pose.orientation
		pose_array = [position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w]
		rospy.logdebug("Grasp pose result in kinect2_rgb_camera_frame: x={0}, y={1}, z={2}, q1={3}, q2={4}, q3={5} and q4={6}".format(*pose_array))

		## Test if successful ##
		if self.grasp:
			return True
		else:
			return False

	def plan_grasp_service(self, req):

		## Get transform between panda_link8 and gripper center ##
		try:
			trans_grip_center_p8 = self.tf2_buffer.lookup_transform("panda_link8", "gripper_center", rospy.Time(0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			return False

		## Get pose expressed in the panda_link8 frame ##
		try:
			pose_msg = self.tf2_buffer.transform(self.pose_msg, "panda_link8", rospy.Duration(1))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			return False

		## Transform grasp pose such that it aligns with the gripper_center frame ##
		# This is needed since the move_group uses the panda_link8 frame as its
		# end-effector.

		## Transform transformation to 4x4 homogeneous matrix ##
		H_trans = conversions.transform_stamped_2_matrix(trans_grip_center_p8)
		H_trans = tf_conversions.transformations.inverse_matrix(H_trans) # Use inverse to correct for planning in the wrong EEF
		H_pose = conversions.pose_msg_stamped_2_matrix(pose_msg)

		## New Pose ##
		pose_new = np.dot(H_trans, H_pose)
		trans = tf_conversions.transformations.translation_from_matrix(pose_new)
		rot = tf_conversions.transformations.quaternion_from_matrix(pose_new)
		pose_msg.pose.position.x = trans[0]
		pose_msg.pose.position.y = trans[1]
		pose_msg.pose.position.z = trans[2]
		pose_msg.pose.orientation.x = rot[0]
		pose_msg.pose.orientation.y = rot[1]
		pose_msg.pose.orientation.z = rot[2]
		pose_msg.pose.orientation.w = rot[3]
		# pose_msg = tf2_geometry_msgs.do_transform_pose(pose_msg, trans_grip_center_p8)

		## Get pose expressed in the panda_link0 frame ##
		# Needed since the panda_link0 is the reference frame
		# of the move group.
		try:
			pose_msg = self.tf2_buffer.transform(pose_msg, "panda_link0", rospy.Duration(1))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			return False

		## Display pose in panda_link0 frame ##
		position = pose_msg.pose.position
		orientation = pose_msg.pose.orientation
		pose_array = [position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w]
		rospy.logdebug("Grasp pose result in panda_link0: x={0}, y={1}, z={2}, q1={3}, q2={4}, q3={5} and q4={6}".format(*pose_array))

		## Publish transformed pose ##
		self.pose_pub.publish(pose_msg)

		## Call grasp plan to pose service ##
		result = self.plan_to_pose_srv(pose_msg.pose)

		## Test if successful ##
		if result.success:
			return True
		else:
			return False

	def visualize_grasp_service(self, req):

		## Call grasp computation service ##
		result = self.visualize_plan_srv()

		## Test if successful ##
		if result.success:
			return True
		else:
			return False

	def execute_grasp_service(self, req):

		## Call grasp computation service ##
		result = self.execute_plan_srv()

		## Test if successful ##
		if result.success:
			return True
		else:
			return False

	def calibrate_sensor_service(self, req):

		## Retrieve camera pose ##
		retval, self.rvec, self.tvec = self.camera_world_calibration(calib_type=POSE_CALIB_METHOD)

		## Test if successful ##
		if retval:

			## Publish the camera frame ##
			self.broadcast_camera_frame(calib_type=POSE_CALIB_METHOD)

			## return result ##
			return True
		else:
			return False

	def camera_world_calibration(self, calib_type="aruco_board"):
		"""Perform camera world calibration (External camera matrix) using
		a chessboard or several aruco markers.

		Parameters
		----------
		color_image : [type]
			[description]
		camera_info : [type]
			[description]

		Returns
		-------
		[type]
			[description]
		"""

		## Switch between different calibrations ##
		if calib_type == "chessboard":
			return self.chessboard_pose_estimation() # Perform calibration using an chessboard
		else:
			return self.aruco_board_pose_estimation()  # Perform calibration using an arucoboard

	def aruco_board_pose_estimation(self):

		## Get current time ##
		start_time = rospy.get_time()

		## Try till chessboard is found or till try time is over ##
		while rospy.get_time() < start_time + CALIB_TRY_DURATION:

			## Retrieve color image and convert to opencv format ##
			color_image = self.color_image
			camera_info = self.camera_info_hd
			color_image_cv = self.bridge.imgmsg_to_cv2(color_image, desired_encoding="passthrough")

			## Get camera information ##
			camera_matrix = np.array(camera_info.K).reshape(3,3)
			dist_coeffs = camera_info.D # Default distortion parameters are 0

			## Get gray image ##
			gray = cv2.cvtColor(color_image_cv, cv2.COLOR_BGR2GRAY)

			## Create screen display image ##
			# Needed since opencv uses BGR instead of RGB
			screen_img = cv2.cvtColor(copy.copy(color_image_cv), cv2.COLOR_RGB2BGR)

			## Detect aruco markers ##
			corners, ids, rejectedImgPoints = aruco.detectMarkers(
					image = gray,
					dictionary = ARUCO_DICT,
					parameters=ARUCO_PARAMETERS,
					cameraMatrix = camera_matrix,
					distCoeff = dist_coeffs)

			## Refine detected markers ##
			# Eliminates markers not part of our board, adds missing markers to the board
			corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
					image = gray,
					board = aruco_board,
					detectedCorners = corners,
					detectedIds = ids,
					rejectedCorners = rejectedImgPoints,
					cameraMatrix = camera_matrix,
					distCoeffs = dist_coeffs)

			## If at least one marker was found try to estimate the pose
			if ids is not None and ids.size > 0:

				## Outline all of the markers detected in our image ##
				screen_img = aruco.drawDetectedMarkers(screen_img, corners, ids, borderColor=(0,255, 0))

				## Estimate pose ##
				retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, aruco_board, camera_matrix, dist_coeffs, RVEC, TVEC);

				## If pose estimation was successful draw pose ##
				if(retval > 0):
					aruco.drawAxis(screen_img, camera_matrix, dist_coeffs, rvec, tvec, POSE_ARROW_SIZE)

					## Show projection to user ##
					plt.figure("Reference frame")
					plt.imshow(screen_img)
					plt.show()
					return retval, rvec, tvec
				else:
					rospy.logwarn("Pose of arcuboard could not be found please try again.")
					return False, None, None
			else:
				rospy.logwarn("Arcuboard could not be detected make sure the arcuboard is present.")

		## Display timeout message and return ##
		rospy.logwarn("Arcuboard detector times out after %s. seconds. Please reposition the arcuboard and try again.", CALIB_TRY_DURATION)
		return False, None, None

	def chessboard_pose_estimation(self):

		## Get current time ##
		start_time = rospy.get_time()

		## Try till chessboard is found or till try time is over ##
		while rospy.get_time() < start_time + CALIB_TRY_DURATION:

			## Retrieve color image and convert to opencv format ##
			color_image = self.color_image
			camera_info = self.camera_info_hd
			color_image_cv = self.bridge.imgmsg_to_cv2(color_image, desired_encoding="passthrough")

			## Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) ##
			objp = np.zeros((N_COLMNS*N_ROWS, 3), np.float32)
			objp[:, :2] = np.mgrid[0:N_ROWS, 0:N_COLMNS].T.reshape(-1, 2) * SQUARE_SIZE # Multiply by chessboard scale factor to get results in mm
			axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3) * SQUARE_SIZE # Coordinate axis

			## Get camera information ##
			camera_matrix = np.array(camera_info.K).reshape(3,3)
			dist_coeffs = camera_info.D # Default distortion parameters are 0

			## Get gray image ##
			gray = cv2.cvtColor(color_image_cv, cv2.COLOR_BGR2GRAY)

			## Create screen display image ##
			screen_img = cv2.cvtColor(copy.copy(color_image_cv), cv2.COLOR_RGB2BGR)

			## Find the chess board corners ##
			retval, corners = cv2.findChessboardCorners(
				gray, (N_ROWS, N_COLMNS), None)

			## Find external matrix ##
			if retval:

				## Find corners ##
				corners2 = cv2.cornerSubPix(
				gray, corners, (11, 11), (-1, -1), criteria)

				## Find the rotation and translation vectors. ##
				retval, rvecs, tvecs, inliers = cv2.solvePnPRansac(
					objp, corners2, camera_matrix, dist_coeffs)

				## project 3D points to image plane ##
				imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, camera_matrix, dist_coeffs)

				## Show projection to user ##
				screen_img = draw_axis(screen_img, corners2, imgpts)
				plt.figure("Reference frame")
				plt.imshow(screen_img)
				plt.show()
				if retval:
					return retval, rvecs, tvecs
				else:
					rospy.logwarn("Pose of chessboard could not be found please try again.")
					return False, None, None
			else:
				rospy.logwarn("Chessboard could not be detected make sure the chessboard is present.")

		## Display timeout message and return ##
		rospy.logwarn("Chessboard detector times out after %s. seconds. Please reposition the chessboard and try again.", CALIB_TRY_DURATION)
		return False, None, None

	def broadcast_camera_frame(self, calib_type="aruco_board"):
		"""Send the sensor pose we acquired from the calibration to the tf2_broadcaster so that
		it can broadcast the sensor camera frame.
		"""

		## Check calibration method ##
		if calib_type == "chessboard":

			## Get rotation matrix ##
			R = np.zeros(shape=(3,3))
			J = np.zeros(shape=(3,3))
			cv2.Rodrigues(self.rvec, R, J)

			## Compute inverse rotation and translation matrix ##
			R = R.T
			tvec = np.dot(-R, self.tvec)

			## Create homogenious matrix and the flip x and y axis ##
			H = np.empty((4, 4))
			H[:3, :3] = R
			H[:3, 3] = tvec.reshape(1,3)
			H[3, :] = [0, 0, 0, 1]
			H = np.dot(np.array([[1,0,0,0],[0,-1,0,0], [0,0,-1,0], [0,0,0,1]]), H)
			R = H[0:3,0:3]
			quat = Quaternion(matrix=R)

			## Print Calibration information ##
			cal_pose = {
				"x": float(H[0,3]/1000.0),
				"y": float(H[1,3]/1000.0),
				"z": float(H[2,3]/1000.0),
				"q1": float(quat[1]),
				"q2": float(quat[2]),
				"q3": float(quat[3]),
				"q4": float(quat[0])
			}
			rospy.loginfo("Calibration result: x={x}, y={y}, z={z}, q1={q1}, q2={q2}, q3={q3} and q4={q4}".format(**cal_pose))

			## Create geometry_msg ##
			sensor_frame_tf_msg = geometry_msgs.msg.TransformStamped()
			sensor_frame_tf_msg.header.stamp = rospy.Time.now()
			sensor_frame_tf_msg.header.frame_id = "calib_frame"
			sensor_frame_tf_msg.child_frame_id = "kinect2_rgb_optical_frame"
			sensor_frame_tf_msg.transform.translation.x = float(H[0,3]/1000.0)
			sensor_frame_tf_msg.transform.translation.y = float(H[1,3]/1000.0)
			sensor_frame_tf_msg.transform.translation.z = float(H[2,3]/1000.0)
			sensor_frame_tf_msg.transform.rotation.x = float(quat[1])
			sensor_frame_tf_msg.transform.rotation.y = float(quat[2])
			sensor_frame_tf_msg.transform.rotation.z = float(quat[3])
			sensor_frame_tf_msg.transform.rotation.w = float(quat[0])

			## Communicate sensor_frame_pose to the tf2_broadcaster node ##
			self.set_sensor_pose_srv(sensor_frame_tf_msg)
		else:

			## Get rotation matrix ##
			R = np.zeros(shape=(3,3))
			J = np.zeros(shape=(3,3))
			cv2.Rodrigues(self.rvec, R, J)

			## Compute inverse rotation and translation matrix ##
			R = R.T
			tvec = np.dot(-R, self.tvec)

			## Create homogenious matrix and the flip x and y axis ##
			H = np.empty((4, 4))
			H[:3, :3] = R
			H[:3, 3] = tvec.reshape(1,3)
			H[3, :] = [0, 0, 0, 1]
			R = H[0:3,0:3]
			quat = Quaternion(matrix=R)

			## Print Calibration information ##
			cal_pose = {
				"x": float(H[0,3]),
				"y": float(H[1,3]),
				"z": float(H[2,3]),
				"q1": float(quat[1]),
				"q2": float(quat[2]),
				"q3": float(quat[3]),
				"q4": float(quat[0])
			}
			rospy.logdebug("Calibration result: x={x}, y={y}, z={z}, q1={q1}, q2={q2}, q3={q3} and q4={q4}".format(**cal_pose))

			## Create geometry_msg ##
			sensor_frame_tf_msg = geometry_msgs.msg.TransformStamped()
			sensor_frame_tf_msg.header.stamp = rospy.Time.now()
			sensor_frame_tf_msg.header.frame_id = "calib_frame"
			sensor_frame_tf_msg.child_frame_id = "kinect2_rgb_optical_frame"
			sensor_frame_tf_msg.transform.translation.x = float(H[0,3])
			sensor_frame_tf_msg.transform.translation.y = float(H[1,3])
			sensor_frame_tf_msg.transform.translation.z = float(H[2,3])
			sensor_frame_tf_msg.transform.rotation.x = float(quat[1])
			sensor_frame_tf_msg.transform.rotation.y = float(quat[2])
			sensor_frame_tf_msg.transform.rotation.z = float(quat[3])
			sensor_frame_tf_msg.transform.rotation.w = float(quat[0])

			## Communicate sensor_frame_pose to the tf2_broadcaster node ##
			self.set_sensor_pose_srv(sensor_frame_tf_msg)

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

	## Initialize ros node ##
	rospy.loginfo("Initializing panda_autograsp_server")
	rospy.init_node('panda_autograsp_server')

	## Argument parser ##
	try:
		POSE_CALIB_METHOD = rospy.get_param("~calib_type")
	except KeyError:
		try:
			POSE_CALIB_METHOD = MAIN_CFG["calibration"]["pose_estimation_calib_board"]
			CALIB_CONFIG_ERROR = False
		except KeyError:
			POSE_CALIB_METHOD = "aruco_board"

	## Create GraspPlannerClient object ##
	grasp_planner_client = ComputeGraspServer()

	## Loop till the service is shutdown ##
	rospy.spin()