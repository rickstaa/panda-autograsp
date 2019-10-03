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
import tf
import dynamic_reconfigure.client

## Import third party automation packages ##
from autolab_core import YamlConfig

## Import messages and services ##
from gqcnn.srv import GQCNNGraspPlanner
from panda_autograsp.srv import (ComputeGrasp, PlanGrasp, PlanToPoint, VisualizePlan, VisualizeGrasp, ExecutePlan, ExecuteGrasp, CalibrateSensor, SetSensorPose)
import geometry_msgs.msg
from std_msgs.msg import Header

## Import custom packages ##
from panda_autograsp.functions import yes_or_no

#################################################
## Script settings ##############################
#################################################

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
main_cfg = YamlConfig(os.path.abspath(os.path.join(os.path.dirname(
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
			rospy.logerr("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.gqcnn_grasp_planning_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node
			return

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
			rospy.logerr("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.plan_to_pose_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Initialize plan visualization service ##
		rospy.loginfo("Conneting to \'moveit/visualize_plan\' service.")
		rospy.wait_for_service("moveit/visualize_plan")
		try:
			self.visualize_plan_srv = rospy.ServiceProxy(
				"moveit/visualize_plan", VisualizePlan)
		except rospy.ServiceException as e:
			rospy.logerr("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.visualize_plan_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Initialize execute plan service ##
		rospy.loginfo("Conneting to \'moveit/execute_plan\' service.")
		rospy.wait_for_service("moveit/execute_plan")
		try:
			self.execute_plan_srv = rospy.ServiceProxy(
				"moveit/execute_plan", ExecutePlan)
		except rospy.ServiceException as e:
			rospy.logerr("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.execute_plan_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

		## Initialize send sensor pose service ##
		rospy.loginfo("Conneting to \'send_sensor_pose\' service.")
		rospy.wait_for_service("set_sensor_pose")
		try:
			self.set_sensor_pose_srv = rospy.ServiceProxy(
				"set_sensor_pose", SetSensorPose)
		except rospy.ServiceException as e:
			rospy.logerr("Service initialization failed: %s" % e)
			shutdown_msg = "Shutting down %s node because %s connection failed." % (
				rospy.get_name(), self.set_sensor_pose_srv)
			rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

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
		self.execute_grasp_srv = rospy.Service('execute_grasp', ExecutePlan, self.execute_grasp_service)

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
		self.tf2_listener = tf.TransformListener()

		## Create pose subscriber ##
		rospy.Subscriber("gqcnn_grasp/pose", geometry_msgs.msg.PoseStamped, self.get_pose_callback)

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

		## Test if successful ##
		if self.grasp:
			return True
		else:
			return False

	def plan_grasp_service(self, req):

		## Translate pose in camera frame to robot frame ##
		# try:
		# 	(trans, rot) = self.tf2_listener.lookupTransform('panda_hand', 'kinect2_rgb_optical_frame', rospy.Time(0))
		# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 	rospy.logerr("Pose could not be transformed from the camera frame to the robot frame.")
		# 	return False

		## Call grasp computation service ##
		# FIXME: Unsure if I need to add a kinect2_ir_optical_frame or that I can use the kinect2_rgb_optical_frame like I did below
		self.pose_msg.header.frame_id = "kinect2_rgb_optical_frame"
		panda_hand_grasp_pose = self.tf2_listener.transformPose('panda_hand', self.pose_msg)
		result = self.plan_to_pose_srv(panda_hand_grasp_pose.pose)

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

	def calibrate_sensor_service(self, req):

		## Retrieve camera pose ##
		retval, self.rvec, self.tvec = self.camera_world_calibration(self.color_image, self.camera_info_hd, calib_type=POSE_CALIB_METHOD)

		## Test if successful ##
		if retval:

			## Publish the camera frame ##
			self.broadcast_camera_frame(calib_type=POSE_CALIB_METHOD)

			## return result ##
			return True
		else:
			return False

	def camera_world_calibration(self, color_image, camera_info, calib_type="aruco_board"):
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
		if calib_type == "chess_board":
			return self.chessboard_pose_estimation(color_image, camera_info) # Perform calibration using an chessboard
		else:
			return self.aruco_board_pose_estimation(color_image, camera_info)  # Perform calibration using an arucoboard

	def aruco_board_pose_estimation(self, color_image, camera_info):

		## Convert color image to opencv format ##
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
		# TODO: Check if I need to add camera matrix
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

		# Refine detected markers
		# TODO: Check if I need to add camera matrix
		# Eliminates markers not part of our board, adds missing markers to the board
		corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
				image = gray,
				board = aruco_board,
				detectedCorners = corners,
				detectedIds = ids,
				rejectedCorners = rejectedImgPoints)

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
				return False, None, None
		else:
			return None, None, None # Aruco calibration failed


	## TODO: take the mean over 5 frames ##
	def chessboard_pose_estimation(self, color_image, camera_info):

		## Convert color image to opencv format ##
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
		if retval == True:

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
				return False, None, None
		else:
			return None, None, None # Chessboard calibration failed

	def broadcast_camera_frame(self, calib_type="aruco_board"):
		"""Send the sensor pose we acquired from the calibration to the tf2_broadcaster so that
		it can broadcast the sensor camera frame.
		"""

		## Check calibration method ##
		if calib_type == "chess_board":

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
			rospy.loginfo("Calibration result: x={x}, y={y}, z={z}, q1={q1}, q2={q2}, q3={q3} and q4={q4}".format(**cal_pose))

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

	## DEBUG: WAIT FOR PTVSD DEBUGGER ##
	import ptvsd
	ptvsd.wait_for_attach()
	## ------------------------------ ##

	## Argument parser ##
	try:
		POSE_CALIB_METHOD = rospy.get_param("~calib_board_type")
	except KeyError:
		try:
			POSE_CALIB_METHOD = main_cfg["calibration"]["pose_estimation_calib_board"]
			CALIB_CONFIG_ERROR = False
		except KeyError:
			POSE_CALIB_METHOD = "aruco_board"

	## Create GraspPlannerClient object ##
	grasp_planner_client = ComputeGraspServer()

	## Loop till the service is shutdown ##
	rospy.spin()