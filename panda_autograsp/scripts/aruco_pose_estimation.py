"""Aruco camera pose estimation
The following code is used to watch a video stream, detect a aruco board, and use
it to determine the posture of the camera in relation to the plane
of markers.

# NOTE: This script assumes that all markers are on the same plane, for example on the same piece of paper
# NOTE: THis script requires the camera to be calibrated. This can be done using the IAI2_kinect/kinect2_calibration package.
"""

## Main imports ##
import numpy
import cv2
import cv2.aruco as aruco
import os
import pickle
from perception import (Kinect2Sensor)
import numpy as np
import pickle
import copy

## Get required paths ##
load_dir_path = os.path.abspath(os.path.join(os.path.dirname(
	os.path.realpath(__file__)), "../cfg/_cfg/aruco_config.dict"))

## Script settings ##
POSE_ARROW_SIZE = 0.1 # [M]

## Initialize board parameters ##
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
rvec, tvec = None, None

#################################################
## Start Kinect sensor ##########################
#################################################
sensor = Kinect2Sensor()
sensor.start()
while(sensor.is_running):

	## Retreive camera calibration values ##
	camera_matrix = sensor.color_intrinsics.K
	ir_intr = sensor._device.getIrCameraParams()
	ir_mtx = [ir_intr.k1, ir_intr.k2, ir_intr.p1, ir_intr.p2, ir_intr.k3]
	dist_coeffs = np.array(ir_mtx)

	## Get image frame and convert to gray ##
	color_im, _, ir_im = sensor.frames(True)
	gray = cv2.cvtColor(color_im.data, cv2.COLOR_BGR2GRAY)

	## Create screen display image ##
	# Needed since opencv uses BGR instead of RGB
	screen_img = cv2.cvtColor(copy.copy(color_im.data), cv2.COLOR_RGB2BGR)

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
		screen_img = aruco.drawDetectedMarkers(screen_img, corners, ids)

		## Estimate pose ##
		retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, aruco_board, camera_matrix, dist_coeffs, rvec, tvec);

		## If pose estimation was successful draw pose ##
		if(retval > 0):
			aruco.drawAxis(screen_img, camera_matrix, dist_coeffs, rvec, tvec, POSE_ARROW_SIZE)

	# Display our image
	cv2.imshow('Show Marker', screen_img)

	# Exit at the end of the video on the 'q' keypress
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cv2.destroyAllWindows()