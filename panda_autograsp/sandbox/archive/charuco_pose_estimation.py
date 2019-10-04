# The following code is used to watch a video stream, detect a Charuco board, and use
# it to determine the posture of the camera in relation to the plane
# of markers.
#
# Assumes that all markers are on the same plane, for example on the same piece of paper
#
# Requires camera calibration (see the rest of the project for example calibration)

import numpy
import cv2
import cv2.aruco as aruco
import os
import pickle
from perception import (Kinect2Sensor)
import numpy as np

## Board Parameters ##
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
SQUARE_LENGTH = 33
MARKER_LENGTH = 25
CHARUCO_BOARD = aruco.CharucoBoard_create(7, 5, SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT)
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
img = CHARUCO_BOARD.draw(outSize=(1400,988))
# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
# plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
cv2.imwrite("test_gridboard.jpg", img)
# ax.axis("off")
# plt.savefig("charuco_board.pdf")

# Create vectors we'll be using for rotations and translations for postures
rvec, tvec = None, None

#################################################
## Start Kinect sensor ##########################
#################################################
sensor = Kinect2Sensor()
sensor.start()

while(sensor.is_running):

	# color_intr = sensor._device.getColorCameraParams()
	color_intr = sensor.color_intrinsics
	ir_intr = sensor._device.getIrCameraParams()
	# ir_intr = sensor.ir_intrinsics
	camera_matrix = color_intr.K
	ir_mtx = [ir_intr.k1, ir_intr.k2, ir_intr.p1, ir_intr.p2, ir_intr.k3]
	dist_coeffs = np.array(ir_mtx)

	## Visualize current image ##
	color_im, _, ir_im = sensor.frames(True)
	gray = cv2.cvtColor(color_im.data, cv2.COLOR_BGR2GRAY)

	# Detect Aruco markers
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

	# Refine detected markers
	# Eliminates markers not part of our board, adds missing markers to the board
	corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
			image = gray,
			board = CHARUCO_BOARD,
			detectedCorners = corners,
			detectedIds = ids,
			rejectedCorners = rejectedImgPoints,
			cameraMatrix = camera_matrix,
			distCoeffs = dist_coeffs)   

	# Outline all of the markers detected in our image
	QueryImg = aruco.drawDetectedMarkers(color_im.data, corners, borderColor=(0, 0, 255))

	# Only try to find CharucoBoard if we found markers
	if ids is not None and len(ids) > 10:

		# Get charuco corners and ids from detected aruco markers
		response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
				markerCorners=corners,
				markerIds=ids,
				image=gray,
				board=CHARUCO_BOARD)

		# Require more than 20 squares
		if response is not None and response > 20:
			# Estimate the posture of the charuco board, which is a construction of 3D space based on the 2D video 
			pose, rvec, tvec = aruco.estimatePoseCharucoBoard(
					charucoCorners=charuco_corners, 
					charucoIds=charuco_ids, 
					board=CHARUCO_BOARD, 
					cameraMatrix=camera_matrix, 
					distCoeffs=dist_coeffs,
					rvec=rvec,
					tvec=tvec)
			if pose:

				# Draw the camera posture calculated from the gridboard
				QueryImg = aruco.drawAxis(color_im.data, camera_matrix, dist_coeffs, rvec, tvec, 200)

	# Display our image
	cv2.imshow('QueryImage', QueryImg)

	# Exit at the end of the video on the 'q' keypress
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cv2.destroyAllWindows()