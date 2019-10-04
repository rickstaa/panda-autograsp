import sys
sys.path.append('/usr/local/python/3.5')

# Standard imports
import os
import cv2
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from perception import (Kinect2Sensor)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
squareLength = 40
markerLength = 30
charuco_board = aruco.CharucoBoard_create(7, 5, squareLength, markerLength, aruco_dict)
aruco_parameters = aruco.DetectorParameters_create()
img = charuco_board.draw(outSize=(1400,988))
# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
# plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
cv2.imwrite("test_gridboard.jpg", img)
# ax.axis("off")
# plt.savefig("charuco_board.pdf")

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

	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters)

	# if enough markers were detected
	# then process the board
	if( ids is not None ):
		#gray   = aruco.drawDetectedMarkers(gray, corners)
		ret, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board )
	
		# if there are enough corners to get a reasonable result
		if( ret > 5 ):
			aruco.drawDetectedCornersCharuco(color_im.data, ch_corners,ch_ids,(0,0,255))

			retval, rvec, tvec = aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, charuco_board, camera_matrix, dist_coeffs, np.zeros((3, 1, 1), dtype = "uint8"),np.zeros((3, 1, 1), dtype = "uint8"))

			# if a pose could be estimated
			if( retval ) :
				frame = aruco.drawAxis(color_im.data, camera_matrix, dist_coeffs, rvec, tvec,0.032)

	# imshow and waitKey are required for the window
	# to open on a mac.
	cv2.imshow('frame', frame)

	if( cv2.waitKey(1) & 0xFF == ord('q') ):
		break

cv2.destroyAllWindows()