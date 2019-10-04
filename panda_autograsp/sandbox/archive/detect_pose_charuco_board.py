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
board = aruco.CharucoBoard_create(7, 5, squareLength, markerLength, aruco_dict)
arucoParams = aruco.DetectorParameters_create()
img = board.draw(outSize=(1400,988))
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
	cameraMatrix = color_intr.K
	ir_mtx = [ir_intr.k1, ir_intr.k2, ir_intr.p1, ir_intr.p2, ir_intr.k3]
	distCoeffs = np.array(ir_mtx)

	## Visualize current image ##
	color_im, _, ir_im = sensor.frames(True)
	gray = cv2.cvtColor(color_im.data, cv2.COLOR_BGR2GRAY)

	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)  # First, detect markers
	aruco.refineDetectedMarkers(gray, board, corners, ids, rejectedImgPoints)

	if ids[0] != None: # if there is at least one marker detected
		charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, gray, board)
		im_with_charuco_board = aruco.drawDetectedCornersCharuco(color_im.data, charucoCorners, charucoIds, (0,255,0))
		# retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs)  # posture estimation from a charuco board
		# if retval == True:
			# im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, cameraMatrix, distCoeffs, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement
	else:
		im_with_charuco_left = color_im.data

	cv2.imshow("charucoboard", im_with_charuco_board)

cap.release()   # When everything done, release the capture
cv2.destroyAllWindows()