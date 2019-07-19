"""
Simple calibration script created with the opencv2 module
"""

## Improve python 2 backcompatibility ##
# NOTE: Python 2 is not officially supported
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

## Standard library imports ##
import numpy as np
import cv2
import glob
import os

## Third party imports ##
import cv2

#################################################
## Script settings ##############################
#################################################
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) # termination criteria
N_FRAMES = 20
CAMERA_CALIB_SAMPLES = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../data/calib/samples/")

#################################################
## Calibration script ###########################
#################################################

## prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) ##
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

## Arrays to store object points and image points from all the images. ##
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

## Get image frames and perform calibration ##
images = glob.glob(os.path.abspath(os.path.join(CAMERA_CALIB_SAMPLES, '*.jpg')))
for fname in images:

    ## Load and preprocess image frames ##
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    ## Find the chess board corners ##
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    ## If found, add object points, image points (after refining them) ##
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

## Close all image windows ##
cv2.destroyAllWindows()

## Perform calibration ##
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

## Print information ##
print("=== Calibration results ==")
print("Reprojection error RMS: %f" % ret)
print("Camera matrix: %f", % mtx)