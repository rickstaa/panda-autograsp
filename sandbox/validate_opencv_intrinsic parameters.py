"""
Simple calibration script created with the opencv2 module
"""

## Improve python 2 backcompatibility ##
# NOTE: Python 2 is not officially supported
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
try:
    # Python 2
    range = xrange
except NameError:
    # Python 3
    pass

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
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30, 0.001)  # termination criteria
N_FRAMES = 20
N_ROWS = 7
N_COLMNS = 6
CAMERA_CALIB_SAMPLES = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../data/calib/samples/")

#################################################
## Functions ####################################
#################################################


def draw(img, corners, imgpts):
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
## Get Intrinsic matrix #########################
#################################################


## prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) ##
objp = np.zeros((N_COLMNS*N_ROWS, 3), np.float32)
objp[:, :2] = np.mgrid[0:N_ROWS, 0:N_COLMNS].T.reshape(-1, 2)

## Arrays to store object points and image points from all the images. ##
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

## Get image frames and perform calibration ##
images = glob.glob(os.path.abspath(
    os.path.join(CAMERA_CALIB_SAMPLES, '*.jpg')))
for fname in images:

    ## Load and preprocess image frames ##
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ## Find the chess board corners ##
    ret, corners = cv2.findChessboardCorners(gray, (N_ROWS, N_COLMNS), None)

    ## If found, add object points, image points (after refining them) ##
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (N_ROWS, N_COLMNS), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

## Close all image windows ##
cv2.destroyAllWindows()

## Perform calibration ##
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

## Print information ##
print("=== Calibration results ==")
print("Reprojection error RMS: %f" % ret)
print("Distortion parameters:")
print(dist)
print("Camera matrix:")
print(mtx)

## Get new optimal camera metrix ##
img = cv2.imread(os.path.join(CAMERA_CALIB_SAMPLES, 'left12.jpg'))
h,  w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

## undistort ##
mapx, mapy = cv2.initUndistortRectifyMap(
    mtx, dist, None, newcameramtx, (w, h), 5)
dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

## crop the image ##
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png', dst)

## Calculate re-projection erorr ##
tot_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

print("Mean error: ", tot_error/len(objpoints))

#TODO: Retreive parameters

#################################################
## Get Extrinsic matrix #########################
#################################################

## Create axis ##
# Scale is relative
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((N_COLMNS*N_ROWS, 3), np.float32)
objp[:, :2] = np.mgrid[0:N_ROWS, 0:N_COLMNS].T.reshape(-1, 2)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

## Plot axis on top of the image ##
for fname in glob.glob(os.path.abspath(os.path.join(CAMERA_CALIB_SAMPLES, '*.jpg'))):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (N_ROWS, N_COLMNS), None)

    if ret == True:
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)

        ## Find the rotation and translation vectors. ##
        rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        ## project 3D points to image plane ##
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        ## Show image with axis ##
        img = draw(img, corners2, imgpts)
        cv2.imshow('img', img)
        k = cv2.waitKey(0) & 0xff
        if k == 's':
            cv2.imwrite(fname[:6]+'.png', img)

cv2.destroyAllWindows()
