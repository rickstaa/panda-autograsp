"""Simple calibration script created with the opencv2 module. This script was based on https://docs.opencv.org/3.4.3/dc/dbb/tutorial_py_calibration.html.
"""

## Standard library imports ##
import numpy as np
import cv2
import os
import sys
import logging
import copy

## Third party imports ##
logger = logging.getLogger()
logger.disabled = True  # Done to suppress perception warnings
from perception import (Kinect2Sensor)
logger.disabled = False

## Custom imports ##
from panda_autograsp import Logger

#################################################
## Script settings ##############################
#################################################

## Script settings ##
vis = True  # Visualize results
factory = False # Use libfreenect2 camera parameters

## Calibration settings ##
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30, 0.001)  # termination criteria
N_FRAMES = 10
# Row size -1 (see https://stackoverflow.com/questions/17993522/opencv-findchessboardcorners-function-is-failing-in-a-apparently-simple-scenar)
N_ROWS = 7
# Column size -1 (see https://stackoverflow.com/questions/17993522/opencv-findchessboardcorners-function-is-failing-in-a-apparently-simple-scenar)
N_COLMNS = 10
SQUARE_SIZE = 34  # The square size in mm

## Text default settings ##
font                   = cv2.FONT_HERSHEY_SIMPLEX
fontScale              = 0.90
fontColor              = (0, 0, 0)
lineType               = 1
rectangle_bgr          = (255, 255, 255)

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
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Create Logger ##
    script_logger = Logger.get_logger(__name__)

    #################################################
    ## Parse arguments ##############################
    #################################################
    ## TODO: ADD arg parser

    #################################################
    ## Start Kinect sensor ##########################
    #################################################
    sensor = Kinect2Sensor()
    sensor.start()

    #################################################
    ## Get Intrinsic  color matrix ##################
    #################################################
    if not factory:

        ## Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) ##
        # Multiply with size of square to et the result in mm
        objp = np.zeros((N_COLMNS*N_ROWS, 3), np.float32)
        # Multiply by chessboard scale factor
        objp[:, :2] = np.mgrid[0:N_ROWS, 0:N_COLMNS].T.reshape(-1, 2) * SQUARE_SIZE

        ## Arrays to store object points and image points from all the images. ##
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.
        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3) * SQUARE_SIZE # Coordinate axis

        ## Start message ##
        script_logger.info("--- Kinect2 camera calibration ---")
        script_logger.info("Starting kinect2 calibration procedure.")

        ## Get a number of images ##
        images = []
        i = 0
        while True:
            if i < N_FRAMES:

                ## Visualize current image ##
                color_im, _, ir_im = sensor.frames(True)
                gray = cv2.cvtColor(color_im.data, cv2.COLOR_BGR2GRAY)

                ## Find the chess board corners ##
                ret, corners = cv2.findChessboardCorners(
                    gray, (N_ROWS, N_COLMNS), None)

                ## If found, add object points, image points (after refining them) ##
                if ret == True:

                    # Improve found corners ##
                    corners2 = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1), criteria)

                    # Draw and display the corners
                    img_detections = cv2.drawChessboardCorners(
                        color_im.data, (N_ROWS, N_COLMNS), corners2, ret)

                    # Create screen display image #
                    screen_img = cv2.cvtColor(copy.copy(img_detections), cv2.COLOR_RGB2BGR)

                    ## Write text on image ##
                    text_offset_x = 10
                    text_offset_y = 910
                    text_width = 550
                    text_height = 160
                    box_coords = ((text_offset_x - 10, text_offset_y - 10), (text_offset_x + text_width  + 10, text_offset_y + text_height + 10))
                    cv2.rectangle(screen_img, box_coords[0], box_coords[1], rectangle_bgr, cv2.FILLED)
                    # putText(image, text, text location, font, fontscale, fontcolor,linetype)
                    cv2.putText(screen_img, '=Info=',
                                (text_offset_x, text_offset_y + 20),
                                font,
                                fontScale,
                                fontColor,
                                lineType)
                    cv2.putText(screen_img, 'press s to save a calibration image',
                                (text_offset_x, text_offset_y + 55),
                                font,
                                fontScale,
                                fontColor,
                                lineType)
                    cv2.putText(screen_img, 'press q to exit the calibration',
                                (text_offset_x, text_offset_y + 90),
                                font,
                                fontScale,
                                fontColor,
                                lineType)
                    cv2.putText(screen_img, 'Computing intrinsic parameters',
                                (text_offset_x, text_offset_y + 125),
                                font,
                                fontScale,
                                (72, 193, 80),
                                lineType)
                    cv2.putText(screen_img, ('%i of the %i images taken' % (i, N_FRAMES)),
                                (text_offset_x, text_offset_y + 160),
                                font,
                                fontScale,
                                (255, 0, 0),
                                lineType)

                    ## Show image ##
                    cv2.imshow('img', screen_img)
                else:

                    ## Create screen display image ##
                    screen_img = cv2.cvtColor(copy.copy(color_im.data), cv2.COLOR_RGB2BGR)

                    ## Write text on image ##
                    text_offset_x = 10
                    text_offset_y = 910
                    text_width = 550
                    text_height = 160
                    box_coords = ((text_offset_x - 10, text_offset_y - 10), (text_offset_x + text_width  + 10, text_offset_y + text_height + 10))
                    cv2.rectangle(screen_img, box_coords[0], box_coords[1], rectangle_bgr, cv2.FILLED)
                    # putText(image, text, text location, font, fontscale, fontcolor,linetype)
                    cv2.putText(screen_img, '=Info=',
                                (text_offset_x, text_offset_y + 20),
                                font,
                                fontScale,
                                fontColor,
                                lineType)
                    cv2.putText(screen_img, 'press s to save a calibration image',
                                (text_offset_x, text_offset_y + 55),
                                font,
                                fontScale,
                                fontColor,
                                lineType)
                    cv2.putText(screen_img, 'press q to exit the calibration',
                                (text_offset_x, text_offset_y + 90),
                                font,
                                fontScale,
                                fontColor,
                                lineType)
                    cv2.putText(screen_img, 'Computing intrinsic parameters',
                                (text_offset_x, text_offset_y + 125),
                                font,
                                fontScale,
                                (72, 193, 80),
                                lineType)
                    cv2.putText(screen_img, ('%i of the %i images taken' % (i, N_FRAMES)),
                                (text_offset_x, text_offset_y + 160),
                                font,
                                fontScale,
                                (255, 0, 0),
                                lineType)

                    ## Show image ##
                    cv2.imshow('img', screen_img)

                ## Process key events ##
                key = cv2.waitKey(delay=1)

                ## Break if someone presses q ##
                if key == ord('q'):
                    sys.exit(0)

                ############################
                ## save frames            ##
                ############################
                # Save frames to list if user clicks s
                if key == ord('s'):
                    i += 1

                    ## Get color, depth and ir image frames ##
                    script_logger.info("%i of the %i calibration images taken. Click s to take another image." % (i, N_FRAMES))
                    if ret == True:
                        objpoints.append(objp)
                        imgpoints.append(corners2)
                    else:
                        script_logger.warning("No chessboard found please try again.")
            else:
                break

        ## Close all image windows ##
        cv2.destroyAllWindows()

        ## Perform calibration ##
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)

        ## Retrieve intr parameters out of camera matrix ##
        f_x = mtx[0, 0]  # x focal length
        f_y = mtx[1, 1]  # Y focal length
        c_x = mtx[0, 2]  # x optical center
        c_y = mtx[1, 2]  # y optical center
        s = mtx[1, 0]  # skew

        ## Unwarp distortion parameters ##
        k_1 = dist[0][0]  # First radial distortion coefficient
        k_2 = dist[0][1]  # Second ...
        k_3 = dist[0][4]  # Third ...
        p_1 = dist[0][2]  # First tangential distortion coefficient
        p_2 = dist[0][3]  # Second ..

        ## Print information ##
        # NOTE: Skew is set to 0 in opencv2 https://stackoverflow.com/questions/23649477/how-to-get-skew-from-intrinics-distortion
        print("=== Calibration results ==")
        print("Reprojection error RMS: %f" % ret)
        print("Distortion parameters:")
        print(dist[0])
        print("Camera matrix:")
        print(mtx)

        ## Calculate re-projection error ##
        tot_error = 0
        for index, value in enumerate(objpoints):
            imgpoints2, _ = cv2.projectPoints(
                objpoints[index], rvecs[index], tvecs[index], mtx, dist)
            error = cv2.norm(imgpoints[index], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error
        print("Mean error: ", tot_error/len(objpoints))

        ###################################################
        ## Save camera matrix and distortion coefficients #
        ###################################################

        ## Save as numpy array ##
        np.savez("calib_results.npz", mtx=mtx, dst=dist, rvecs=rvecs, tvec=tvecs)

        # ## Save as berkeley format file ##
        # camera_intr_obj = CameraIntrinsics("none", f_x, f_y, c_x, c_y, s) # Create Berkeley perception intrinsic camera parmeter format
        # camera_intr_obj.save("calib_intr.intr")

        # ## Reload numpy array ##
        # with np.load('B.npz') as X:
        #     mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

    ##################################################
    ## Get IR intrinsic parameters ###################
    ##################################################

    ##################################################
    ## Display extrensic #############################
    ##################################################

    ## Get intrinsic parameters if factory is set ##
    if factory:
        color_intr = sensor.color_intrinsics
        ir_intr = sensor.ir_intrinsics
        color_mtx = color_intr.K
        ir_mtx = ir_intr.K

    ## Show results ##
    while True:

        ## Get color image ##
        color_im, _, _ = sensor.frames(True)
        gray = cv2.cvtColor(color_im.data, cv2.COLOR_BGR2GRAY)

        ## Create screen display image ##
        screen_img = cv2.cvtColor(copy.copy(color_im.data), cv2.COLOR_RGB2BGR)

        ## Find the chess board corners ##
        ret, corners = cv2.findChessboardCorners(gray, (N_ROWS, N_COLMNS), None)

        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

            # Find the rotation and translation vectors.
            _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            screen_img = draw_axis(screen_img, corners2, imgpts)
            cv2.imshow('img',screen_img)
            k = cv2.waitKey(delay=1)
            if k == ord('s'):

                ## Save rotation and transformation matrix
                np.savez("calib_result_extr.npz", rvecs=rvecs, tvecs=tvecs, inliers=inliers)
                # cv2.imwrite(fname[:6]+'.png', img)

            ## Break if someone presses q ##
            if key == ord('q'):
                sys.exit(0)

    cv2.destroyAllWindows()
