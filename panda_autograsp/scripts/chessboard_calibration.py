""" This script can be used to calibrate the kinect camera using a chessboard.
calculates the intrinsic camera matrix and the distortion parameters.

Usage:
    [q]: Stop the chessboard_calibration script.
    [s]: Save the frames/calibration results.
"""

# Main python packages
import numpy as np
import cv2
import os
import sys
import copy
import datetime

from perception import Kinect2Sensor, CameraIntrinsics

# Panda_autograsp modules, msgs and srvs
from panda_autograsp import Logger
from panda_autograsp.functions import draw_axis

# Create Logger
script_logger = Logger.get_logger("chessboard_calibration.py")

#################################################
# Script settings ###############################
#################################################

# General settings
FULL_SCREEN = True  # full screen
FACTORY = True  # Use libfreenect2 factory camera parameters

# Calibration settings
criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)  # termination criteria
N_FRAMES = 10  # Take the mean over N_frames
N_ROWS = 5  # Inner chessboard rows
N_COLMNS = 7  # Inner chessboard columns
SQUARE_SIZE = 30  # [mm] The square size of a chessboard square

# Results save location
SAVE_PATH = os.path.abspath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "data/calib")
)  # Folder in which you want to save the calib results

#################################################
# Other script parameters #######################
#################################################

# Text default settings
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.90
fontColor = (0, 0, 0)
lineType = 1
rectangle_bgr = (255, 255, 255)

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # set root logger format
    root_logger = Logger.get_logger(
        log_file=os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "..",
                "logs/chessboard_calibration.log",
            )
        )
    )

    # Welcome message
    print(
        "\n== Chessboard calibration script ==\n"
        "This script can be used to calibrate the "
        "calculates the intrinsic camera matrix and "
        "the distortion parameters.\n"
        "\n"
        "Usage:\n"
        "  [q]: Stop the chessboard_calibration script.\n"
        "  [s]: Save the frames/calibration results.\n"
    )

    #############################################
    # Start Kinect sensor #######################
    #############################################
    sensor = Kinect2Sensor()
    try:
        sensor.start()
    except IOError:
        script_logger.warn(
            "No kinect device detected make sure the kinect is connected."
        )
        script_logger.info("Shutting down chessboard_calibration script.")
        sys.exit(0)

    #############################################
    # Get Intrinsic  color matrix ###############
    #############################################

    # Setup results save name
    time_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    NUMPY_SAVE = os.path.abspath(
        os.path.join(SAVE_PATH, ("calib_intr_%s.npz" % time_str))
    )
    BERKLEY_SAVE = os.path.abspath(
        os.path.join(SAVE_PATH, ("calib_intr_%s.intr" % time_str))
    )
    CAMERA_POSE_SAVE = os.path.abspath(
        os.path.join(SAVE_PATH, ("calib_pose_%s.npz" % time_str))
    )

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # Multiply with size of square to et the result in mm
    objp = np.zeros((N_COLMNS * N_ROWS, 3), np.float32)

    # Multiply by chessboard scale factor
    objp[:, :2] = np.mgrid[0:N_ROWS, 0:N_COLMNS].T.reshape(-1, 2) * SQUARE_SIZE

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    axis = (
        np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3) * SQUARE_SIZE
    )  # Coordinate axis
    if not FACTORY:

        # Start message
        script_logger.info("--- Kinect2 camera calibration ---")
        script_logger.info("Starting kinect2 calibration procedure.")

        # Create opencv window
        if FULL_SCREEN:
            cv2.namedWindow("Calibration", cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(
                "Calibration", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
            )

        # Get a number of images
        images = []
        frame = 0
        while True:
            if frame < N_FRAMES:

                # Visualize current image
                color_im, _, ir_im = sensor.frames(True)
                gray = cv2.cvtColor(color_im.data, cv2.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, (N_ROWS, N_COLMNS), None)

                # If found, add object points, image points (after refining them)
                if ret:

                    # Improve found corners ##
                    corners2 = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1), criteria
                    )

                    # Draw and display the corners
                    img_detections = cv2.drawChessboardCorners(
                        color_im.data, (N_ROWS, N_COLMNS), corners2, ret
                    )

                    # Create screen display image #
                    screen_img = cv2.cvtColor(
                        copy.copy(img_detections), cv2.COLOR_RGB2BGR
                    )

                    # Write text on image
                    text_offset_x = 10
                    text_offset_y = 910
                    text_width = 550
                    text_height = 160
                    box_coords = (
                        (text_offset_x - 10, text_offset_y - 10),
                        (
                            text_offset_x + text_width + 10,
                            text_offset_y + text_height + 10,
                        ),
                    )
                    cv2.rectangle(
                        screen_img,
                        box_coords[0],
                        box_coords[1],
                        rectangle_bgr,
                        cv2.FILLED,
                    )
                    cv2.putText(
                        screen_img,
                        "=Info=",
                        (text_offset_x, text_offset_y + 20),
                        font,
                        fontScale,
                        fontColor,
                        lineType,
                    )
                    cv2.putText(
                        screen_img,
                        "press s to save a calibration image",
                        (text_offset_x, text_offset_y + 55),
                        font,
                        fontScale,
                        fontColor,
                        lineType,
                    )
                    cv2.putText(
                        screen_img,
                        "press q to exit the calibration",
                        (text_offset_x, text_offset_y + 90),
                        font,
                        fontScale,
                        fontColor,
                        lineType,
                    )
                    cv2.putText(
                        screen_img,
                        "Computing intrinsic parameters",
                        (text_offset_x, text_offset_y + 125),
                        font,
                        fontScale,
                        (72, 193, 80),
                        lineType,
                    )
                    cv2.putText(
                        screen_img,
                        ("%i of the %i images taken" % (frame, N_FRAMES)),
                        (text_offset_x, text_offset_y + 160),
                        font,
                        fontScale,
                        (255, 0, 0),
                        lineType,
                    )

                    # Show image
                    cv2.imshow("Calibration", screen_img)
                else:

                    # Create screen display image
                    screen_img = cv2.cvtColor(
                        copy.copy(color_im.data), cv2.COLOR_RGB2BGR
                    )

                    # Write text on image
                    text_offset_x = 10
                    text_offset_y = 910
                    text_width = 550
                    text_height = 160
                    box_coords = (
                        (text_offset_x - 10, text_offset_y - 10),
                        (
                            text_offset_x + text_width + 10,
                            text_offset_y + text_height + 10,
                        ),
                    )
                    cv2.rectangle(
                        screen_img,
                        box_coords[0],
                        box_coords[1],
                        rectangle_bgr,
                        cv2.FILLED,
                    )
                    cv2.putText(
                        screen_img,
                        "=Info=",
                        (text_offset_x, text_offset_y + 20),
                        font,
                        fontScale,
                        fontColor,
                        lineType,
                    )
                    cv2.putText(
                        screen_img,
                        "press s to save a calibration image",
                        (text_offset_x, text_offset_y + 55),
                        font,
                        fontScale,
                        fontColor,
                        lineType,
                    )
                    cv2.putText(
                        screen_img,
                        "press q to exit the calibration",
                        (text_offset_x, text_offset_y + 90),
                        font,
                        fontScale,
                        fontColor,
                        lineType,
                    )
                    cv2.putText(
                        screen_img,
                        "Computing intrinsic parameters",
                        (text_offset_x, text_offset_y + 125),
                        font,
                        fontScale,
                        (72, 193, 80),
                        lineType,
                    )
                    cv2.putText(
                        screen_img,
                        ("%i of the %i images taken" % (frame, N_FRAMES)),
                        (text_offset_x, text_offset_y + 160),
                        font,
                        fontScale,
                        (255, 0, 0),
                        lineType,
                    )

                    # Show image
                    cv2.setWindowProperty(
                        "Calibration", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
                    )
                    cv2.imshow("Calibration", screen_img)

                # Process key events
                key = cv2.waitKey(delay=1)

                # Break if someone presses q or esc
                if (key == 27) or (key == ord("q")):
                    sys.exit(0)

                #################################
                # save frames ###################
                #################################

                # Save frames to list if user clicks s
                if key == ord("s"):
                    frame += 1

                    # Get color, depth and ir image frames
                    script_logger.info(
                        "%i of the %i calibration images taken. Click s to take "
                        "another image." % (frame, N_FRAMES)
                    )
                    if ret:
                        objpoints.append(objp)
                        imgpoints.append(corners2)
                    else:
                        script_logger.warning("No chessboard found please try again.")
            else:
                break

        # Close all image windows
        cv2.destroyAllWindows()

        # Perform calibration
        ret, color_mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        # Retrieve intr parameters out of camera matrix
        f_x = color_mtx[0, 0]  # x focal length
        f_y = color_mtx[1, 1]  # Y focal length
        c_x = color_mtx[0, 2]  # x optical center
        c_y = color_mtx[1, 2]  # y optical center
        s = color_mtx[1, 0]  # skew

        # Unwrap distortion parameters
        k_1 = dist[0][0]  # First radial distortion coefficient
        k_2 = dist[0][1]  # Second ...
        k_3 = dist[0][4]  # Third ...
        p_1 = dist[0][2]  # First tangential distortion coefficient
        p_2 = dist[0][3]  # Second ..

        # Print information
        # NOTE: Skew is set to 0 in opencv2
        print("=== Calibration results ==")
        print("Reprojection error RMS: %f" % ret)
        print("Distortion parameters:")
        print(dist[0])
        print("Camera matrix:")
        print(color_mtx)

        # Calculate re-projection error
        tot_error = 0
        for index, value in enumerate(objpoints):
            imgpoints2, _ = cv2.projectPoints(
                objpoints[index], rvecs[index], tvecs[index], color_mtx, dist
            )
            error = cv2.norm(imgpoints[index], imgpoints2, cv2.NORM_L2) / len(
                imgpoints2
            )
            tot_error += error
        print("Mean error: ", tot_error / len(objpoints))

        ##################################################
        # Save camera matrix and distortion coefficients #
        ##################################################

        # Save as numpy array
        np.savez(NUMPY_SAVE, color_mtx=color_mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

        # Save as berkeley format file
        camera_intr_obj = CameraIntrinsics("none", f_x, f_y, c_x, c_y, s)
        camera_intr_obj.save(BERKLEY_SAVE)

    ##################################################
    # Get chessboard/world pose estimation ############
    ##################################################

    # Get intrinsic parameters if factory is set
    if FACTORY:

        # Retrieve camera parameters
        color_intr = sensor.color_intrinsics
        ir_intr = sensor._device.getIrCameraParams()
        color_mtx = color_intr.K
        ir_mtx = [ir_intr.k1, ir_intr.k2, ir_intr.p1, ir_intr.p2, ir_intr.k3]
        dist = np.array(ir_mtx)
    else:

        # Load camera parameters
        try:
            time_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            with np.load(NUMPY_SAVE) as X:
                color_mtx, dist, _, _ = [
                    X[item] for item in ("color_mtx", "dist", "rvecs", "tvecs")
                ]
        except IOError:
            script_logger.warn("Camera parameters could not be loaded.")

    # Print information
    print(
        "\nTo determine the camera/world external matrix put"
        "the chessboard in the upper left corner of the robot"
        "table. Following click [s] to save the calibration"
        "results. You can click [q] to exit."
    )

    # Create opencv window
    if FULL_SCREEN:
        cv2.namedWindow("Pose", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Pose", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # Show results
    while True:

        # Get color image
        color_im, _, _ = sensor.frames(True)
        gray = cv2.cvtColor(color_im.data, cv2.COLOR_BGR2GRAY)

        # Create screen display image
        # Needed since opencv uses BGR instead of BGR
        screen_img = cv2.cvtColor(copy.copy(color_im.data), cv2.COLOR_RGB2BGR)

        # Write text on image
        text_offset_x = 10
        text_offset_y = 970
        text_width = 630
        text_height = 100
        box_coords = (
            (text_offset_x - 10, text_offset_y - 10),
            (text_offset_x + text_width + 10, text_offset_y + text_height + 10),
        )
        cv2.rectangle(
            screen_img, box_coords[0], box_coords[1], rectangle_bgr, cv2.FILLED
        )
        cv2.putText(
            screen_img,
            "=Info=",
            (text_offset_x, text_offset_y + 20),
            font,
            fontScale,
            fontColor,
            lineType,
        )
        cv2.putText(
            screen_img,
            "press s to save a camera/world translation",
            (text_offset_x, text_offset_y + 55),
            font,
            fontScale,
            fontColor,
            lineType,
        )
        cv2.putText(
            screen_img,
            "press q to exit the calibration",
            (text_offset_x, text_offset_y + 90),
            font,
            fontScale,
            fontColor,
            lineType,
        )

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (N_ROWS, N_COLMNS), None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Find the rotation and translation vectors.
            _, rvecs, tvecs, inliers = cv2.solvePnPRansac(
                objp, corners2, color_mtx, dist
            )

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, color_mtx, dist)

            # Show image
            screen_img = draw_axis(screen_img, corners2, imgpts)
            cv2.imshow("Pose", screen_img)

            # Check for user commands
            key = cv2.waitKey(delay=1)
            if key == ord("s"):

                # Save rotation and transformation matrix
                script_logger.info(
                    "Saving chessboard calibration results at %s." % (CAMERA_POSE_SAVE)
                )
                np.savez(
                    CAMERA_POSE_SAVE,
                    mtx=color_mtx,
                    dst=dist,
                    rvecs=rvecs,
                    tvecs=tvecs,
                    inliers=inliers,
                )

                # Close figure
                script_logger.info("Closing aruco pose estimation script.")
                sys.exit(0)

            # Break if someone presses q or esc
            if (key == 27) or (key == ord("q")):
                script_logger.info("Closing aruco pose estimation script.")
                sys.exit(0)

    cv2.destroyAllWindows()
