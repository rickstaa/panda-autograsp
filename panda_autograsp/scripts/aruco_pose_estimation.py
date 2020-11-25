"""This script can be used to watch a video stream, detect a Aruco board, and use
it to determine the posture of the camera in relation to the plane
of markers.

.. warning::

    - This script assumes that all markers are on the same plane.
    - This script requires the camera to be calibrated.

Source code
----------------------------
.. literalinclude:: /../../panda_autograsp/scripts/aruco_pose_estimation.py
   :language: python
   :linenos:
   :lines: 18-
"""

# Main python packages
import sys
import cv2
import cv2.aruco as aruco
import os
import pickle
from perception import Kinect2Sensor
import numpy as np
import copy

# Panda_autograsp modules, msgs and srvs
from panda_autograsp import Logger

# Create Logger
script_logger = Logger.get_logger("aruco_pose_estimation.py")

#################################################
# Script settings ###############################
#################################################
FULL_SCREEN = False  # Open result on full screen
POSE_ARROW_SIZE = 0.1  # [M]
LOAD_DIR_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "../cfg/_cfg/aruco_config.dict"
    )
)  # Retrieve full path

# Result save location
SAVE_CALIB_POSE_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "..",
        "data",
        "calib",
        "calib_pose_aruco.npz",
    )
)  # Retrieve full path

#################################################
# Get aruco config file settings ################
#################################################

# Initialize board parameters
with open(LOAD_DIR_PATH, "rb") as config_dict_file:
    config_dict = pickle.load(config_dict_file)  # Load the aruco board settings

# Overwrite settings based on measurements
config_dict["MARKER_LENGTH"] = 0.032  # [M]
config_dict["MARKER_SEPERATION"] = 0.009  # [M]
ARUCO_DICT = aruco.Dictionary_get(config_dict["ARUCO_DICT_TYPE"])
aruco_board = aruco.GridBoard_create(
    markersX=config_dict["MARKERS_X"],
    markersY=config_dict["MARKERS_Y"],
    markerLength=config_dict["MARKER_LENGTH"],
    markerSeparation=config_dict["MARKER_SEPARATION"],
    dictionary=ARUCO_DICT,
)
ARUCO_PARAMETERS = aruco.DetectorParameters_create()

# Initialize rvec and tvec vectors
# These will be used as an initial guess in the pose estimation.
rvec, tvec = None, None

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
                "logs/aruco_pose_estimation.log",
            )
        )
    )

    # Welcome message
    print(
        "== Aruco pose estimation script ==\n"
        "This script can be used to watch a video "
        "stream, detect a Aruco board, and use "
        "it to determine the posture of the camera "
        " in relation to the plane of markers.\n"
        "\n"
        "Usage: \n"
        "  [q]: Stop the kinect_processing script. \n"
        "  [s]: Save pose estimation results.\n"
    )

    #################################################
    # Start Kinect sensor ###########################
    #################################################
    sensor = Kinect2Sensor()
    sensor.start()

    # Create opencv window
    if FULL_SCREEN:
        cv2.namedWindow("Acuco_pose", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(
            "Acuco_pose", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        )

    # Sensor processing loop
    while sensor.is_running:

        # Retreive camera calibration values
        camera_matrix = sensor.color_intrinsics.K
        ir_intr = sensor._device.getIrCameraParams()
        ir_mtx = [ir_intr.k1, ir_intr.k2, ir_intr.p1, ir_intr.p2, ir_intr.k3]
        dist_coeffs = np.array(ir_mtx)

        # Get image frame and convert to gray
        color_im, _, ir_im = sensor.frames(True)
        gray = cv2.cvtColor(color_im.data, cv2.COLOR_BGR2GRAY)

        # Create screen display image
        # Needed since opencv uses BGR instead of RGB
        screen_img = cv2.cvtColor(copy.copy(color_im.data), cv2.COLOR_RGB2BGR)

        # Detect aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            image=gray,
            dictionary=ARUCO_DICT,
            parameters=ARUCO_PARAMETERS,
            # camMatrix=camera_matrix,
            # distCoeff=dist_coeffs,
        )

        # Refine detected markers
        # Eliminates markers not part of our board, adds missing markers to the board
        corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
            image=gray,
            board=aruco_board,
            detectedCorners=corners,
            detectedIds=ids,
            rejectedCorners=rejectedImgPoints,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
        )

        # If at least one marker was found try to estimate the pose
        if ids is not None and ids.size > 0:

            # Outline all of the markers detected in our image
            screen_img = aruco.drawDetectedMarkers(screen_img, corners, ids)

            # Estimate pose
            retval, rvec, tvec = aruco.estimatePoseBoard(
                corners, ids, aruco_board, camera_matrix, dist_coeffs, rvec, tvec
            )

            # If pose estimation was successful draw pose
            if retval > 0:
                aruco.drawAxis(
                    screen_img, camera_matrix, dist_coeffs, rvec, tvec, POSE_ARROW_SIZE
                )

        # Display our image
        cv2.imshow("Acuco_pose", screen_img)
        k = cv2.waitKey(delay=1)
        if k == ord("s"):

            # Save rotation and transformation matrix
            script_logger.info(
                "Saving external pose calibration results at %s." % SAVE_CALIB_POSE_PATH
            )
            np.savez(SAVE_CALIB_POSE_PATH, rvecs=rvec, tvecs=tvec)

        # Break if someone presses q
        if k == ord("q"):
            script_logger.info("Closing aruco pose estimation script.")
            sys.exit(0)

    cv2.destroyAllWindows()
