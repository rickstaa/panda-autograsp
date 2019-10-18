"""This script can be used to generate a Aruco marker board. This board is used in the
camera pose estimation.

Printing instructions:
    - Make sure Scale to fit is selected.
    - Verify the size of the markers after the board is printed. If
      size does not match change the settings below or in the detection algorithm.
"""

# Main python packages
import os
import pickle
import cv2
from cv2 import aruco
import datetime

# Panda_autograsp modules, msgs and srvs
from panda_autograsp import Logger

# Get required paths
SAVE_DIR_PATH = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))

# Create script logger
script_logger = Logger.get_logger("kinect_processing.py")

#################################################
# Script settings ###############################
#################################################

MARKERS_X = 4
MARKERS_Y = 6
MARKER_LENGTH = 0.032  # [M]
MARKER_SEPARATION = 0.009  # [M]
ARUCO_DICT_TYPE = aruco.DICT_6X6_50  # [ROWSxCOLUMNS_LIBRARY_SIZE]
IM_OUT_SIZE = (1100, 1681)  # Pixel size == Pattern size
MARGIN_SIZE = 0

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # set root logger format
    root_logger = Logger.get_logger(
        log_file=os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)), "..", "logs/plan_grasp.log"
            )
        )
    )

    # Welcome message
    print(
        "== Generate arucoboard script ==\n",
        "This script can be used to generate ",
        "a Aruco marker board. This board is used ",
        "in the camera pose estimation.",
    )

    # Save config settings
    config_dict = {
        "MARKERS_X": MARKERS_X,
        "MARKERS_Y": MARKERS_Y,
        "MARKER_LENGTH": 0.032,
        "MARKER_SEPARATION": 0.009,
        "ARUCO_DICT_TYPE": ARUCO_DICT_TYPE,
        "IM_OUT_SIZE": IM_OUT_SIZE,
        "MARGIN_SIZE": MARGIN_SIZE,
    }
    config_save_str = os.path.abspath(
        os.path.join(SAVE_DIR_PATH, "../cfg/_cfg/aruco_config.dict")
    )
    with open(config_save_str, "wb") as config_dict_file:
        pickle.dump(config_dict, config_dict_file)

    # Create aruco gridboard
    ARUCO_DICT = aruco.Dictionary_get(ARUCO_DICT_TYPE)
    gridboard = aruco.GridBoard_create(
        markersX=MARKERS_X,
        markersY=MARKERS_Y,
        markerLength=MARKER_LENGTH,
        markerSeparation=MARKER_SEPARATION,
        dictionary=ARUCO_DICT,
    )

    # Create an image from the gridboard
    gridboard_img = gridboard.draw(
        outSize=IM_OUT_SIZE, marginSize=MARGIN_SIZE, borderBits=1
    )

    # Display and save gridboard image
    cv2.namedWindow("Arcu-board")
    cv2.startWindowThread()
    cv2.imshow("Arcu-board", gridboard_img)
    while True:
        k = cv2.waitKey(100)

        # Break if someone presses q or esc
        if (k == 27) or (k == ord("q")):
            print("Closing window")
            cv2.destroyAllWindows()
            break

        # Break if window is closed
        if cv2.getWindowProperty("Arcu-board", cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()

    # Save generated Arucoboard
    time_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    img_save_pth = os.path.abspath(
        os.path.join(
            SAVE_DIR_PATH, ("../data/calib/arucoboard" + "_" + time_str + ".jpg")
        )
    )
    cv2.imwrite(img_save_pth, gridboard_img)
