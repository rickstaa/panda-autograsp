"""This script can be used retrieve ir, color, depth and registered images from
the Kinect camera. The images will be saved in the ``data/frames`` folder.

Usage:
    [q]: Stop the kinect_processing script.
    [s]: Save kinect frames.
"""

# Main python packages
import numpy as np
import os
import datetime
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame

# Panda_autograsp modules, msgs and srvs
from panda_autograsp import Logger

# Create script logger
script_logger = Logger.get_logger("kinect_processing.py")

#################################################
# Kinect process settings #######################
#################################################

NEED_BIGDEPTH = False  # Use Full size depth image
NEED_COLOR_DEPTH_MAP = False  # Overlay depth and colour
DATA_SAVE_FOLDER = os.path.abspath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "data/frames")
)

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Welcome message
    print(
        "== Kinect processing script ==\n"
        "This script can be used retrieve ir, color, \n"
        "depth and registered images from the Kinect \n "
        "camera. The images will be saved in the \n "
        "``data/frames`` folder.\n"
        "\n"
        "Usage: \n"
        "  [q]: Stop the kinect_processing script. \n"
        "  [s]: Save kinect frames.\n"
    )

    # set root logger format
    root_logger = Logger.get_logger(
        log_file=os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "..",
                "logs/kinect_processing.log",
            )
        )
    )

    #############################################
    # Kinect initiation settings ################
    #############################################

    # Open depth package pipline
    try:  # Check if GPU can be used
        from pylibfreenect2 import OpenCLPacketPipeline

        pipeline = OpenCLPacketPipeline()
    except ImportError:  # Otherwise open CPU
        from pylibfreenect2 import CpuPacketPipeline

        pipeline = CpuPacketPipeline()

    # Create freenect2 object and check sensor properties
    fn = Freenect2()
    num_devices = fn.enumerateDevices()
    if num_devices == 0:
        script_logger.error("No device connected!")
        sys.exit(0)
    serial = fn.getDeviceSerialNumber(0)
    device = fn.openDevice(serial, pipeline=pipeline)
    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

    # Register frame listeners
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)

    # Start data processing
    device.start()

    # Combine IR and Color data
    registration = Registration(
        device.getIrCameraParams(), device.getColorCameraParams()
    )

    # Set sensor frames formats
    undistorted = Frame(512, 424, 4)
    registered = Frame(512, 424, 4)
    bigdepth = Frame(1920, 1082, 4) if NEED_BIGDEPTH else None
    color_depth_map = (
        np.zeros((424, 512), np.int32).ravel() if NEED_COLOR_DEPTH_MAP else None
    )

    #############################################
    # Kinect frame process loop #################
    #############################################
    while True:

        # Get frames
        frames = listener.waitForNewFrame()
        color = frames["color"]
        ir = frames["ir"]
        depth = frames["depth"]

        # Process frames
        registration.apply(
            color,
            depth,
            undistorted,
            registered,
            bigdepth=bigdepth,
            color_depth_map=color_depth_map,
        )

        # Visualize frames
        # NOTE for visualization:
        # cv2.imshow without OpenGL backend seems to be quite slow to draw all
        # things below. Try commenting out some imshow if you don't have a fast
        # visualization backend.
        ir_img = cv2.imshow("ir", ir.asarray() / 65535.0)
        depth_img = cv2.imshow("depth", depth.asarray() / 4500.0)
        color_img = cv2.imshow(
            "color", cv2.resize(color.asarray(), (int(1920 / 3), int(1080 / 3)))
        )
        reg_img = cv2.imshow("registered", registered.asarray(np.uint8))
        if NEED_BIGDEPTH:
            cv2.imshow(
                "bigdepth",
                cv2.resize(
                    bigdepth.asarray(np.float32), (int(1920 / 3), int(1082 / 3))
                ),
            )
        if NEED_COLOR_DEPTH_MAP:
            cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))
        listener.release(frames)

        # Break if someone presses q or esc
        key = cv2.waitKey(delay=1)
        if (key == 27) or (key == ord("q")):
            break

        # Save frames to datafolder if user presses ctrl + s
        if key == ord("s"):

            # Save frames
            time_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            cv2.imwrite(DATA_SAVE_FOLDER + "/ir_" + time_str + ".png", ir_img)
            cv2.imwrite(DATA_SAVE_FOLDER + "/depth_" + time_str + ".png", depth_img)
            cv2.imwrite(DATA_SAVE_FOLDER + "/color_" + time_str + ".png", color_img)
            cv2.imwrite(DATA_SAVE_FOLDER + "/reg_" + time_str + ".png", reg_img)
            script_logger.info("Saving frames to data folder")

    # Shutdown kinect connection
    device.stop()
    device.close()
    sys.exit(0)
