"""This script is used to process the Kinect images and qeury them to the
GQCNN algorithm.
"""

## pylibfrenect imports
import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

## Other imports ##
import logging

################################
## Kinect process settings #####
################################
## Optional parameters for registration. ##
# Set true if you need.
NEED_BIGDEPTH = False
NEED_COLOR_DEPTH_MAP = False

## Other settings ##
DATA_SAVE_FOLDER = "../data/"

################################
## Initialize loggers ##########
################################

## Create and set pylibfrenect logger ##
logger = createConsoleLogger(LoggerLevel.Info)
setGlobalLogger(logger)

## Create panda autograsp logger ##
logging.addLevelName(logging.INFO, 'Info') # Capitalize INFO logger level
logging.addLevelName(logging.ERROR, 'Error') # Capitalize ERROR logger level
logging.addLevelName(logging.DEBUG, 'Debug') # Capitalize DEBUG logger level
logging.addLevelName(logging.WARNING, 'Warning') # Capitalize WARNING logger level
logging.addLevelName(logging.CRITICAL, 'Critical') # Capitalize CRITICAL logger level
logging.addLevelName(logging.NOTSET, 'Notset') # Capitalize NOTSET logger level
logging.basicConfig(level=logging.INFO,
                    format='[%(levelname)s] [%(processName)s] %(message)s')
panda_logger = logging.getLogger("panda_logger")

################################
## Kinect process initiation ###
################################

## Open depth package pipline ##
try:  # Check if GPU can be used
    from pylibfreenect2 import OpenCLPacketPipeline
    pipeline = OpenCLPacketPipeline()
except:  # Otherwise open CPU
    from pylibfreenect2 import CpuPacketPipeline
    pipeline = CpuPacketPipeline()

## Create freenect2 object and check sensor properties ##
fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)
serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)
listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)

## Register listeners ##
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

## Start data processing ##
# NOTE: must be called after device.start()
device.start()

## Combine IR and Color data ##
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

## Setup frames format ##
undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)
bigdepth = Frame(1920, 1082, 4) if NEED_BIGDEPTH else None
color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
    if NEED_COLOR_DEPTH_MAP else None

################################
## Kinect frame process loop ###
################################
while True:

    ############################
    ## Retreive frames        ##
    ############################
    ## Get frames ##
    frames = listener.waitForNewFrame()
    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    ## Process frames ##
    registration.apply(color, depth, undistorted, registered,
                       bigdepth=bigdepth,
                       color_depth_map=color_depth_map)

    ############################
    ## Visualize frames       ##
    ############################
    # NOTE for visualization:
    # cv2.imshow without OpenGL backend seems to be quite slow to draw all
    # things below. Try commenting out some imshow if you don't have a fast
    # visualization backend.
    cv2.imshow("ir", ir.asarray() / 65535.)
    cv2.imshow("depth", depth.asarray() / 4500.)
    cv2.imshow("color", cv2.resize(color.asarray(),
                                   (int(1920 / 3), int(1080 / 3))))
    cv2.imshow("registered", registered.asarray(np.uint8))
    if NEED_BIGDEPTH:
        cv2.imshow("bigdepth", cv2.resize(bigdepth.asarray(np.float32),
                                          (int(1920 / 3), int(1082 / 3))))
    if NEED_COLOR_DEPTH_MAP:
        cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))
    listener.release(frames)
    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

    ############################
    ## save frames            ##
    ############################
    # Save frames to datafolder if
    # user clicks ctrl + s
    if key == ord('s'):

        ## Save frames ##
        panda_logger.info("Saving frames to data folder")

################################
## Shutdown kinect processing ##
################################
device.stop()
device.close()
sys.exit(0)
