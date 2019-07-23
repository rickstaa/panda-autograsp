"""Example script to test the GPUPacketPipeline of the pylibfreenect2 library
"""

## Import system packages ##
import sys
import logging

## Import Kinect packages ##
import pylibfreenect2 as pylfk2
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

## Import other python packages ##
import cv2
import numpy as np

#################################################
## Script settings ##############################
#################################################
## Optional parameters for registration. ##
# Set true if you need.
NEED_BIGDEPTH = True
NEED_COLOR_DEPTH_MAP = False

#################################################
## Initialize loggers ###########################
#################################################

## Create and set pylibfreenect logger ##
logger = createConsoleLogger(LoggerLevel.Info)
setGlobalLogger(logger)

## Create panda autograsp logger ##
logging.addLevelName(logging.INFO, 'Info')  # Capitalize INFO logger level
logging.addLevelName(logging.ERROR, 'Error')  # Capitalize ERROR logger level
logging.addLevelName(logging.DEBUG, 'Debug')  # Capitalize DEBUG logger level
# Capitalize WARNING logger level
logging.addLevelName(logging.WARNING, 'Warning')
# Capitalize CRITICAL logger level
logging.addLevelName(logging.CRITICAL, 'Critical')
# Capitalize NOTSET logger level
logging.addLevelName(logging.NOTSET, 'Notset')
logging.basicConfig(level=logging.INFO,
                    format='[%(levelname)s] [%(processName)s] %(message)s')
main_logger = logging.getLogger("main_logger")

#################################################
## Start kinect and process frames ##############
#################################################

## Open depth package pipline ##
try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline
        pipeline = OpenCLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
main_logger.info("Packet pipeline: " + type(pipeline).__name__)

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

## Get camera intrinsic and extrinsic parameters ##
color_intrinsics = device.getColorCameraParams()
ir_intrinsics = device.getIrCameraParams()

## Combine IR and Color data ##
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

## Create frame objects ##
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
    ## Retrieve frames        ##
    ############################

    ## Retrieve frames ##
    frames = listener.waitForNewFrame()
    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    ## Map colour images on depth images ##
    registration.apply(color, depth, undistorted, registered,
                       bigdepth=bigdepth,
                       color_depth_map=color_depth_map)

    ## Preprocess frame ##
    depth_frame = depth.asarray() / 4500. # Normalize depth data
    big_depth_frame = cv2.resize(bigdepth.asarray(np.float32), (int(1920 / 3), int(1082 / 3)))
    depth_data  = np.resize(depth_frame, [depth_frame.shape[0],depth_frame.shape[1],1]) # Get depth data in right format for GQCNN algorithm
    big_depth_data  = np.resize(big_depth_frame, [big_depth_frame.shape[0],big_depth_frame.shape[1],1]) # Get depth data in right format for GQCNN algorithm
    color_frame = cv2.resize(color.asarray(), (int(1920 / 3), int(1080 / 3))) # Reduce image size by factor of 3

    ############################
    ## Visualize frames       ##
    ############################
    # NOTE for visualization:
    # cv2.imshow without OpenGL backend seems to be quite slow to draw all
    # things below. Try commenting out some imshow if you don't have a fast
    # visualization backend.
    # TODO: Create depth image colormap
    #cv2.imshow("depth", depth_frame)
    cv2.imshow("color", color_frame)
    #cv2.imshow("registered", registered.asarray(np.uint8))
    #if NEED_BIGDEPTH:
    #    cv2.imshow("bigdepth", cv2.resize(bigdepth.asarray(np.float32),
    #                                      (int(1920 / 3), int(1082 / 3))))
    #if NEED_COLOR_DEPTH_MAP:
    #    cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))
    listener.release(frames)
    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

################################
## Shutdown kinect processing ##
################################
device.stop()
device.close()
sys.exit(0)