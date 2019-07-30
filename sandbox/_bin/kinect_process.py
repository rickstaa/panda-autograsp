#"""This script is used to process the Kinect images and query them to the
#GQCNN algorithm.
#"""

## pylibfrenect imports
import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

## Other imports ##
import os
import time
import logging

## Get file path ##
file_path = os.path.dirname(os.path.abspath(__file__))
main_path = os.path.abspath(os.path.join(file_path, os.pardir))
data_path = os.path.abspath(os.path.join(main_path, 'data'))

## Create data/tmp folders if it does not exists ##
if not os.path.exists(os.path.join(data_path, 'tmp')):
    os.makedirs(os.path.join(data_path, 'tmp'))

################################
## Script settings #############
################################
# Other script parameters
VISUALIZE = False

################################
## Kinect process settings #####
################################
## Optional parameters for registration. ##
# Set true if you need.
NEED_BIGDEPTH = True
NEED_COLOR_DEPTH_MAP = False

################################
## Initialize loggers ##########
################################

## Create and set pylibfrenect logger ##
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

    ############################
    ## save frames            ##
    ############################
    # Save frames to datafolder if
    # user clicks ctrl + s
    if key == ord('s'):

        ## Create image save names ##
        timestamp = time.strftime("%H%M%S") # Create file timestamp
        big_depth_save = os.path.join(data_path, 'tmp', 'depth_'+timestamp+'.npy')
        color_save = os.path.join(data_path, 'tmp', 'color_'+timestamp+'.png')

        ## Save frames ##
        panda_logger.info("Saving frames to data folder")
        np.save(big_depth_save, big_depth_data) # Save depth frame
        cv2.imwrite(color_save,color_frame) # Save color frame
        panda_logger.info("Frames saved to data folder")

        ############################
        ## Create segmask         ##
        ############################
        # Done with method of https://docs.opencv.org/3.1.0/d3/db4/tutorial_py_watershed.html

        # Find approximate estimate of objects ##
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) # Use otsu algorithm for finding treshhold
        cv2.imshow("trash", gray)

        # noise removal
        kernel = np.ones((3,3), np.uint8)
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations = 2)

        # Background area using Dialation
        sure_bg = cv2.dilate(closing, kernel, iterations = 1)

        # Finding foreground area
        dist_transform = cv2.distanceTransform(closing, cv2.DIST_L2, 0)
        ret, fg = cv2.threshold(dist_transform, 0.02
                        * dist_transform.max(), 255, 0)
        cv2.imshow("image",fg)

        # Finding unknown region
        #sure_fg = np.uint8(sure_fg)
        #unknown = cv2.subtract(sure_bg, sure_fg)

        # Show sure foreground and background
        #cv2.imshow("surefore", sure_fg)
        #cv2.imshow("sureback", sure_bg)

        # Make JET collormap
        #imC = cv2.applyColorMap(unknown, cv2.COLORMAP_JET)
        #cv2.imshow("test3", imC)

        # Marker labelling
        #ret, markers = cv2.connectedComponents(sure_fg)

        # Add one to all labels so that sure background is not 0, but 1
        #markers = markers+1

        # Now, mark the region of unknown with zero
        #markers[unknown==255] = 0

        # Perform watershed algorithm
#        markers = cv2.watershed(color_frame, markers)
#        color_frame[markers == -1] = [255,0,0]


################################
## Shutdown kinect processing ##
################################
device.stop()
device.close()
sys.exit(0)