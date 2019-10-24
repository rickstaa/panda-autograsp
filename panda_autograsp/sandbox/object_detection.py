"""This script can be used retrieve ir, color, depth and registered images from
the Kinect camera. And detect objects in these images.
"""

# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# Main python packages
import numpy as np
import os
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame

from autolab_core import YamlConfig
from perception import (
    ColorImage,
    RgbdForegroundMaskQueryImageDetector,
    CameraIntrinsics,
    DepthImage,
)
import matplotlib.pyplot as plt

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


def detect_object(color_im, depth_im, camera_intr, config):
    """ Detects an object in the given images """
    # detect object in image
    detector = RgbdForegroundMaskQueryImageDetector()
    detections = detector.detect(
        color_im,
        depth_im,
        config["detection"],
        camera_intr=camera_intr,
        vis_foreground=config["vis"]["foreground"],
        vis_segmentation=config["vis"]["segmentation"],
    )

    # take the detection that is most distinct from the background
    bgmodel = color_im.background_model()
    object_detection = None
    max_dist = 0
    for detection in detections:
        bg_dist = np.median(
            np.linalg.norm(detection.color_im.nonzero_data() - bgmodel, axis=1)
        )
        if bg_dist > max_dist:
            object_detection = detection
            max_dist = bg_dist

    return object_detection


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

    # read config
    config_path = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "regrasp.yaml")
    )
    cfg = YamlConfig(config_path)

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

    # init detector
    detector = RgbdForegroundMaskQueryImageDetector()
    workspace_cfg = cfg["detection"]["workspace"]
    rescale_factor = cfg["detection"]["rescale_factor"]
    interpolation = cfg["detection"]["interpolation"]

    # Retrieve camera parameters
    color_intr = device.getColorCameraParams()
    ir_intr = device.getIrCameraParams()
    camera_intr = CameraIntrinsics(
        "kinect2_rgb_camera_frame",
        ir_intr.fx,
        ir_intr.fy,
        ir_intr.cx,
        ir_intr.cx,
        height=512,
        width=424,
    )
    camera_intr_detect = camera_intr.resize(rescale_factor)

    #############################################
    # Kinect frame process loop #################
    #############################################
    while True:

        # Get frames
        frames = listener.waitForNewFrame()
        color_img = frames["color"]
        ir = frames["ir"]
        depth_img = frames["depth"]

        # Process frames
        registration.apply(
            color_img,
            depth_img,
            undistorted,
            registered,
            bigdepth=bigdepth,
            color_depth_map=color_depth_map,
        )

        # Convert to berkley format
        color_im = ColorImage(cv2.cvtColor(color_img.asarray(), cv2.COLOR_RGBA2RGB))
        depth_im = DepthImage(depth_img.asarray())

        # Save original image
        orig_color_im = color_im.copy()
        orig_depth_im = depth_im.copy()

        # inpaint
        color_im_inpainted = color_im.inpaint(
            rescale_factor=cfg["image_proc"]["inpaint_rescale_factor"]
        )
        depth_im_inpainted = depth_im.inpaint(
            rescale_factor=cfg["image_proc"]["inpaint_rescale_factor"]
        )

        # filter
        color_im_filtered = color_im.apply(
            cv2.medianBlur, ksize=cfg["image_proc"]["color_median_win_size"]
        )
        color_im_filtered = color_im_filtered.apply(
            cv2.bilateralFilter,
            d=cfg["image_proc"]["color_bilateral_win_size"],
            sigmaColor=cfg["image_proc"]["color_bilateral_range_sigma"],
            sigmaSpace=cfg["image_proc"]["color_bilateral_spatial_sigma"],
        )
        depth_im = depth_im.to_float()
        depth_im_filtered = depth_im.apply(
            cv2.medianBlur, ksize=cfg["image_proc"]["depth_median_win_size"]
        )
        depth_im_filtered = depth_im_filtered.apply(
            cv2.bilateralFilter,
            d=cfg["image_proc"]["depth_bilateral_win_size"],
            sigmaColor=cfg["image_proc"]["depth_bilateral_range_sigma"],
            sigmaSpace=cfg["image_proc"]["depth_bilateral_spatial_sigma"],
        )
        color_im_detect = color_im.resize(rescale_factor, interp=interpolation)
        depth_im_detect = depth_im.resize(rescale_factor, interp=interpolation)

        if "workspace" in cfg["detection"].keys():
            workspace_cfg = cfg["detection"]["workspace"]
            color_im_detect = color_im_detect.focus(
                rescale_factor * workspace_cfg["crop_dim"][0],
                rescale_factor * workspace_cfg["crop_dim"][1],
                rescale_factor * workspace_cfg["crop_center_px"][0],
                rescale_factor * workspace_cfg["crop_center_px"][1],
            )
            depth_im_detect = depth_im_detect.focus(
                rescale_factor * workspace_cfg["crop_dim"][0],
                rescale_factor * workspace_cfg["crop_dim"][1],
                rescale_factor * workspace_cfg["crop_center_px"][0],
                rescale_factor * workspace_cfg["crop_center_px"][1],
            )

        # detect object in image
        detection = detect_object(color_im_detect, depth_im_detect, camera_intr, cfg)
        if cfg["vis"]["object_detection"]:
            plt.figure()
            plt.subplot(1, 3, 1)
            plt.imshow(detection.color_im.data)
            plt.subplot(1, 3, 2)
            plt.imshow(detection.depth_im.data)
            plt.subplot(1, 3, 3)
            plt.imshow(detection.depth_im_table.data)
            plt.show()

    # Shutdown kinect connection
    device.stop()
    device.close()
    sys.exit(0)
