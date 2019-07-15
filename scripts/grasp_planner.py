#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Implementation of the `GQCNN grasp detection algorithm <https://github.com/BerkeleyAutomation/gqcnn>`_ on the Franka Emika Panda Robots. This file is
based on the grasp_planner_node.py file that was supplied with the GQCNN package.

Author
-----
Rick Staa
"""

## Standard library imports ##
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import json
import math
import os
import time
import logging
import sys

## Related third party imports ##
import numpy as np
import cv2

## GQCNN module imports ##
from autolab_core import YamlConfig
from perception import (Kinect2Sensor, CameraIntrinsics, ColorImage, DepthImage, BinaryImage,
                        RgbdImage)
from visualization import Visualizer2D as vis
from gqcnn.grasping import (Grasp2D, SuctionPoint2D,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState)
from gqcnn.utils import GripperMode, NoValidGraspsException

## Setup logger ##
main_logger = logging.getLogger("GraspPlanner")

#################################################
## Script settings ##############################
#################################################
MODEL_NAME = "GQCNN-4.0-PJ"
MODEL_DIR = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../gqcnn/models")


#################################################
## GQCNN grasp class ############################
#################################################
## TODO: Update docstring
class GQCNNGrasp(object):
    """Class to store grasps in
    """
    def __init__(self):
        self.pose = object()
        self.q_value = 0.0
        self.grasp_type = 1
        self.center_px = [0, 0]
        self.angle = 0.0
        self.depth = 0.0

    @property
    def PARALLEL_JAW(self):
        """:obj:`PARALLEL_JAW` Specifies parallel jaw gripper type information
        """
        return 0

    @property
    def SUCTION(self):
        """:obj:`SUCTION` Specifies suction jaw gripper type information
        """
        return 1

#################################################
## Grasp planner Class ##########################
#################################################
class GraspPlanner(object):
    def __init__(self, cfg, grasping_policy, sensor_type="kinectv2"):
        """
        Parameters
        ----------
        cfg : dict
            Dictionary of configuration parameters.
        grasping_policy: :obj:`GraspingPolicy`
            Grasping policy to use.
        grasp_pose_publisher: :obj:`Publisher`
            ROS publisher to publish pose of planned grasp for visualization.
        """
        self.cfg = cfg
        self.grasping_policy = grasping_policy
        self.sensor_type = sensor_type
        self.gqcnn_model = grasping_policy.grasp_quality_fn.config['gqcnn_model'].split(
            "/")[-1]
        self.gripper_mode = self.grasping_policy._grasp_quality_fn._gqcnn.gripper_mode

        ## Initiate sensor ##
        if self.sensor_type == "kinectv2":
            self.sensor = Kinect2Sensor()
        else:
            main_logger.error("Unfortunately the " +
                              self.sensor_type+" camera is not yet supported.")
            sys.exit(0)  # Exit script

        ## Set minimum input dimensions. ##
        self.pad = max(
            math.ceil(
                np.sqrt(2) *
                (float(self.cfg["policy"]["metric"]["crop_width"]) / 2)),
            math.ceil(
                np.sqrt(2) *
                (float(self.cfg["policy"]["metric"]["crop_height"]) / 2)))
        self.min_width = 2 * self.pad + self.cfg["policy"]["metric"][
            "crop_width"]
        self.min_height = 2 * self.pad + self.cfg["policy"]["metric"][
            "crop_height"]

    def start(self):
        """Starts grasp planning
        """
        start_msg = "Starting " + self.gqcnn_model + \
            " grasp planner for a " + self.gripper_mode + " robot."
        main_logger.info(start_msg)
        self._start_sensor()

    def _start_sensor(self):
        """Start sensor
        """
        self.sensor.start()

        ## Get intrinsic camera parameters ##
        self.camera_intr = self.sensor.ir_intrinsics
        self.camera_color_intr = self.sensor.color_intrinsics

    def read_images(self, skip_registration=False):
        """Retrieves images from the sensor

        Parameters
        ----------
        skip_registration : bool
            If True, the registration step is skipped.

        Returns
        -------
        :obj:`tuple` of :obj:`ColorImage`, :obj:`DepthImage`, :obj:`IrImage`, :obj:`numpy.ndarray`
            The ColorImage, DepthImage, and IrImage of the current frame.

        Raises
        ------
        RuntimeError
            If the Kinect stream is not running.
        """
        ## Get color, depth and ir image frames ##
        if self.sensor_type == "kinectv2":
            color_im, depth_im, ir_im = self.sensor.frames()

        ## Validate whether depth and color images are the same size ##
        if color_im.height != depth_im.height or \
           color_im.width != depth_im.width:
            msg = ("Color image and depth image must be the same shape! Color"
                   " is %d x %d but depth is %d x %d") % (
                       color_im.height, color_im.width, depth_im.height,
                       depth_im.width)
            main_logger.warning(msg)
        if (color_im.height < self.min_height
                or color_im.width < self.min_width):
            msg = ("Color image is too small! Must be at least %d x %d"
                   " resolution but the requested image is only %d x %d") % (
                       self.min_height, self.min_width, color_im.height,
                       color_im.width)
            main_logger.warning(msg)

        ## Return color and depth frames
        return color_im, depth_im, ir_im

    # TODO: Update Docstring
    def plan_grasp(self):
        """Grasp planner request handler.

        Returns
        -------
        [type]
            [description]
        """

        color_im, depth_im, _ = self.read_images()
        return self._plan_grasp(color_im, depth_im, self.camera_intr)

    # TODO: Add bounding box functionality
    # TODO: Update Docstring
    def plan_grasp_bb(self, bounding_box):
        """Grasp planner request handler.

        Parameters
        ----------
        bounding_box : [type], optional
            [description], by default None

        Returns
        -------
        [type]
            [description]
        """
        color_im, depth_im, _ = self.read_images()
        return self._plan_grasp(color_im,
                                depth_im,
                                self.camera_intr,
                                bounding_box=bounding_box)

    # TODO: Update Docstring
    def plan_grasp_segmask(self, segmask):
        """Grasp planner request handler.

        Parameters
        ---------
        req: :obj:`ROS ServiceRequest`
            ROS `ServiceRequest` for grasp planner service.
        """
        color_im, depth_im, _ = self.read_images()
        raw_segmask = segmask
        try:
            segmask = BinaryImage(raw_segmask,
                                  frame=self.camera_intr.frame)
        except Exception as plan_grasp_segmask_exception:
            main_logger.warning(plan_grasp_segmask_exception)

        ## Validate whether image and segmask are the same size ##
        if color_im.height != segmask.height or \
           color_im.width != segmask.width:
            msg = ("Images and segmask must be the same shape! Color image is"
                   " %d x %d but segmask is %d x %d") % (
                       color_im.height, color_im.width, segmask.height,
                       segmask.width)
            main_logger.warning(msg)

        return self._plan_grasp(color_im,
                                depth_im,
                                self.camera_intr,
                                segmask=segmask)

    # TODO: Update Docstring
    # TODO: Check if threshold is good enough
    def _plan_grasp(self,
                    color_im,
                    depth_im,
                    camera_intr,
                    bounding_box=None,
                    segmask=None):
        """Grasp planner request handler.

        Parameters
        ---------
        req: :obj:`ROS ServiceRequest`
            ROS `ServiceRequest` for grasp planner service.
        """
        main_logger.info("Planning Grasp")

        # Inpaint images.
        color_im = color_im.inpaint(
            rescale_factor=self.cfg["inpaint_rescale_factor"])
        depth_im = depth_im.inpaint(
            rescale_factor=self.cfg["inpaint_rescale_factor"])

        # Init segmask.
        if segmask is None:
            segmask = BinaryImage(255 *
                                  np.ones(depth_im.shape).astype(np.uint8),
                                  frame=color_im.frame)

        # Visualize.
        if self.cfg["vis"]["color_image"]:
            vis.imshow(color_im)
            vis.title("Color image")
            vis.show()
        if self.cfg["vis"]["depth_image"]:
            vis.imshow(depth_im)
            vis.title("Depth image")
            vis.show()
        if self.cfg["vis"]["segmask"] and segmask is not None:
            vis.imshow(segmask)
            vis.title("Segmask image")
            vis.show()

        # Aggregate color and depth images into a single
        # BerkeleyAutomation/perception `RgbdImage`.
        rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)

        # Mask bounding box.
        if bounding_box is not None:
            # Calc bb parameters.
            min_x = bounding_box.minX
            min_y = bounding_box.minY
            max_x = bounding_box.maxX
            max_y = bounding_box.maxY

            # Contain box to image->don't let it exceed image height/width
            # bounds.
            if min_x < 0:
                min_x = 0
            if min_y < 0:
                min_y = 0
            if max_x > rgbd_im.width:
                max_x = rgbd_im.width
            if max_y > rgbd_im.height:
                max_y = rgbd_im.height

            # Mask.
            bb_segmask_arr = np.zeros([rgbd_im.height, rgbd_im.width])
            bb_segmask_arr[min_y:max_y, min_x:max_x] = 255
            bb_segmask = BinaryImage(bb_segmask_arr.astype(np.uint8),
                                     segmask.frame)
            segmask = segmask.mask_binary(bb_segmask)

        # Visualize.
        if self.cfg["vis"]["rgbd_state"]:
            masked_rgbd_im = rgbd_im.mask_binary(segmask)
            vis.figure()
            vis.title("Masked RGBD state")
            vis.subplot(1, 2, 1)
            vis.imshow(masked_rgbd_im.color)
            vis.subplot(1, 2, 2)
            vis.imshow(masked_rgbd_im.depth)
            vis.show()

        # Create an `RgbdImageState` with the cropped `RgbdImage` and
        # `CameraIntrinsics`.
        rgbd_state = RgbdImageState(rgbd_im, self.camera_intr, segmask=segmask)

        # Execute policy.
        try:
            return self.execute_policy(rgbd_state, self.grasping_policy,
                                       self.camera_intr.frame)
        except NoValidGraspsException:
            main_logger.error(
                ("While executing policy found no valid grasps from sampled"
                 " antipodal point pairs. Aborting Policy!"))

    def execute_policy(self, rgbd_image_state, grasping_policy, pose_frame):
        """Executes a grasping policy on an `RgbdImageState`.

        Parameters
        ----------
        rgbd_image_state: :obj:`RgbdImageState`
            `RgbdImageState` from BerkeleyAutomation/perception to encapsulate
            depth and color image along with camera intrinsics.
        grasping_policy: :obj:`GraspingPolicy`
            Grasping policy to use.
        pose_frame: :obj:`str`
            Frame of reference to publish pose in.
        """
        # Execute the policy"s action.
        grasp_planning_start_time = time.time()
        grasp = grasping_policy(rgbd_image_state)

        # Create `GQCNNGrasp` return msg and populate it.
        gqcnn_grasp = GQCNNGrasp()
        gqcnn_grasp.q_value = grasp.q_value
        gqcnn_grasp.pose = grasp.grasp.pose()
        if isinstance(grasp.grasp, Grasp2D):
            gqcnn_grasp.grasp_type = GQCNNGrasp.PARALLEL_JAW
        elif isinstance(grasp.grasp, SuctionPoint2D):
            gqcnn_grasp.grasp_type = GQCNNGrasp.SUCTION
        else:
            main_logger.error("Grasp type not supported!")

        # Store grasp representation in image space.
        gqcnn_grasp.center_px[0] = grasp.grasp.center[0]
        gqcnn_grasp.center_px[1] = grasp.grasp.center[1]
        gqcnn_grasp.angle = grasp.grasp.angle
        gqcnn_grasp.depth = grasp.grasp.depth
        # gqcnn_grasp.thumbnail = grasp.image.rosmsg

        # # Create and publish the pose alone for easy visualization of grasp
        # # pose in Rviz.
        # pose_stamped = PoseStamped()
        # pose_stamped.pose = grasp.grasp.pose().pose_msg
        # header = Header()
        # header.stamp = rospy.Time.now()
        # header.frame_id = pose_frame
        # pose_stamped.header = header

        # Return `GQCNNGrasp` msg.
        main_logger.info("Total grasp planning time: " +
                         str(time.time() - grasp_planning_start_time) + " secs.")
        return gqcnn_grasp


#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    #############################################
    ## Get configuration values #################
    #############################################

    ## Get configs. ##
    model_name = MODEL_NAME
    model_dir = os.path.abspath(os.path.join(MODEL_DIR, model_name))
    model_config = json.load(open(os.path.join(model_dir, "config.json"), "r"))
    try:
        gqcnn_config = model_config["gqcnn"]
        gripper_mode = gqcnn_config["gripper_mode"]
    except KeyError:
        gqcnn_config = model_config["gqcnn_config"]
        input_data_mode = gqcnn_config["input_data_mode"]
        if input_data_mode == "tf_image":
            gripper_mode = GripperMode.LEGACY_PARALLEL_JAW
        elif input_data_mode == "tf_image_suction":
            gripper_mode = GripperMode.LEGACY_SUCTION
        elif input_data_mode == "suction":
            gripper_mode = GripperMode.SUCTION
        elif input_data_mode == "multi_suction":
            gripper_mode = GripperMode.MULTI_SUCTION
        elif input_data_mode == "parallel_jaw":
            gripper_mode = GripperMode.PARALLEL_JAW
        else:
            raise ValueError(
                "Input data mode {} not supported!".format(input_data_mode))

    ## TODO: Update cfg file location
    ## NOTE WHY DO I NEED THIS?
    # Set config.
    config_filename = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                   "..", "gqcnn/cfg/examples/ros/gqcnn_suction.yaml"))
    if (gripper_mode == GripperMode.LEGACY_PARALLEL_JAW
            or gripper_mode == GripperMode.PARALLEL_JAW):
        config_filename = os.path.abspath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "..",
            "gqcnn/cfg/examples/ros/gqcnn_pj.yaml"))

    # Read config.
    cfg = YamlConfig(config_filename)
    policy_cfg = cfg["policy"]
    policy_cfg["metric"]["gqcnn_model"] = model_dir

    #############################################
    ## Create grasp policy ######################
    #############################################
    main_logger.info("Creating Grasping Policy")
    grasping_policy = CrossEntropyRobustGraspingPolicy(policy_cfg)

    ## Create a grasp planner. ##
    grasp_planner = GraspPlanner(cfg, grasping_policy)

    ## Start grasp planner ##
    grasp_planner.start()
    grasp = grasp_planner.plan_grasp()
    print("Started")

#################################################
## Visualize ####################################
#################################################
    while True:

        ## Get raw color iamge ##

        # ## Get full color image ##
        # ## Plot i
        # color_im, depth_im, _ = grasp_planner.read_images()
        # cv2.imshow("color", color_im.data)
        # cv2.imshow("depth", depth_im.data)
        # key = cv2.waitKey(delay=1)
        if key == ord('q'):
            break
