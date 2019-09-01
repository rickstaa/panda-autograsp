#!/usr/bin/env python
"""Implementation of the `GQCNN grasp detection algorithm <https://berkeleyautomation.github.io/gqcnn>`_ on the Franka Emika Panda Robots. This file is
based on the grasp_planner_node.py file that was supplied with the GQCNN package.
"""

## Make script both python2 and python3 compatible ##
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
try:
    input = raw_input
except NameError:
    pass

## Import standard library packages ##
import json
import math
import os
import time
import sys

## Imports third party packages ##
import numpy as np

## Import pyros packages ##
from cv_bridge import CvBridge, CvBridgeError
import rospy

## Import BerkeleyAutomation packages ##
from autolab_core import YamlConfig
from perception import (CameraIntrinsics, ColorImage, DepthImage, BinaryImage,
                        RgbdImage)
from visualization import Visualizer2D as vis
from gqcnn.grasping import (Grasp2D, SuctionPoint2D,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState)
from gqcnn.utils import GripperMode, NoValidGraspsException

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from gqcnn.srv import (GQCNNGraspPlanner, GQCNNGraspPlannerBoundingBox,
                       GQCNNGraspPlannerSegmask)
from gqcnn.msg import GQCNNGrasp

## Custom imports ##
from panda_autograsp.functions import download_model
from panda_autograsp import Logger
from panda_autograsp.grasp_planners.gqcnn_ros_grasp_planner import GraspPlanner

#################################################
## Script parameters ############################
#################################################

## Read panda_autograsp configuration file ##
main_cfg = YamlConfig(os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../cfg/main_config.yaml")))

## Get settings out of main_cfg ##
DEFAULT_SOLUTION = main_cfg["defaults"]["solution"]
DEFAULT_MODEL = main_cfg["grasp_detection_solutions"][DEFAULT_SOLUTION]["defaults"]["model"]
DEFAULT_POLICY = main_cfg["grasp_detection_solutions"][DEFAULT_SOLUTION]["defaults"]["policy"]
MODELS_PATH = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", main_cfg["defaults"]["models_dir"]))
DOWNLOAD_SCRIPT_PATH = os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../..", "gqcnn/scripts/downloads/models/download_models.sh"))

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Initialize the ROS node. ##
    rospy.init_node("grasp_planner_server")

    ## Initialize `CvBridge`. ##
    cv_bridge = CvBridge()

    ## Argument parser ##
    try:
        model_name = rospy.get_param("~model_name")
    except KeyError:
        model_name = DEFAULT_MODEL
    try:
        model_dir = rospy.get_param("~model_dir")
        if model_dir == "default":
            model_dir = os.path.abspath(os.path.join(MODELS_PATH, model_name))
    except KeyError:
        model_dir = os.path.abspath(os.path.join(MODELS_PATH, model_name))

    ## Download CNN model if not present ##
    model_dir = os.path.join(MODELS_PATH, model_name)
    if not os.path.exists(model_dir):
        rospy.logwarn("The " + model_name + " model was not found in the models folder. This model is required to continue.")
        while True:
            prompt_result = raw_input(
                "Do you want to download this model now? [Y/n] ")
            # Check user input #
            # If yes download sample
            if prompt_result.lower() in ['y', 'yes']:
                val = download_model(model_name, MODELS_PATH, DOWNLOAD_SCRIPT_PATH)
                if not val == 0: # Check if download was successful
                    shutdown_msg = "Shutting down %s node because grasp model could not downloaded." % (
                    model_name)
                    rospy.logwarn(shutdown_msg)
                    sys.exit(0)
                else:
                    break
            elif prompt_result.lower() in ['n', 'no']:
                shutdown_msg = "Shutting down %s node because grasp model is not downloaded." % (
                model_name)
                rospy.logwarn(shutdown_msg)
                sys.exit(0)
            elif prompt_result == "":
                download_model(model_name, MODELS_PATH, DOWNLOAD_SCRIPT_PATH)
                if not val == 0: # Check if download was successful
                    shutdown_msg = "Shutting down %s node because grasp model could not downloaded." % (
                    model_name)
                    rospy.logwarn(shutdown_msg)
                    sys.exit(0)
                else:
                    break
            else:
                print(
                    prompt_result + " is not a valid response please answer with Y or N to continue.")

    ## Retrieve model related configuration values ##
    model_config = json.load(
        open(os.path.join(model_dir, "config.json"), "r"))
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

    ## Get the policy based config parameters ##
    config_filename = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                   "../..", "gqcnn/cfg/examples/gqcnn_suction.yaml"))
    if (gripper_mode == GripperMode.LEGACY_PARALLEL_JAW
            or gripper_mode == GripperMode.PARALLEL_JAW):
        config_filename = os.path.abspath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "../..",
            "gqcnn/cfg/examples/gqcnn_pj.yaml"))

    ## Get CNN and Policy files ##
    cfg = YamlConfig(config_filename)
    policy_cfg = cfg["policy"]
    policy_cfg["metric"]["gqcnn_model"] = model_dir

    ## Add main Polyicy values to the GQCNN based cfg ##
    # This allows us to add and overwrite to the original GQCNN config file
    cfg.update(main_cfg)

    ## Create publisher to publish pose of final grasp ##
    grasp_pose_publisher = rospy.Publisher("/gqcnn_grasp/pose",
                                           PoseStamped,
                                           queue_size=10)

    ## Create a grasping policy ##
    rospy.loginfo("Creating Grasping Policy")
    grasping_policy = CrossEntropyRobustGraspingPolicy(policy_cfg)

    ## Create a grasp planner ##
    grasp_planner = GraspPlanner(cfg, cv_bridge, grasping_policy,
                                 grasp_pose_publisher)

    ## Initialize the ROS services ##
    grasp_planning_service = rospy.Service("grasp_planner", GQCNNGraspPlanner,
                                           grasp_planner.plan_grasp)
    grasp_planning_service_bb = rospy.Service("grasp_planner_bounding_box",
                                              GQCNNGraspPlannerBoundingBox,
                                              grasp_planner.plan_grasp_bb)
    grasp_planning_service_segmask = rospy.Service(
        "grasp_planner_segmask", GQCNNGraspPlannerSegmask,
        grasp_planner.plan_grasp_segmask)
    rospy.loginfo("Grasping Policy Initialized")

    ## Spin forever. ##
    rospy.spin()
