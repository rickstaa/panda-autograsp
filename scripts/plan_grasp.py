#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Implementation of the `GQCNN grasp detection algorithm <https://berkeleyautomation.github.io/gqcnn>`_ on the Franka Emika Panda Robots. This file is
based on the grasp_planner_node.py file that was supplied with the GQCNN package.
"""

## Standard library imports ##
import os
import logging
import json

## GQCNN module imports ##
from gqcnn.grasping import CrossEntropyRobustGraspingPolicy
from gqcnn.utils import GripperMode

## Import custom modules ##
from panda_autograsp import GraspPlanner

## Setup logger ##
main_logger = logging.getLogger("GraspPlanner")

#################################################
## Script settings ##############################
#################################################
MODEL_NAME = "GQCNN-4.0-PJ"
MODEL_DIR = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../gqcnn/models")

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    #############################################
    ## Get configuration values #################
    #############################################

    ## Get model dir path ##
    model_name = MODEL_NAME
    model_dir = os.path.abspath(os.path.join(MODEL_DIR, model_name))

    ## Check if model folder exists and otherwise give a warning ##
    cont_bool = False
    if not os.path.exists(model_dir):
        main_logger.warning("No pretrained CNN model found.")
        prompt_result = input("A pretrained CNN model is required to continue."
                              "These models can be downloaded from the berkeleyautomation/gqcnn repository. "
                              "Do you want to download these models now? [Y/n] ")

        # Check user input #
        if prompt_result.lower() in ['y', 'yes']:  # If yes download sample
            print("YESSSSS!!!")
            cont_bool = True
        elif prompt_result.lower() in ['n', 'no']:
            print("NOOOOOO")
            sys.exit(0)  # Terminate script
        else:
            print("OTHER!!!")
        #     sys.exit(0) # Terminate script

    ## Retrieve model related configuration values ##
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

    ## Get the policy based config parameters ##
    config_filename = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                   "..", "cfg/gqcnn/gqcnn_suction.yaml"))
    if (gripper_mode == GripperMode.LEGACY_PARALLEL_JAW
            or gripper_mode == GripperMode.PARALLEL_JAW):
        config_filename = os.path.abspath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "..",
            "cfg/gqcnn/gqcnn_pj.yaml"))

    ## Read config. ##
    cfg = YamlConfig(config_filename)
    policy_cfg = cfg["policy"]
    policy_cfg["metric"]["gqcnn_model"] = model_dir

    ## Add autograsp config parameters to the config dictionary ##

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

    # # Visualise
    # vis.figure()
    # vis.imshow(grasp.thumbnail)
    # vis.show()
    # print("test")
