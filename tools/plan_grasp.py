#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Implementation of the `GQCNN grasp detection algorithm <https://berkeleyautomation.github.io/gqcnn>`_ on the Franka Emika Panda Robots. This file is
based on the grasp_planner_node.py file that was supplied with the GQCNN package.
"""

## Standard library imports ##
import os

## Import custom modules ##
from panda_autograsp import GraspPlanner
from panda_autograsp import Logger

## set root logger format ##
root_logger = Logger.get_logger(log_file=os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), ".." ,"logs/main_log.log"))) # Get root logger and format according to the panda_autograsp.loggers.Logger class

## Create script logger ##
main_logger = Logger.get_logger("plan_grasp.py")

#################################################
## Script settings ##############################
#################################################
MODEL_NAME = "GQCNN-4.0-PJ"

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Create a grasp planner. ##
    grasp_planner = GraspPlanner()
    # main_logger.info("test")

    ## Start grasp planner ##
    grasp_planner.start()
    grasp = grasp_planner.plan_grasp()