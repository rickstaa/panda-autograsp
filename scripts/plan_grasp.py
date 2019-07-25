#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Implementation of the `GQCNN grasp detection algorithm <https://berkeleyautomation.github.io/gqcnn>`_ on the Franka Emika Panda Robots. This file is
based on the grasp_planner_node.py file that was supplied with the GQCNN package.
"""

## Standard library imports ##
import os
import json
import logging
import sys

## Setup logger ##
# from panda_autograsp import Logger

## Import custom modules ##
from panda_autograsp import GraspPlanner

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

    ## Start grasp planner ##
    grasp_planner.start()
    grasp = grasp_planner.plan_grasp()