#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" This script can be used to test out the :py:mod:`gqcnn_grasp_planner` python module.

.. note::

    **Usage:**
    To visualize the grasp please set the ``vis`` settings in the
    :file:`../cfg/main_config.yaml`.

Source code
----------------------------
.. literalinclude:: ../../../../panda_autograsp/scripts/plan_grasp.py
   :language: python
   :linenos:
   :lines: 19-
"""

# Main python packages
import os

# Panda_autograsp modules, msgs and srvs
from panda_autograsp.grasp_planners.gqcnn_grasp_planner import GraspPlanner
from panda_autograsp import Logger

# Create script logger
script_logger = Logger.get_logger("plan_grasp.py")

#################################################
# Script settings ###############################
#################################################
MODEL_NAME = "GQCNN-4.0-PJ"

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Welcome message
    print(
        "== Plan grasp script ==\n"
        "This script can be used to test the "
        "'gqcnn_grasp_planner python' module.\n"
        "\n"
        "Usage: To show the computed grasp edit "
        "the vis settings in the `cfg/main_config.yaml` file."
    )

    # set root logger format
    root_logger = Logger.get_logger(
        log_file=os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)), "..", "logs/plan_grasp.log"
            )
        )
    )

    # Create a grasp planner
    grasp_planner = GraspPlanner(model=MODEL_NAME)

    # Plan a grasp and display the result
    grasp_planner.start()
    grasp = grasp_planner.plan_grasp()

    # Print log message
    if grasp:
        script_logger.info(
            "Grasp computed successfully shutting down plan_grasp.py script."
        )
    else:
        script_logger.info(
            "Grasp computation failed shutting down plan_grasp.py script."
        )
