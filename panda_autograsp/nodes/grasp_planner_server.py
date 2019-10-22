#!/usr/bin/env python
"""This node set up a number of grasp computation services. These services
can be used used to compute a grasp pose out of RBG-D image data. It uses the
:py:class:`GraspPlannerROS` class to do this. The services that are
set up by the ``grasp_planner_server`` node are listed below:

Services:
    - gqcnn_grasp_planner: Computes a grasp pose out of RGB-D images.
    - gqcnn_grasp_planner_bounding_box: Also computes the grasp but allows
    you to supply a bounding box.
    - gqcnn_grasp_planner_segmask: Also computes the grasp but allows you
    to supply a segmask.
"""

# Main python packages
import json
import sys
import os
from gqcnn.utils import GripperMode

from gqcnn.grasping import (
    CrossEntropyRobustGraspingPolicy,
    FullyConvolutionalGraspingPolicyParallelJaw,
    FullyConvolutionalGraspingPolicySuction,
)
from autolab_core import YamlConfig

# ROS python packages
import rospy
from cv_bridge import CvBridge
from gqcnn.srv import GQCNNGraspPlannerBoundingBox, GQCNNGraspPlannerSegmask

# ROS messages and services
from tf2_geometry_msgs import PoseStamped  # Needed because we use tf2

# Panda_autograsp modules, msgs and srvs
from panda_autograsp.functions import download_model
from panda_autograsp.srv import GQCNNGraspPlanner
from panda_autograsp import GraspPlannerROS

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Read panda_autograsp configuration file
    MAIN_CFG = YamlConfig(
        os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)), "../cfg/main_config.yaml"
            )
        )
    )

    # Get settings out of main_cfg
    DEFAULT_SOLUTION = MAIN_CFG["main"]["solution"]
    DEFAULT_MODEL = MAIN_CFG["grasp_detection_solutions"][DEFAULT_SOLUTION]["defaults"][
        "model"
    ]
    MODELS_PATH = os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..",
            MAIN_CFG["main"]["models_dir"],
        )
    )
    DOWNLOAD_SCRIPT_PATH = os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "../..",
            "gqcnn/scripts/downloads/models/download_models.sh",
        )
    )

    # Initialize the ROS node
    rospy.init_node("grasp_planner_server")

    # Initialize `CvBridge`
    cv_bridge = CvBridge()

    # Argument parser
    try:
        model_name = rospy.get_param("~model_name")
    except KeyError:
        model_name = DEFAULT_MODEL
    try:
        model_dir = rospy.get_param("~ymodel_dir")
        if model_dir == "default":
            model_dir = os.path.abspath(os.path.join(MODELS_PATH, model_name))
    except KeyError:
        model_dir = os.path.abspath(os.path.join(MODELS_PATH, model_name))

    # Download CNN model if not present
    model_dir = os.path.join(MODELS_PATH, model_name)
    if not os.path.exists(model_dir):
        rospy.logwarn(
            "The " + model_name + " model was not found in the models"
            " folder. This model is required"
            "to continue."
        )
        while True:
            prompt_result = input("Do you want to download this model now? [Y/n] ")
            # Check user input #
            # If yes download sample
            if prompt_result.lower() in ["y", "yes"]:
                val = download_model(model_name, MODELS_PATH, DOWNLOAD_SCRIPT_PATH)
                if not val == 0:  # Check if download was successful
                    shutdown_msg = (
                        "Shutting down %s node because grasp model could not"
                        "downloaded." % (model_name)
                    )
                    rospy.logwarn(shutdown_msg)
                    sys.exit(0)
                else:
                    break
            elif prompt_result.lower() in ["n", "no"]:
                shutdown_msg = (
                    "Shutting down %s node because grasp model is not downloaded."
                    % (model_name)
                )
                rospy.logwarn(shutdown_msg)
                sys.exit(0)
            elif prompt_result == "":
                download_model(model_name, MODELS_PATH, DOWNLOAD_SCRIPT_PATH)
                if not val == 0:  # Check if download was successful
                    shutdown_msg = (
                        "Shutting down %s node because grasp model could not "
                        "downloaded." % (model_name)
                    )
                    rospy.logwarn(shutdown_msg)
                    sys.exit(0)
                else:
                    break
            else:
                print(
                    prompt_result + " is not a valid response please answer"
                    "with Y or N to continue."
                )

    # Retrieve model related configuration values
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
                "Input data mode {} not supported!".format(input_data_mode)
            )

    # Get the policy based config parameters
    config_filename = os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "../..",
            "gqcnn/cfg/examples/gqcnn_suction.yaml",
        )
    )
    if (
        gripper_mode == GripperMode.LEGACY_PARALLEL_JAW
        or gripper_mode == GripperMode.PARALLEL_JAW
    ):
        config_filename = os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "../..",
                "gqcnn/cfg/examples/gqcnn_pj.yaml",
            )
        )

    # Get CNN and Policy files
    cfg = YamlConfig(config_filename)
    policy_cfg = cfg["policy"]
    policy_cfg["metric"]["gqcnn_model"] = model_dir

    # Add main policy values to the GQCNN based cfg. This allows
    # us to add and overwrite to the original GQCNN config file
    cfg.update(MAIN_CFG)
    cfg["policy"]["gripper_width"] = MAIN_CFG["robot"][
        "gripper_width"
    ]  # Update gripper width

    # Create publisher to publish pose of final grasp
    grasp_pose_publisher = rospy.Publisher(
        "gqcnn_grasp/pose", PoseStamped, queue_size=10
    )

    # Create grasping policy
    rospy.loginfo("Creating Grasping Policy")
    try:
        if (
            "cross_entropy"
            == MAIN_CFG["grasp_detection_solutions"]["gqcnn"]["parameters"][
                "available"
            ][model_name]
        ):
            grasping_policy = CrossEntropyRobustGraspingPolicy(policy_cfg)
        elif (
            "fully_conv"
            == MAIN_CFG["grasp_detection_solutions"]["gqcnn"]["parameters"][
                "available"
            ][model_name]
        ):
            if "pj" in model_name.lower():
                grasping_policy = FullyConvolutionalGraspingPolicyParallelJaw(
                    policy_cfg
                )
            elif "suction" in model_name.lower():
                grasping_policy = FullyConvolutionalGraspingPolicySuction(policy_cfg)
    except KeyError:
        rospy.loginfo(
            "The %s model of the %s policy is not yet implemented."
            % (model_name, "gqcnn")
        )
        sys.exit(0)

    # Create a grasp planner
    grasp_planner = GraspPlannerROS(
        cfg, cv_bridge, grasping_policy, grasp_pose_publisher
    )

    # Initialize the ROS services
    grasp_planning_service = rospy.Service(
        "gqcnn_grasp_planner", GQCNNGraspPlanner, grasp_planner.plan_grasp
    )
    grasp_planning_service_bb = rospy.Service(
        "gqcnn_grasp_planner_bounding_box",
        GQCNNGraspPlannerBoundingBox,
        grasp_planner.plan_grasp_bb,
    )
    grasp_planning_service_segmask = rospy.Service(
        "gqcnn_grasp_planner_segmask",
        GQCNNGraspPlannerSegmask,
        grasp_planner.plan_grasp_segmask,
    )
    rospy.loginfo("Grasping Policy Initialized")

    # Spin forever
    rospy.spin()
