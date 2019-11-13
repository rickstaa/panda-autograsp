"""Module that sets up a number of services that enable
the :py:mod:`panda_autograsp_cli` command line interface (CLI) to
communicate with the components of the `panda_autograps` solution.
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
import sys
import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import os
import copy
from pyquaternion import Quaternion
import pickle
from autolab_core import YamlConfig

# ROS python packages
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import tf2_ros

# ROS messages and services
import sensor_msgs
from tf2_geometry_msgs import PoseStamped  # Needed because we use tf2
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from std_srvs.srv import Empty

# Panda_autograsp modules, msgs and srvs
from panda_autograsp.srv import (
    ComputeGrasp,
    PlanGrasp,
    PlanToPoint,
    VisualizePlan,
    VisualizeGrasp,
    ExecutePlan,
    ExecuteGrasp,
    CalibrateSensor,
    SetSensorPose,
    SetGripperWidth,
    SetGripperOpen,
    SetGripperClosed,
    PlanPlace,
    PlanToPath,
    GQCNNGraspPlanner,
)

# Panda_autograsp modules, msgs and srvs
from panda_autograsp.functions import draw_axis

# Set right matplotlib backend
# Needed in order to show images inside imported modules
plt.switch_backend("TkAgg")

#################################################
# Read main config ##############################
#################################################

# Open panda_autograsp configuration file
MAIN_CFG = YamlConfig(
    os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "../../cfg/main_config.yaml"
        )
    )
)

# Get important parameters
POSE_CALIB_METHOD = MAIN_CFG["calibration"]["pose_estimation_calib_board"]

#################################################
# Script settings ###############################
#################################################
CALIB_TRY_DURATION = MAIN_CFG["calibration"]["calib_try_duration"]  # [s]

###############################
# Chessboard settings #########
###############################

# Board parameters
calib_config = MAIN_CFG["calibration"]
N_FRAMES = calib_config["chessboard_settings"]["N_FRAMES"]
N_ROWS = calib_config["chessboard_settings"]["N_ROWS"]
N_COLMNS = calib_config["chessboard_settings"]["N_COLUMNS"]
SQUARE_SIZE = calib_config["chessboard_settings"]["SQUARE_SIZE"]

#################################################
# Aruco settings ################################
#################################################

# Initialize board parameters
load_dir_path = os.path.abspath(
    os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "../../cfg/_cfg/aruco_config.dict"
    )
)
with open(load_dir_path, "rb") as config_dict_file:
    ARUCO_CFG = pickle.load(config_dict_file)  # Load the aruco board settings

# Overwrite settings based on measurements
ARUCO_CFG["MARKER_LENGTH"] = 0.032  # [M]
ARUCO_CFG["MARKER_SEPERATION"] = 0.009  # [M]
ARUCO_DICT = aruco.Dictionary_get(ARUCO_CFG["ARUCO_DICT_TYPE"])
aruco_board = aruco.GridBoard_create(
    markersX=ARUCO_CFG["MARKERS_X"],
    markersY=ARUCO_CFG["MARKERS_Y"],
    markerLength=ARUCO_CFG["MARKER_LENGTH"],
    markerSeparation=ARUCO_CFG["MARKER_SEPARATION"],
    dictionary=ARUCO_DICT,
)


#################################################
# GraspPlannerClient Class ######################
#################################################
class PandaAutograspServer:
    """Class used to create and call all the panda_autograsp components.

    Attributes
    -------
    rvec : :py:obj:`list`
        The rotation vector of the camera/world calibration.
    tvec : :py:obj:`list`
        The translation vector of the camera/world calibration.
    """

    def __init__(self):

        # Setup cv_bridge
        self._cv_bridge = CvBridge()

        # Setup member variables
        self.rvec = None
        self.tvec = None

        # Setup opencv termination criteria
        self._criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        ###############################################
        # Initialize grasp computation services #######
        ###############################################

        rospy.loginfo("Connecting to 'gqcnn_grasp_planner' service...")
        rospy.wait_for_service("gqcnn_grasp_planner")
        try:
            self._gqcnn_grasp_planning_srv = rospy.ServiceProxy(
                "gqcnn_grasp_planner", GQCNNGraspPlanner
            )
            rospy.loginfo("Connected to `gqcnn_grasp_planner service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'gqcnn_grasp_planner' service initialization "
                "failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._gqcnn_grasp_planning_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        ###############################################
        # Initialize moveit_planner server services ###
        ###############################################

        # Initialize Plan to point service
        rospy.loginfo("Connecting to moveit_planner services.")
        rospy.logdebug("Connecting to 'moveit_planner_server/plan_to_point' service...")
        rospy.wait_for_service("moveit_planner_server/plan_to_point")
        try:
            self._plan_to_pose_srv = rospy.ServiceProxy(
                "moveit_planner_server/plan_to_point", PlanToPoint
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/plan_to_point' " "service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/plan_to_point' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._plan_to_pose_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize random cartesian path service
        rospy.logdebug("Connecting to 'moveit_planner_server/plan_to_path' service...")
        rospy.wait_for_service("moveit_planner_server/plan_to_path")
        try:
            self._plan_to_path_srv = rospy.ServiceProxy(
                "moveit_planner_server/plan_to_path", PlanToPath
            )
            rospy.logdebug("Connected to 'moveit_planner_server/plan_to_path' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "moveit_planner_server 'plan_to_path' service initialization failed: %s"
                % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._plan_to_path_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize plan visualization service
        rospy.logdebug(
            "Connecting to 'moveit_planner_server/visualize_plan' " "service..."
        )
        rospy.wait_for_service("moveit_planner_server/visualize_plan")
        try:
            self._visualize_plan_srv = rospy.ServiceProxy(
                "moveit_planner_server/visualize_plan", VisualizePlan
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/visualize_plan' " "service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/visualize_plan' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._visualize_plan_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize execute plan service
        rospy.logdebug("Connecting to 'moveit_planner_server/execute_plan' service...")
        rospy.wait_for_service("moveit_planner_server/execute_plan")
        try:
            self._execute_plan_srv = rospy.ServiceProxy(
                "moveit_planner_server/execute_plan", ExecutePlan
            )
            rospy.logdebug("Connected to 'moveit_planner_server/execute_plan' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/execute_plan' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._execute_plan_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize send sensor pose service
        rospy.logdebug("Connecting to 'tf2_broadcaster/send_sensor_pose' service...")
        rospy.wait_for_service("tf2_broadcaster/set_sensor_pose")
        try:
            self._set_sensor_pose_srv = rospy.ServiceProxy(
                "tf2_broadcaster/set_sensor_pose", SetSensorPose
            )
            rospy.logdebug("Connected to 'tf2_broadcaster/set_sensor_pose' service.")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'tf2_broadcaster/set_sensor_pose' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._set_sensor_pose_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize set gripper_width service
        rospy.logdebug(
            "Connecting to 'moveit_planner_server/set_gripper_width' " "service..."
        )
        rospy.wait_for_service("moveit_planner_server/set_gripper_width")
        try:
            self._set_gripper_width_srv = rospy.ServiceProxy(
                "moveit_planner_server/set_gripper_width", SetGripperWidth
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/set_gripper_width' " "service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/set_gripper_width' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._set_gripper_width_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize set gripper_width service
        rospy.logdebug(
            "Connecting to 'moveit_planner_server/set_gripper_open' " "service..."
        )
        rospy.wait_for_service("moveit_planner_server/set_gripper_open")
        try:
            self._set_gripper_open_srv = rospy.ServiceProxy(
                "moveit_planner_server/set_gripper_open", SetGripperOpen
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/set_gripper_open' " "service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/set_gripper_open' service "
                "initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._set_gripper_open_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Initialize set gripper_width service
        rospy.logdebug(
            "Connecting to 'moveit_planner_server/set_gripper_closed' " "service..."
        )
        rospy.wait_for_service("moveit_planner_server/set_gripper_closed")
        try:
            self._set_gripper_closed_srv = rospy.ServiceProxy(
                "moveit_planner_server/set_gripper_closed", SetGripperClosed
            )
            rospy.logdebug(
                "Connected to 'moveit_planner_server/set_gripper_width' " "service."
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Panda_autograsp 'moveit_planner_server/set_gripper_closed' "
                "service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._set_gripper_closed_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Print moveit_planner_server services connection success message
        rospy.loginfo("Successfully connected to all moveit_planner_server services.")

        ###############################################
        # Create panda_autograsp_server services ######
        ###############################################

        # Calibrate sensor
        rospy.loginfo("Initializing %s services.", rospy.get_name())
        rospy.logdebug("Initializing 'calibrate_sensor' service...")
        self._calibrate_sensor_srv = rospy.Service(
            "calibrate_sensor", CalibrateSensor, self.calibrate_sensor_service
        )

        # Compute grasp service
        rospy.logdebug("Initializing 'compute_grasp' service...")
        self._compute_grasp_srv = rospy.Service(
            "compute_grasp", ComputeGrasp, self.compute_grasp_service
        )

        # Plan grasp service
        rospy.logdebug("Initializing 'plan_grasp' service...")
        self._plan_grasp_srv = rospy.Service(
            "plan_grasp", PlanGrasp, self.plan_grasp_service
        )

        # Visualize grasp service
        rospy.logdebug("Initializing 'visualize_grasp' service...")
        self._visualize_grasp_srv = rospy.Service(
            "visualize_grasp", VisualizeGrasp, self.visualize_grasp_service
        )

        # Execute grasp service
        rospy.logdebug("Initializing 'execute_grasp' service...")
        self._execute_grasp_srv = rospy.Service(
            "execute_grasp", ExecuteGrasp, self.execute_grasp_service
        )

        # Object place planning service
        rospy.logdebug("Initializing 'plan_place' service...")
        self.plan_place_srv = rospy.Service(
            "plan_place", PlanPlace, self.plan_place_service
        )

        # Service initiation success message
        rospy.loginfo(
            "'%s' services initialized successfully. Waiting for requests.",
            rospy.get_name(),
        )

        ###############################################
        # Create subscribers and publishers ###########
        ###############################################

        # Create msg filter subscribers
        rospy.logdebug("Creating camera sensor message_filter...")
        self._color_image_sub = Subscriber("image_color", sensor_msgs.msg.Image)
        self._color_image_rect_sub = Subscriber(
            "image_color_rect", sensor_msgs.msg.Image
        )
        self._depth_image_rect_sub = Subscriber(
            "image_depth_rect_32FC1", sensor_msgs.msg.Image
        )
        self._camera_info_hd_sub = Subscriber(
            "hd/camera_info", sensor_msgs.msg.CameraInfo
        )
        self._camera_info_qhd_sub = Subscriber(
            "qhd/camera_info", sensor_msgs.msg.CameraInfo
        )
        self._camera_info_sd_sub = Subscriber(
            "sd/camera_info", sensor_msgs.msg.CameraInfo
        )

        # Wait till camera is online
        rospy.loginfo("Waiting for first camera message...")
        rospy.wait_for_message("image_color", sensor_msgs.msg.Image)
        rospy.loginfo("Camera is online and publishing messages.")

        # Create msg filter
        self._ats = ApproximateTimeSynchronizer(
            [
                self._color_image_sub,
                self._color_image_rect_sub,
                self._depth_image_rect_sub,
                self._camera_info_hd_sub,
                self._camera_info_qhd_sub,
                self._camera_info_sd_sub,
            ],
            queue_size=5,
            slop=0.1,
        )
        self._ats.registerCallback(self.msg_filter_callback)
        rospy.logdebug("Camera sensor message_filter created.")

        # Create state listener
        self._tf2_buffer = tf2_ros.Buffer()
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer)

        # Create publisher to publish the place pose
        rospy.logdebug("Creating place pose publisher...")
        self._place_pose_pub = rospy.Publisher(
            "moveit/place_pose", PoseStamped, queue_size=10
        )
        rospy.logdebug("Place pose publisher created.")

        # Create pose subscriber
        rospy.logdebug("Creating grasp pose subscriber...")
        self._pose_sub = rospy.Subscriber(
            "gqcnn_grasp/pose", PoseStamped, self.get_pose_callback
        )
        rospy.logdebug("Grasp pose subscriber created.")

    def get_pose_callback(self, pose_msg):
        """Callback function of the 'gqcnn_graps/pose' subscriber. This function updates the
        self.pose_msg member variable.

        Parameters
        ----------
        pose_msg : :py:obj:`!geometry_msgs.PosedStamed`
            Grasp pose msgs.
        """

        # Update pose_msg
        rospy.loginfo("Received grasp pose.")
        self.pose_msg = pose_msg

    def msg_filter_callback(
        self,
        color_image,
        color_image_rect,
        depth_image_rect,
        camera_info_hd,
        camera_info_qhd,
        camera_info_sd,
    ):
        """Callback function of the message filter. This message filter subscribed
        to a number of camera topics which are required by the panda_autograsp
        solution.
        Parameters
        ----------
        color_image : :py:obj:`!sensor_msgs.msg.Image`
            The color image.
        color_image_rect : :py:obj:`!sensor_msgs.msg.Image`
            The rectified color image.
        depth_image_rect : :py:obj:`!sensor_msgs.msg.Image`
            The depth image.
        camera_info_hd :  :py:obj:`!sensor_msgs.msg.CameraInfo`
            The HD camera info topic.
        camera_info_qhd :  :py:obj:`!sensor_msgs.msg.CameraInfo`
            The QHD camera topic.
        camera_info_sd :  :py:obj:`!sensor_msgs.msg.CameraInfo`
            The SD camera topic.
        """

        # Call the grasp_planner_service
        self.color_image = color_image
        self.color_image_rect = color_image_rect
        self.depth_image_rect = depth_image_rect
        self.camera_info_hd = camera_info_hd
        self.camera_info_qhd = camera_info_qhd
        self.camera_info_sd = camera_info_sd

    def compute_grasp_service(self, req):
        """This service is used for computing a vallid grasp out of the
        sensor data. This is done by calling the main grasp computation service
        of the :py:obj:`grasp_planner_server.py` module with the sensor data as
        its input.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.ComputeGrasp`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Call grasp computation service
        self.grasp = self._gqcnn_grasp_planning_srv(
            self.color_image_rect, self.depth_image_rect, self.camera_info_sd
        )

        # Print grasp
        position = self.grasp.grasp.pose.position
        orientation = self.grasp.grasp.pose.orientation
        pose_array = [
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        rospy.logdebug(
            "Grasp pose result in kinect2_rgb_camera_frame: x={0}, y={1}, z={2}, "
            "q1={3}, q2={4}, q3={5} and q4={6}".format(*pose_array)
        )

        # Test if successful
        if self.grasp:
            return True
        else:
            return False

    def plan_grasp_service(self, req):
        """This service can be used to plan for the by the
        :py:meth:`compute_grasp_service` computed grasp pose.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanGrasp`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Get pose expressed in the panda_link0 frame
        # Needed since the panda_link0 is the reference frame
        # of the move group.
        try:
            grasp_pose_msg = copy.copy(self.pose_msg)  # Create copy
            grasp_pose_msg.header.stamp = (
                rospy.Time.now()
            )  # As we use the default tf buffer we will set the time to be now
            pose_msg = self._tf2_buffer.transform(
                grasp_pose_msg, "panda_link0", rospy.Duration(1)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return False

        # Display pose in panda_link0 frame
        position = pose_msg.pose.position
        orientation = pose_msg.pose.orientation
        pose_array = [
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        rospy.logdebug(
            "Grasp pose result in panda_link0: x={0}, y={1}, z={2}, q1={3}, q2={4}, "
            "q3={5} and q4={6}".format(*pose_array)
        )

        # Call grasp plan to pose service
        result = self._plan_to_pose_srv(pose_msg.pose)

        # Test if successful
        if result.success:
            return True
        else:
            return False

    def plan_place_service(self, req):
        """This service computes the movement plan for placing the
        object at the desired goal position. The place goal position
        can be changed by modifying the ``cfg/main_config.yaml``
        document.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanPlace`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Retrieve the starting grasp pose (as expressed in the panda_link0 frame
        try:
            grasp_pose_msg = copy.copy(self.pose_msg)  # Create copy
            grasp_pose_msg.header.stamp = (
                rospy.Time.now()
            )  # As we use the default tf buffer we will set the time to be now
            grasp_pose_msg = self._tf2_buffer.transform(
                self.pose_msg, "panda_link0", rospy.Duration(1)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return False

        # Generate place pose relative to the panda_link0 frame
        # The location of this frame can be set in the `main_config.yaml` file
        # in the cfg folder.
        self.place_pose_msg = PoseStamped()
        self.place_pose_msg.pose.position.x = MAIN_CFG["main"]["place"]["pose"]["x_pos"]
        self.place_pose_msg.pose.position.y = MAIN_CFG["main"]["place"]["pose"]["y_pos"]
        self.place_pose_msg.pose.position.z = (
            grasp_pose_msg.pose.position.z
        )  # Set equal to start grasp height
        self.place_pose_msg.pose.orientation.x = (
            grasp_pose_msg.pose.orientation.x
        )  # Set equal to grasp pose orientation
        self.place_pose_msg.pose.orientation.y = (
            grasp_pose_msg.pose.orientation.y
        )  # Set equal to grasp pose orientation
        self.place_pose_msg.pose.orientation.z = (
            grasp_pose_msg.pose.orientation.z
        )  # Set equal to grasp pose orientation
        self.place_pose_msg.pose.orientation.w = (
            grasp_pose_msg.pose.orientation.w
        )  # Set equal to grasp pose orientation
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "panda_link0"
        self.place_pose_msg.header = header

        # Publish place pose
        self._place_pose_pub.publish(self.place_pose_msg)

        # Create a intermediate pose
        # Done to pick up the object instead of sliding it.
        self.pickup_pose_msg = PoseStamped()
        self.pickup_pose_msg.pose.position.x = grasp_pose_msg.pose.position.x
        self.pickup_pose_msg.pose.position.y = grasp_pose_msg.pose.position.y
        self.pickup_pose_msg.pose.position.z = (
            grasp_pose_msg.pose.position.z + MAIN_CFG["main"]["pickup"]["height"]
        )  # Add pickup height
        self.pickup_pose_msg.pose.orientation.x = (
            grasp_pose_msg.pose.orientation.x
        )  # Set equal to grasp pose orientation
        self.pickup_pose_msg.pose.orientation.y = (
            grasp_pose_msg.pose.orientation.y
        )  # Set equal to grasp pose orientation
        self.pickup_pose_msg.pose.orientation.z = (
            grasp_pose_msg.pose.orientation.z
        )  # Set equal to grasp pose orientation
        self.pickup_pose_msg.pose.orientation.w = (
            grasp_pose_msg.pose.orientation.w
        )  # Set equal to grasp pose orientation

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "panda_link0"
        self.pickup_pose_msg.header = header

        # Call grasp plan to pose service
        result = self._plan_to_path_srv(
            [self.pickup_pose_msg.pose, self.place_pose_msg.pose]
        )

        # Test if successful
        if result.success:
            return True
        else:
            return False

    def visualize_grasp_service(self, req):
        """This service can be used to visualize the planned grasp.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.VizualizeGrasp`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """
        # Call grasp computation service
        result = self._visualize_plan_srv()

        # Test if successful
        if result.success:
            return True
        else:
            return False

    def execute_grasp_service(self, req):
        """This service is used to execute the computed grasp.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.ExecuteGrasp`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Call grasp computation service
        result = self._execute_plan_srv()

        # Test if successful
        if result.success:
            return True
        else:
            return False

    def calibrate_sensor_service(self, req):
        """This service can be used to perform the sensor/world
        calibration. To do this you need to place a chessboard/aruco
        board in the bottom left corner of the robot table. You have
        to define which calibration pattern you use in the ``cfg/main_config.yaml
        file``. After a successful camera/world transformation matrix is computed
        this matrix is send to the :py:mod:`tf2_broadcaster` module and the
        sensor frame position is updated.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.ExecuteGrasp`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Retrieve camera pose
        retval, self.rvec, self.tvec = self._camera_world_calibration(
            calib_type=POSE_CALIB_METHOD
        )

        # Test if successful
        if retval:

            # Publish the camera frame
            self.broadcast_camera_frame(calib_type=POSE_CALIB_METHOD)

            # return result
            return True
        else:
            return False

    def _camera_world_calibration(self, calib_type=POSE_CALIB_METHOD):
        """Perform camera world calibration (External camera matrix) using
        a chessboard or several aruco markers.

        Parameters
        ----------
        calib_type : :py:obj:`str`
            Calibration pattern type.

        Returns
        -------
        retval : :py:obj:`bool`
            Calibration succes bool.
        :py:obj:
        rvec : :py:obj:`list`
            Rotation vector.
        tvec : :py:obj:`list`
            Translation vector.
        """

        # Switch between different calibrations
        if calib_type == "chessboard":
            return (
                self.chessboard_pose_estimation()
            )  # Perform calibration using an chessboard
        else:
            return (
                self.aruco_board_pose_estimation()
            )  # Perform calibration using an arucoboard

    def aruco_board_pose_estimation(self):
        """Function that performs the camera/world calibration by using
        a Aruco board as a reference.

        Returns
        -------
        retval : :py:obj:`bool`
            Calibration success bool.
        rvec : :py:obj:`list`
            Rotation vector.
        tvec : :py:obj:`list`
            Translation vector.
        """

        # Get current time
        start_time = rospy.get_time()

        # Try till chessboard is found or till try time is over
        while rospy.get_time() < start_time + CALIB_TRY_DURATION:

            # Retrieve color image and convert to opencv format
            color_image = self.color_image
            camera_info = self.camera_info_hd
            color_image_cv = self._cv_bridge.imgmsg_to_cv2(
                color_image, desired_encoding="passthrough"
            )

            # Get camera information
            camera_matrix = np.array(camera_info.K).reshape(3, 3)
            dist_coeffs = camera_info.D  # Default distortion parameters are 0

            # Get gray image
            gray = cv2.cvtColor(color_image_cv, cv2.COLOR_BGR2GRAY)

            # Create screen display image
            # Needed since opencv uses BGR instead of RGB
            screen_img = cv2.cvtColor(copy.copy(color_image_cv), cv2.COLOR_RGB2BGR)

            # Detect aruco markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(
                image=gray,
                dictionary=ARUCO_DICT,
                parameters=aruco.DetectorParameters_create(),
                # cameraMatrix=camera_matrix,
                # distCoeffs=dist_coeffs,
            )

            # Refine detected markers
            # Eliminates markers not part of our board and adds missing markers
            corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                image=gray,
                board=aruco_board,
                detectedCorners=corners,
                detectedIds=ids,
                rejectedCorners=rejectedImgPoints,
                cameraMatrix=camera_matrix,
                distCoeffs=dist_coeffs,
            )

            # If at least one marker was found try to estimate the pose
            if ids is not None and ids.size > 0:

                # Outline all of the markers detected in our image
                screen_img = aruco.drawDetectedMarkers(
                    screen_img, corners, ids, borderColor=(0, 255, 0)
                )

                # Estimate pose
                retval, rvec, tvec = aruco.estimatePoseBoard(
                    corners, ids, aruco_board, camera_matrix, dist_coeffs, None, None
                )

                # If pose estimation was successful draw pose
                if retval > 0:
                    aruco.drawAxis(
                        screen_img, camera_matrix, dist_coeffs, rvec, tvec, 0.2
                    )

                    # Show projection to user
                    if MAIN_CFG["vis"]["calib"]["figs"]["calib_frame"]:
                        plt.figure("Reference frame")
                        plt.imshow(screen_img)
                        plt.show()
                    return retval, rvec, tvec
                else:
                    rospy.logwarn(
                        "Pose of arcuboard could not be found please try again."
                    )
                    return False, None, None
            else:
                rospy.logwarn(
                    "Arcuboard could not be detected make sure the arcuboard is "
                    "present."
                )

        # Display timeout message and return
        rospy.logwarn(
            "Arcuboard detector times out after %s. seconds. Please reposition the "
            "arcuboard and try again.",
            CALIB_TRY_DURATION,
        )
        return False, None, None

    def chessboard_pose_estimation(self):
        """Function that performs the camera/world calibration by using
        a chessboard as a reference.

        Returns
        -------
        retval : :py:obj:`bool`
            Calibration success bool.
        rvec : :py:obj:`list`
            Rotation vector.
        tvec : :py:obj:`list`
            Translation vector.
        """

        # Get current time
        start_time = rospy.get_time()

        # Try till chessboard is found or till try time is over
        while rospy.get_time() < start_time + CALIB_TRY_DURATION:

            # Retrieve color image and convert to opencv format
            color_image = self.color_image
            camera_info = self.camera_info_hd
            color_image_cv = self._cv_bridge.imgmsg_to_cv2(
                color_image, desired_encoding="passthrough"
            )

            # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
            objp = np.zeros((N_COLMNS * N_ROWS, 3), np.float32)
            objp[:, :2] = (
                np.mgrid[0:N_ROWS, 0:N_COLMNS].T.reshape(-1, 2) * SQUARE_SIZE
            )  # Multiply by chessboard scale factor to get results in mm
            axis = (
                np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
                * SQUARE_SIZE
            )  # Coordinate axis

            # Get camera information
            camera_matrix = np.array(camera_info.K).reshape(3, 3)
            dist_coeffs = camera_info.D  # Default distortion parameters are 0

            # Get gray image
            gray = cv2.cvtColor(color_image_cv, cv2.COLOR_BGR2GRAY)

            # Create screen display image
            screen_img = cv2.cvtColor(copy.copy(color_image_cv), cv2.COLOR_RGB2BGR)

            # Find the chess board corners
            retval, corners = cv2.findChessboardCorners(gray, (N_ROWS, N_COLMNS), None)

            # Find external matrix
            if retval:

                # Find corners
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), self._criteria
                )

                # Find the rotation and translation vectors.
                retval, rvecs, tvecs, inliers = cv2.solvePnPRansac(
                    objp, corners2, camera_matrix, dist_coeffs
                )

                # project 3D points to image plane
                imgpts, jac = cv2.projectPoints(
                    axis, rvecs, tvecs, camera_matrix, dist_coeffs
                )

                # Show projection to user
                screen_img = draw_axis(screen_img, corners2, imgpts)
                plt.figure("Reference frame")
                plt.imshow(screen_img)
                plt.show()
                if retval:
                    return retval, rvecs, tvecs
                else:
                    rospy.logwarn(
                        "Pose of chessboard could not be found please try again."
                    )
                    return False, None, None
            else:
                rospy.logwarn(
                    "Chessboard could not be detected make sure the chessboard is"
                    " present."
                )

        # Display timeout message and return
        rospy.logwarn(
            "Chessboard detector times out after %s. seconds. Please reposition the "
            "chessboard and try again.",
            CALIB_TRY_DURATION,
        )
        return False, None, None

    def broadcast_camera_frame(self, calib_type="aruco_board"):
        """Send the sensor pose we acquired from the calibration to the tf2_broadcaster so that
        it can broadcast the sensor camera frame.

        Parameters
        ----------
        calib_type : :py:obj:`str`, optional
            The calibration pattern you want to use, by default "aruco_board"
        """

        # Check calibration method
        if calib_type == "chessboard":

            # Get rotation matrix
            R = np.zeros(shape=(3, 3))
            J = np.zeros(shape=(3, 3))
            cv2.Rodrigues(self.rvec, R, J)

            # Compute inverse rotation and translation matrix
            R = R.T
            tvec = np.dot(-R, self.tvec)

            # Create homogeneous matrix and the flip x and y axis
            H = np.empty((4, 4))
            H[:3, :3] = R
            H[:3, 3] = tvec.reshape(1, 3)
            H[3, :] = [0, 0, 0, 1]
            H = np.dot(
                np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]), H
            )
            R = H[0:3, 0:3]
            quat = Quaternion(matrix=R)

            # Print Calibration information
            cal_pose = {
                "x": float(H[0, 3] / 1000.0),
                "y": float(H[1, 3] / 1000.0),
                "z": float(H[2, 3] / 1000.0),
                "q1": float(quat[1]),
                "q2": float(quat[2]),
                "q3": float(quat[3]),
                "q4": float(quat[0]),
            }
            rospy.loginfo(
                "Calibration result: x={x}, y={y}, z={z}, q1={q1}, q2={q2},"
                " q3={q3} and q4={q4}".format(**cal_pose)
            )

            # Create geometry_msg
            sensor_frame_tf_msg = TransformStamped()
            sensor_frame_tf_msg.header.stamp = rospy.Time.now()
            sensor_frame_tf_msg.header.frame_id = "calib_frame"
            sensor_frame_tf_msg.child_frame_id = "kinect2_rgb_optical_frame"
            sensor_frame_tf_msg.transform.translation.x = float(H[0, 3] / 1000.0)
            sensor_frame_tf_msg.transform.translation.y = float(H[1, 3] / 1000.0)
            sensor_frame_tf_msg.transform.translation.z = float(H[2, 3] / 1000.0)
            sensor_frame_tf_msg.transform.rotation.x = float(quat[1])
            sensor_frame_tf_msg.transform.rotation.y = float(quat[2])
            sensor_frame_tf_msg.transform.rotation.z = float(quat[3])
            sensor_frame_tf_msg.transform.rotation.w = float(quat[0])

            # Communicate sensor_frame_pose to the tf2_broadcaster node
            self._set_sensor_pose_srv(sensor_frame_tf_msg)
        else:

            # Get rotation matrix
            R = np.zeros(shape=(3, 3))
            J = np.zeros(shape=(3, 3))
            cv2.Rodrigues(self.rvec, R, J)

            # Compute inverse rotation and translation matrix
            R = R.T
            tvec = np.dot(-R, self.tvec)

            # Create homogenious matrix and the flip x and y axis
            H = np.empty((4, 4))
            H[:3, :3] = R
            H[:3, 3] = tvec.reshape(1, 3)
            H[3, :] = [0, 0, 0, 1]
            R = H[0:3, 0:3]
            quat = Quaternion(matrix=R)

            # Print Calibration information
            cal_pose = {
                "x": float(H[0, 3]),
                "y": float(H[1, 3]),
                "z": float(H[2, 3]),
                "q1": float(quat[1]),
                "q2": float(quat[2]),
                "q3": float(quat[3]),
                "q4": float(quat[0]),
            }
            rospy.logdebug(
                "Calibration result: x={x}, y={y}, z={z}, q1={q1}, q2={q2},"
                " q3={q3} and q4={q4}".format(**cal_pose)
            )

            # Create geometry_msg
            sensor_frame_tf_msg = TransformStamped()
            sensor_frame_tf_msg.header.stamp = rospy.Time.now()
            sensor_frame_tf_msg.header.frame_id = "calib_frame"
            sensor_frame_tf_msg.child_frame_id = "kinect2_rgb_optical_frame"
            sensor_frame_tf_msg.transform.translation.x = float(H[0, 3])
            sensor_frame_tf_msg.transform.translation.y = float(H[1, 3])
            sensor_frame_tf_msg.transform.translation.z = float(H[2, 3])
            sensor_frame_tf_msg.transform.rotation.x = float(quat[1])
            sensor_frame_tf_msg.transform.rotation.y = float(quat[2])
            sensor_frame_tf_msg.transform.rotation.z = float(quat[3])
            sensor_frame_tf_msg.transform.rotation.w = float(quat[0])

            # Communicate sensor_frame_pose to the tf2_broadcaster node
            self._set_sensor_pose_srv(sensor_frame_tf_msg)
