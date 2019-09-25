#!/usr/bin/env python
"""This node calls the ROS GQCNN message service.
"""

## Make script both python2 and python3 compatible ##
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

## Import standard library packages ##
import sys
import six

## Import ROS python packages ##
import rospy
import sensor_msgs
from message_filters import (ApproximateTimeSynchronizer, Subscriber)

## Import BerkeleyAutomation package ##
from gqcnn.srv import (
    GQCNNGraspPlanner, GQCNNGraspPlannerBoundingBox, GQCNNGraspPlannerSegmask)

#################################################
## Script settings ##############################
#################################################

## Message filter settings ##
MSG_FILTER_QUEUE_SIZE = 5  # Max queue size
MSG_FILTER_SLOP = 0.1  # Max sync delay (in seconds)

#################################################
## GraspPlannerClient Class #####################
#################################################
class GraspPlannerClient():

    def __init__(self, grasp_detection_srv, color_topic, depth_topic, camera_info_topic, queue_size=5, slop=0.1):

        ## Initialize grasp_planning service ##
        rospy.loginfo("Conneting to %s service." % grasp_detection_srv)
        rospy.wait_for_service(grasp_detection_srv)
        try:
            self.planning_scene_srv = rospy.ServiceProxy(
                grasp_detection_srv, GQCNNGraspPlanner)
        except rospy.ServiceException as e:
            rospy.loginfo("Service initialization failed: %s" % e)
            shutdown_msg = "Shutting down %s node because %s connection failed." % (
                rospy.get_name(), self.planning_scene_srv)
            rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node
            return

        ## Create msg filter subscribers ##
        rospy.loginfo("Creating camera sensor message_filter.")
        color_image_sub = Subscriber(color_topic, sensor_msgs.msg.Image)
        depth_image_sub = Subscriber(depth_topic, sensor_msgs.msg.Image)
        camera_info_sub = Subscriber(
            camera_info_topic, sensor_msgs.msg.CameraInfo)

        ## Create msg filter ##
        ats = ApproximateTimeSynchronizer(
            [color_image_sub, depth_image_sub, camera_info_sub], queue_size, slop)
        ats.registerCallback(self.msg_filter_callback)
        rospy.loginfo("Camera sensor message_filter created.")

    def msg_filter_callback(self, color_image, depth_image, camera_info):
        raw_input("Click enter to compute a grasp.")
        self.planning_scene_srv(color_image, depth_image, camera_info)

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Initialize ros node ##
    rospy.loginfo("Initializing grasp_planner_client node")
    rospy.init_node('grasp_planner_client', anonymous=True)

    ## Argument parser ##
    try:
        img_quality = rospy.get_param("~grasp_img_quality")
    except KeyError:
        img_quality = 'sd'
    try:
        grasp_detection_srv = rospy.get_param("~grasp_detection_srv")
    except KeyError:
        grasp_detection_srv = 'grasp_planner'

    ## Create topics ##
    kinect_color_topic="/kinect2/%s/image_color_rect" % img_quality
    kinect_depth_topic = "/kinect2/%s/image_depth_rect_32FC1" % img_quality
    kinect_camera_info_topic = "/kinect2/%s/camera_info" % img_quality

    ## Create GraspPlannerClient object ##
    grasp_planner_client = GraspPlannerClient(grasp_detection_srv, kinect_color_topic,
                                              kinect_depth_topic, kinect_camera_info_topic, MSG_FILTER_QUEUE_SIZE, MSG_FILTER_SLOP)

    ## Loop till the service is shutdown. ##
    rospy.spin()
