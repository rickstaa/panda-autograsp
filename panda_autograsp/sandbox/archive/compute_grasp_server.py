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

## Import messages and services ##
from gqcnn.srv import GQCNNGraspPlanner
from panda_autograsp.srv import ComputeGrasp

#################################################
## Script settings ##############################
#################################################

## Message filter settings ##
MSG_FILTER_QUEUE_SIZE = 5  # Max queue size
MSG_FILTER_SLOP = 0.1  # Max sync delay (in seconds)

#################################################
## GraspPlannerClient Class #####################
#################################################


class ComputeGraspServer():

    def __init__(self, grasp_detection_srv, color_topic, depth_topic, camera_info_topic, queue_size=5, slop=0.1):

        ## Initialize grasp_planning service ##
        rospy.loginfo("Conneting to %s service." % grasp_detection_srv)
        rospy.wait_for_service(grasp_detection_srv)
        try:
            self.planning_scene_srv = rospy.ServiceProxy(
                grasp_detection_srv, GQCNNGraspPlanner)
        except rospy.ServiceException as e:
   		    rospy.logerr("Panda_autograsp \'%s\' service initialization failed: %s" % grasp_detection_srv, e)
		    shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(), self.planning_scene_srv.resolved_name)
		    rospy.logerr(shutdown_msg)
		    sys.exit(0)

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

        ## TODO: CHANGE names now confusing
        ## Initialize the ROS services ##
        rospy.Service("compute_grasp", ComputeGrasp,
                      ComputeGraspServer.compute_grasp_service)

    ## TODO: Docstring
    def msg_filter_callback(self, color_image_rect, depth_image_rect, camera_info):

        ## Call the grasp_planner_service ##
        self.color_image_rect = color_image_rect
        self.depth_image_rect = depth_image_rect
        self.camera_info = camera_info

    def compute_grasp_service(self, req):

        ## Call grasp computation service ##
        self.grasp = self.planning_scene_srv(
            self.color_image_rect, self.depth_image_rect, self.camera_info)

        ## Test if successful ##
        if self.grasp:
            return True
        else:
            return False


#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Initialize ros node ##
    rospy.loginfo("Initializing grasp_planner_service node")
    rospy.init_node('grasp_planner_service', anonymous=True)

    ## Argument parser ##
    try:
        img_quality = rospy.get_param("~grasp_img_quality")
    except KeyError:
        img_quality = 'sd'
    try:
        grasp_detection_srv = rospy.get_param("~grasp_detection_srv")
    except KeyError:
        grasp_detection_srv = 'gqcnn_grasp_planner'

    ## Create topics ##
    kinect_color_image_rect_topic = "/kinect2/%s/image_color_rect" % img_quality
    kinect_depth_image_rect_topic = "/kinect2/%s/image_depth_rect_32FC1" % img_quality
    kinect_camera_info_topic = "/kinect2/%s/camera_info" % img_quality

    ## Create GraspPlannerClient object ##
    grasp_planner_client = ComputeGraspServer(grasp_detection_srv, kinect_color_image_rect_topic,
                                           kinect_depth_image_rect_topic, kinect_camera_info_topic, MSG_FILTER_QUEUE_SIZE, MSG_FILTER_SLOP)

    ## Loop till the service is shutdown. ##
    rospy.spin()
