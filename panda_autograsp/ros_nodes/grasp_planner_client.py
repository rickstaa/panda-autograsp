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

## Camera Topics ##
KINECT_COLOR_TOPIC = "/kinect2/sd/image_color_rect"
KINECT_DEPTH_TOPIC = "/kinect2/sd/image_depth"
KINECT_CAMERA_INFO_TOPIC = "/kinect2/hd/camera_info"

## Message filter settings ##
MSG_FILTER_QUEUE_SIZE = 5  # Max queue size
MSG_FILTER_SLOP = 0.1  # Max sync delay (in seconds)

## Grasp detection service settings ##
# GRASP_DETECTION_SRV = "/gqcnn/grasp_planner"
GRASP_DETECTION_SRV = "/grasp_planner"

#################################################
## GraspPlannerClient Class #####################
#################################################
class GraspPlannerClient():

    def __init__(self, grasp_detection_srv, color_topic, depth_topic, camera_info_topic, queue_size=5, slop=0.1):

        ## Initialize grasp_planning service ##
        rospy.loginfo("Conneting to %s service" % grasp_detection_srv)
        rospy.wait_for_service(GRASP_DETECTION_SRV)
        try:
            self.grasp_planner_service_handle = rospy.ServiceProxy(
                GRASP_DETECTION_SRV, GQCNNGraspPlanner)
        except rospy.ServiceException as e:
            rospy.loginfo("Service initialization failed: %s" % e)
            shutdown_msg = "Shutting down %s node because %s connection failed" % (
                rospy.get_name(), self.grasp_planner_service_handle)
            rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node

        ## Create msg filter subscribers ##
        rospy.loginfo("Creating camera sensor message_filter")
        color_image_sub = Subscriber(color_topic, sensor_msgs.msg.Image)
        depth_image_sub = Subscriber(depth_topic, sensor_msgs.msg.Image)
        camera_info_sub = Subscriber(
            camera_info_topic, sensor_msgs.msg.CameraInfo)

        ## Create msg filter ##
        ats = ApproximateTimeSynchronizer(
            [color_image_sub, depth_image_sub, camera_info_sub], queue_size, slop)
        ats.registerCallback(self.msg_filter_callback)

    def msg_filter_callback(self, color_image, depth_image, camera_info):
        print("got an Image and CameraInfo")
        # six.moves.input()
        self.grasp_planner_service_handle(color_image, depth_image, camera_info)

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Initialize ros node ##
    rospy.loginfo("Initializing grasp_planner_client node")
    rospy.init_node('grasp_planner_client', anonymous=True)

    ## Create GraspPlannerClient object ##
    grasp_planner_client = GraspPlannerClient(GRASP_DETECTION_SRV, KINECT_COLOR_TOPIC,
                                              KINECT_DEPTH_TOPIC, KINECT_CAMERA_INFO_TOPIC, MSG_FILTER_QUEUE_SIZE, MSG_FILTER_SLOP)

    ## Loop till the service is shutdown. ##
    rospy.spin()
