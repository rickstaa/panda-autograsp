#!/usr/bin/env python3

## Import standard libray packages ##
import sys

## Import ROS python packages ##
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

# ## Import custom package modules ##
from panda_autograsp import GraspPlanner
# # from panda_autograsp import gqcnn_grasp_planner

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def gqcnn_grasp_planner():

    ## Get camera information ##
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/kinect2/hd/image_color', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # print("python"+str(sys.version_info[0]))
    gqcnn_grasp_planner()
