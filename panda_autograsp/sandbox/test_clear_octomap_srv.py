#!/usr/bin/env python

import rospy
from panda_autograsp.srv import ResetOctomap

if __name__ == "__main__":
    rospy.init_node("test_reset_octomap")
    rospy.wait_for_service("reset_octomap")
    try:
        add_two_ints = rospy.ServiceProxy("reset_octomap", ResetOctomap)
        add_two_ints()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
