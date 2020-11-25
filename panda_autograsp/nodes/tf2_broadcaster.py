#!/usr/bin/env python
"""This node broadcasts a number of tf2 frames and ensures these frames
can be updated using a dynamic reconfigure server. It uses the
:py:class:`Tf2Broadcaster` class to do this.
"""

# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# Import ROS packages
import rospy

# Panda_autograsp modules, msgs and srvs
from panda_autograsp import Tf2Broadcaster

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize TF2 broadcaster node
    rospy.init_node("tf2_broadcaster", anonymous=False)

    # Get private parameters specified in the launch file
    try:
        sensor_base_frame = rospy.get_param("~camera_link")
    except KeyError:
        sensor_base_frame = "camera_link"
    try:
        sensor_depth_frame = rospy.get_param("~sensor_depth_frame")
    except KeyError:
        sensor_depth_frame = "sensor_depth_frame"

    # Initialize calib frame broadcaster
    Tf2Broadcaster(sensor_base_frame=sensor_base_frame)

    # Spin forever
    rospy.spin()
