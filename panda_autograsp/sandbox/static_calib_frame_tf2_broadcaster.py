#!/usr/bin/env python
"""This node publishes the camera frame when the calibration is performed.
"""

## Import standard library packages ##
import sys

## Import ros packages ##
import rospy
import tf
import tf2_ros
import geometry_msgs.msg

#################################################
## Main script ##################################
#################################################
if __name__ == '__main__':

    ## Initialize TF2 broadcaster node ##
    rospy.init_node('static_calib_frame_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    ## Check if enough arguments are supplied ##
    if len(sys.argv) < 3:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './static_calib_frame_tf2_broadcaster.py '
                     'frame_id child_frame_name')
        sys.exit(0)
    else:

        ## Check if frame name is valid ##
        if sys.argv[2] == 'world':
            rospy.logerr('Your frame cannot be named "world"')
            sys.exit(0)

        ## Load calib frame pose parameters ##
        x_pos = rospy.get_param("x_pos")
        y_pos = rospy.get_param("y_pos")
        z_pos = rospy.get_param("z_pos")
        yaw = rospy.get_param("yaw")
        pitch = rospy.get_param("pitch")
        roll = rospy.get_param("roll")

        ## Generate static transform msg ##
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = sys.argv[1]
        static_transformStamped.child_frame_id = sys.argv[2]
        static_transformStamped.transform.translation.x = float(x_pos)
        static_transformStamped.transform.translation.y = float(y_pos)
        static_transformStamped.transform.translation.z = float(z_pos)
        quat = tf.transformations.quaternion_from_euler(
                   float(yaw),float(pitch),float(roll), axes='rzyx')
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        ## Broadcast transform ##
        broadcaster.sendTransform(static_transformStamped)
        rospy.spin()
