#!/usr/bin/env python
#!/usr/bin/env python
""" Calib frame tf2 broadcaster node
This node publishes the calibframe position relative to the robot base (panda_link0).
It also includes a dynamic reconfiguration server such that this frame can be changed by the user.
"""

## Import standard library packages ##
import sys

## Import ros packages ##
import rospy
from dynamic_reconfigure.server import Server
from panda_autograsp.cfg import PandaAutoGraspConfig
import tf_conversions
import tf
import tf2_ros
import geometry_msgs.msg

#################################################
## calib_frame_tf2_broadcaster script ###########
#################################################
class calib_frame_tf2_broadcaster():
    def __init__(self, parent_frame="panda_link0", child_id="calib_frame"):
        """Calib_frame_tf2_broadcaster class initialization.

        Parameters
        ----------
        parent_frame : str, optional
            Name of the parent frame, by default "panda_link0"
        child_id : str, optional
            Name of the child frame, by default "calib_frame"
        """

        ## Retrieve parameters ##
        self.parent_frame = parent_frame
        self.child_id = child_id

        ## Create static broadcster ##
        self.static_br = tf2_ros.StaticTransformBroadcaster()

        ## Initialize transform broardcaster ##
        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = geometry_msgs.msg.TransformStamped()

        ## Initialize dynamic reconfigure server ##
        self.srv = Server(PandaAutoGraspConfig, self.dync_reconf_callback)

    def dync_reconf_callback(self, config, level):
        """Dynamic reconfigure callback function.

        Parameters
        ----------
        config : dict
            Dictionary containing the dynamically reconfigured parameters.
        level : int
            A bitmask which will later be passed to the dynamic reconfigure callback.
            When the callback is called all of the level values for parameters that have
            been changed are ORed together and the resulting value is passed to the callback.
        """

        ## Print information ##
        rospy.loginfo("""Reconfigure Request: {x_pos}, {y_pos},\ 
            , {z_pos}, {yaw}, {pitch} & {roll}""".format(**config))

        ## Generate TF message ##
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.header.frame_id = self.parent_frame
        self.tf_msg.child_frame_id = self.child_id
        self.tf_msg.transform.translation.x = config["x_pos"]
        self.tf_msg.transform.translation.y = config["y_pos"]
        self.tf_msg.transform.translation.z = config["z_pos"]
        quat = tf.transformations.quaternion_from_euler(
            float(config["yaw"]), float(config["pitch"]), float(config["roll"]), axes='rzyx')
        self.tf_msg.transform.rotation.x = quat[0]
        self.tf_msg.transform.rotation.y = quat[1]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]

        ## Send tf message ##
        self.br.sendTransform(self.tf_msg)

        ## Return dynamic configuration dictionary ##
        return config

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Initialize TF2 broadcaster node ##
    rospy.init_node('calib_frame', anonymous=False)

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

    ## Initialize calib frame broadcaster ##
    calib_frame_tf2_broadcaster(sys.argv[1], sys.argv[2])

    ## Spin forever ##
    rospy.spin()
