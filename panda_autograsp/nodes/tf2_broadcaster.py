#!/usr/bin/env python
"""This node broadcasts a number of tf2 frames and ensures these frames
can be updated using a dynamic reconfigure server.
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
import os
from autolab_core import YamlConfig

# ROS python packages
import rospy
from dynamic_reconfigure.server import Server
from panda_autograsp.cfg import CalibFramesConfig
import tf_conversions
import tf2_ros
from pyquaternion import Quaternion

# Messages and services
import geometry_msgs.msg

# Panda_autograsp modules, msgs and srvs
from panda_autograsp.srv import SetSensorPose

###############################
# Read main config #########
###############################
# Read panda_autograsp configuration file
MAIN_CFG = YamlConfig(
    os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "../cfg/main_config.yaml"
        )
    )
)


#################################################
# tf2_broadcaster class #########################
#################################################
class tf2_broadcaster:
    """Tf2 broadcaster class.

    Attributes
    -------
    calib_frame_x_pos : :py:obj:`float`
        Calibration frame x position [m] (Relative to the 'panda_link0` frame).
    calib_frame_y_pos : :py:obj:`float`
        Calibration frame y position [m] (Relative to the 'panda_link0` frame).
    calib_frame_z_pos : :py:obj:`float`
        Calibration frame z position [m] (Relative to the 'panda_link0` frame).
    calib_frame_yaw : :py:obj:`float`
        Calibration frame yaw orientation [rad] (Relative to the 'panda_link0` frame).
    calib_frame_pitch : :py:obj:`float`
        Calibration frame pitch orientation [rad] (Relative to the 'panda_link0` frame).
    calib_frame_roll : :py:obj:`float`
        Calibration frame roll orientation [rad] (Relative to the 'panda_link0` frame).
    sensor_frame_x_pos : :py:obj:`float`
        Sensor frame x position [m] (Relative to the 'calib_frame` frame).
    sensor_frame_y_pos : :py:obj:`float`
        Sensor frame y position [m] (Relative to the 'calib_frame` frame).
    sensor_frame_z_pos : :py:obj:`float`
        Sensor frame z position [m] (Relative to the 'calib_frame` frame).
    sensor_frame_q1 : :py:obj:`float`
        Sensor frame x quaternion orientation [rad] (Relative to the 'calib_frame`
         frame).
    sensor_frame_q2 : :py:obj:`float`
        Sensor frame y quaternion orientation [rad] (Relative to the 'calib_frame`
         frame).
    sensor_frame_q3 : :py:obj:`float`
        Sensor frame z quaternion orientation [rad] (Relative to the 'calib_frame`
         frame).
    sensor_frame_q4 : :py:obj:`float`
        Sensor frame w quaternion orientation [rad] (Relative to the 'calib_frame`
         frame).
    sensor_frame_yaw : :py:obj:`float`
        Sensor frame yaw orientation [rad] (Relative to the 'panda_link0` frame).
    sensor_frame_pitch : :py:obj:`float`
        Sensor frame pitch orientation [rad] (Relative to the 'panda_link0` frame).
    sensor_frame_roll : :py:obj:`float`
        Sensor frame roll orientation [rad] (Relative to the 'panda_link0` frame).
    conf : :py:obj:`dict`
        Dynamic reconfigure server configuration dictionary.
    """

    def __init__(self):

        # Generate member variables
        self._just_calibrated = False  # Specifies whether a calibration was just done

        # Set calib frame member variables
        self.calib_frame_x_pos = self._get_dynamic_param("~calib_frame_x_pos")
        self.calib_frame_y_pos = self._get_dynamic_param("~calib_frame_y_pos")
        self.calib_frame_z_pos = self._get_dynamic_param("~calib_frame_z_pos")
        self.calib_frame_yaw = self._get_dynamic_param("~calib_frame_yaw")
        self.calib_frame_pitch = self._get_dynamic_param("~calib_frame_pitch")
        self.calib_frame_roll = self._get_dynamic_param("~calib_frame_roll")

        # Set sensor starting parameters
        self.sensor_frame_x_pos = self._get_dynamic_param("~sensor_frame_x_pos")
        self.sensor_frame_y_pos = self._get_dynamic_param("~sensor_frame_y_pos")
        self.sensor_frame_z_pos = self._get_dynamic_param("~sensor_frame_z_pos")
        try:
            quat_x = self._get_dynamic_param("~sensor_frame_q1")
            quat_y = self._get_dynamic_param("~sensor_frame_q2")
            quat_w = self._get_dynamic_param("~sensor_frame_q4")
            quat_z = self._get_dynamic_param("~sensor_frame_q3")
        except KeyError:  # If not found in parameter server
            quat_from_euler = tf_conversions.transformations.quaternion_from_euler(
                CalibFramesConfig.defaults["sensor_frame_yaw"],
                CalibFramesConfig.defaults["sensor_frame_pitch"],
                CalibFramesConfig.defaults["sensor_frame_roll"],
            )
            quat_x = quat_from_euler[0]
            quat_y = quat_from_euler[1]
            quat_z = quat_from_euler[2]
            quat_w = quat_from_euler[3]
        quat = Quaternion(quat_x, quat_y, quat_z, quat_w).normalised
        quat = Quaternion(0, 0, 0, 1) if not quat.__nonzero__() else quat
        euler = tf_conversions.transformations.euler_from_quaternion(list(quat))
        self.sensor_frame_q1 = float(quat[0])
        self.sensor_frame_q2 = float(quat[1])
        self.sensor_frame_q3 = float(quat[2])
        self.sensor_frame_q4 = float(quat[3])
        self.sensor_frame_yaw = euler[0]
        self.sensor_frame_pitch = euler[1]
        self.sensor_frame_roll = euler[2]

        # Initialize transform broadcaster
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / 10.0), self.tf2_broadcaster_callback
        )
        self._tf2_br = tf2_ros.TransformBroadcaster()

        # Initialize dynamic reconfigure server
        self.config = CalibFramesConfig.defaults
        self._dyn_reconfig_init = True
        self._dyn_reconfig_srv = Server(CalibFramesConfig, self.dync_reconf_callback)

        # Update dynamic reconfigure server
        self._dyn_reconfig_srv.update_configuration(
            {
                "sensor_frame_x_pos": self.sensor_frame_x_pos,
                "sensor_frame_y_pos": self.sensor_frame_y_pos,
                "sensor_frame_z_pos": self.sensor_frame_z_pos,
                "sensor_frame_yaw": self.sensor_frame_yaw,
                "sensor_frame_pitch": self.sensor_frame_pitch,
                "sensor_frame_roll": self.sensor_frame_roll,
            }
        )
        self._dyn_reconfig_init = False

        # Create set_sensor_pose service
        rospy.loginfo("Initializing %s services.", rospy.get_name())
        self._set_sensor_pose_srv = rospy.Service(
            "%s/set_sensor_pose" % rospy.get_name()[1:],
            SetSensorPose,
            self.set_sensor_pose_service,
        )

    def set_sensor_pose_service(self, sensor_pose):
        """Give sensor pose of the calibration to the tf2 broadcaster.

        Parameters
        ----------
        sensor_pose: dict
            Dictionary containing the pose of the sensor {x, y, z,q1, q2,
            q3, q4, yaw, pitch, roll}
        """

        # Copy pose to member variables
        self.sensor_frame_x_pos = sensor_pose.sensor_pose.transform.translation.x
        self.sensor_frame_y_pos = sensor_pose.sensor_pose.transform.translation.y
        self.sensor_frame_z_pos = sensor_pose.sensor_pose.transform.translation.z
        self.sensor_frame_q1 = sensor_pose.sensor_pose.transform.rotation.x
        self.sensor_frame_q2 = sensor_pose.sensor_pose.transform.rotation.y
        self.sensor_frame_q3 = sensor_pose.sensor_pose.transform.rotation.z
        self.sensor_frame_q4 = sensor_pose.sensor_pose.transform.rotation.w
        quat = [
            self.sensor_frame_q1,
            self.sensor_frame_q2,
            self.sensor_frame_q3,
            self.sensor_frame_q4,
        ]

        # Set rotation angles to dynamic parameters server
        euler = tf_conversions.transformations.euler_from_quaternion(quat)
        self._just_calibrated = True
        self._dyn_reconfig_srv.update_configuration(
            {
                "sensor_frame_x_pos": self.sensor_frame_x_pos,
                "sensor_frame_y_pos": self.sensor_frame_y_pos,
                "sensor_frame_z_pos": self.sensor_frame_z_pos,
                "sensor_frame_yaw": euler[0],
                "sensor_frame_pitch": euler[1],
                "sensor_frame_roll": euler[2],
            }
        )

        # Update calib frame ros parameters
        rospy.set_param("~calib_frame_x_pos", self.sensor_frame_x_pos)
        rospy.set_param("~calib_frame_y_pos", self.sensor_frame_y_pos)
        rospy.set_param("~calib_frame_z_pos", self.sensor_frame_z_pos)
        rospy.set_param("~calib_frame_q1", self.sensor_frame_q1)
        rospy.set_param("~calib_frame_q2", self.sensor_frame_q2)
        rospy.set_param("~calib_frame_q3", self.sensor_frame_q3)
        rospy.set_param("~calib_frame_q4", self.sensor_frame_q4)
        rospy.set_param("~calib_frame_yaw", euler[0])
        rospy.set_param("~calib_frame_pitch", euler[1])
        rospy.set_param("~calib_frame_roll", euler[2])

        # Return success bool
        return True

    def dync_reconf_callback(self, config, level):
        """Dynamic reconfigure callback function. This callback function
        updates the values of the dynamic reconfigure server.

        Parameters
        ----------
        config : :py:obj:`dict`
            Dictionary containing the dynamically reconfigured parameters.
        level : :py:obj:`int`
            A bitmask which will later be passed to the dynamic reconfigure callback.
            When the callback is called all of the level values for parameters that have
            been changed are added together and the resulting value is passed to the
            callback.
        """

        # Print information about which values were changed
        diff_items = {
            k: config[k]
            for k in config
            if k in self.config and config[k] != self.config[k]
        }
        if not diff_items == {}:
            log_msg = ""
            for key, value in diff_items.items():
                log_msg = log_msg + ", {}: {}".format(key, diff_items[key])
            rospy.loginfo("Reconfigure Request: " + log_msg[2:])

        #########################################
        # Update calib frame parms ###########
        #########################################

        # Update calib frame member variables
        self.calib_frame_x_pos = config["calib_frame_x_pos"]
        self.calib_frame_y_pos = config["calib_frame_y_pos"]
        self.calib_frame_z_pos = config["calib_frame_z_pos"]
        self.calib_frame_yaw = config["calib_frame_yaw"]
        self.calib_frame_pitch = config["calib_frame_pitch"]
        self.calib_frame_roll = config["calib_frame_roll"]

        #########################################
        # Update sensor frame parms ##########
        #########################################
        if (
            self._just_calibrated or self._dyn_reconfig_init
        ):  # If dynamic reconfigure server was called by set_sensor_pose_service

            # Update sensor frame parameters
            rospy.set_param("~sensor_frame_x_pos", config["sensor_frame_x_pos"])
            rospy.set_param("~sensor_frame_y_pos", config["sensor_frame_y_pos"])
            rospy.set_param("~sensor_frame_z_pos", config["sensor_frame_z_pos"])
            rospy.set_param("~sensor_frame_yaw", config["sensor_frame_yaw"])
            rospy.set_param("~sensor_frame_pitch", config["sensor_frame_pitch"])
            rospy.set_param("~sensor_frame_roll", config["sensor_frame_roll"])

            # Change just calibrated variable
            self._just_calibrated = False

            # Save config
            self.config = config

            # Return possibly updated configuration
            return config

        else:  # If user changed something

            # Get relative rotaion
            yaw_diff = config["sensor_frame_yaw"] - self.config["sensor_frame_yaw"]
            pitch_diff = (
                config["sensor_frame_pitch"] - self.config["sensor_frame_pitch"]
            )
            roll_diff = config["sensor_frame_roll"] - self.config["sensor_frame_roll"]

            # Apply relative rotation to old quaternion
            quat_old = [
                self.sensor_frame_q1,
                self.sensor_frame_q2,
                self.sensor_frame_q3,
                self.sensor_frame_q4,
            ]
            quat_diff = tf_conversions.transformations.quaternion_from_euler(
                yaw_diff, pitch_diff, roll_diff
            )
            quat = tf_conversions.transformations.quaternion_multiply(
                quat_diff, quat_old
            )

            # Set new quaternion values
            self.sensor_frame_x_pos = config["sensor_frame_x_pos"]
            self.sensor_frame_y_pos = config["sensor_frame_y_pos"]
            self.sensor_frame_z_pos = config["sensor_frame_z_pos"]
            self.sensor_frame_q1 = float(quat[0])
            self.sensor_frame_q2 = float(quat[1])
            self.sensor_frame_q3 = float(quat[2])
            self.sensor_frame_q4 = float(quat[3])

            # Update calib frame ros parameters
            rospy.set_param("~calib_frame_x_pos", config["sensor_frame_x_pos"])
            rospy.set_param("~calib_frame_y_pos", config["sensor_frame_y_pos"])
            rospy.set_param("~calib_frame_z_pos", config["sensor_frame_z_pos"])
            rospy.set_param("~calib_frame_q1", self.sensor_frame_q1)
            rospy.set_param("~calib_frame_q2", self.sensor_frame_q2)
            rospy.set_param("~calib_frame_q3", self.sensor_frame_q3)
            rospy.set_param("~calib_frame_q4", self.sensor_frame_q4)
            rospy.set_param("~calib_frame_yaw", config["sensor_frame_yaw"])
            rospy.set_param("~calib_frame_pitch", config["sensor_frame_yaw"])
            rospy.set_param("~calib_frame_roll", config["sensor_frame_yaw"])

            # Save current config
            self.config = config

            # Return possibly updated configuration
            return config

    def tf2_broadcaster_callback(self, event=None):
        """TF2 broadcaster callback function. This callback function broadcasts
        the tf2 frames.

        Parameters
        ----------
        event : :py:obj:`rospy.timer.TimerEvent`, optional
            Structure passed in provides you timing information that can be useful when
            debugging or profiling, by default None
        """

        #########################################
        # Generate calib Frame msg ###########
        #########################################
        calib_frame_tf_msg = geometry_msgs.msg.TransformStamped()
        calib_frame_tf_msg.header.stamp = rospy.Time.now()
        calib_frame_tf_msg.header.frame_id = "panda_link0"
        calib_frame_tf_msg.child_frame_id = "calib_frame"
        calib_frame_tf_msg.transform.translation.x = self.calib_frame_x_pos
        calib_frame_tf_msg.transform.translation.y = self.calib_frame_y_pos
        calib_frame_tf_msg.transform.translation.z = self.calib_frame_z_pos
        calib_quat = tf_conversions.transformations.quaternion_from_euler(
            float(self.calib_frame_yaw),
            float(self.calib_frame_pitch),
            float(self.calib_frame_roll),
            axes="rzyx",
        )
        calib_frame_tf_msg.transform.rotation.x = calib_quat[0]
        calib_frame_tf_msg.transform.rotation.y = calib_quat[1]
        calib_frame_tf_msg.transform.rotation.z = calib_quat[2]
        calib_frame_tf_msg.transform.rotation.w = calib_quat[3]

        #########################################
        # Generate sensor rgb frame msg ######
        #########################################
        sensor_frame_rgb_tf_msg = geometry_msgs.msg.TransformStamped()
        sensor_frame_rgb_tf_msg.header.stamp = rospy.Time.now()
        sensor_frame_rgb_tf_msg.header.frame_id = "calib_frame"
        sensor_frame_rgb_tf_msg.child_frame_id = "kinect2_rgb_optical_frame"
        sensor_frame_rgb_tf_msg.transform.translation.x = self.sensor_frame_x_pos
        sensor_frame_rgb_tf_msg.transform.translation.y = self.sensor_frame_y_pos
        sensor_frame_rgb_tf_msg.transform.translation.z = self.sensor_frame_z_pos
        sensor_frame_rgb_tf_msg.transform.rotation.x = self.sensor_frame_q1
        sensor_frame_rgb_tf_msg.transform.rotation.y = self.sensor_frame_q2
        sensor_frame_rgb_tf_msg.transform.rotation.z = self.sensor_frame_q3
        sensor_frame_rgb_tf_msg.transform.rotation.w = self.sensor_frame_q4

        #########################################
        # Generate sensor ir frame msg #######
        #########################################
        sensor_frame_ir_tf_msg = geometry_msgs.msg.TransformStamped()
        sensor_frame_ir_tf_msg.header.stamp = rospy.Time.now()
        sensor_frame_ir_tf_msg.header.frame_id = "calib_frame"
        sensor_frame_ir_tf_msg.child_frame_id = "kinect2_ir_optical_frame"
        sensor_frame_ir_tf_msg.transform.translation.x = self.sensor_frame_x_pos
        sensor_frame_ir_tf_msg.transform.translation.y = self.sensor_frame_y_pos
        sensor_frame_ir_tf_msg.transform.translation.z = self.sensor_frame_z_pos
        sensor_frame_ir_tf_msg.transform.rotation.x = self.sensor_frame_q1
        sensor_frame_ir_tf_msg.transform.rotation.y = self.sensor_frame_q2
        sensor_frame_ir_tf_msg.transform.rotation.z = self.sensor_frame_q3
        sensor_frame_ir_tf_msg.transform.rotation.w = self.sensor_frame_q4

        # Send tf message
        self._tf2_br.sendTransform(
            [calib_frame_tf_msg, sensor_frame_rgb_tf_msg, sensor_frame_ir_tf_msg]
        )

    def _get_dynamic_param(self, parameter_name):
        """This function retrieves a parameter from the parameter server
        just like the :py:meth:`!rospy.get_param` function. Additionally
        to the :py:meth:`!rospy.get_param` function, if a parameter is not
        found on the parameter server it also tries to retrieve a value
        for the parameter using the CalibFramesConfig.defaults dict.

        Returns
        -------
        :py:obj:`str`, :py:obj:`int`, :py:obj:`float`
        and :py:obj:`bool`.
            The parameter value.
        """

        # Get parameters
        try:
            return rospy.get_param(parameter_name)
        except KeyError:
            param_value = CalibFramesConfig.defaults[parameter_name.replace("~", "")]
            rospy.set_param(parameter_name, param_value)
            return param_value


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize TF2 broadcaster node
    rospy.init_node("tf2_broadcaster", anonymous=False)

    # Initialize calib frame broadcaster
    tf2_broadcaster()

    # Spin forever
    rospy.spin()
