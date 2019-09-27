#!/usr/bin/env python
""" tf2 broadcaster node
This node broadcasts a number of tf2 frames and ensures these frames
can be updated using a dynamic reconfigure server.
"""

## Import standard library packages ##
import sys

## Import ros packages ##
import rospy
from dynamic_reconfigure.server import Server
from panda_autograsp.cfg import CalibFramesConfig
import tf_conversions
import tf
import tf2_ros
import geometry_msgs.msg
from pyquaternion import Quaternion

#################################################
## tf2_broadcaster class ########################
#################################################
class tf2_broadcaster():
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

		## Set calib frame member variables ##
		self.calib_frame_x_pos = rospy.get_param(rospy.get_name()+"/calib_frame_x_pos")
		self.calib_frame_y_pos = rospy.get_param(rospy.get_name()+"/calib_frame_y_pos")
		self.calib_frame_z_pos = rospy.get_param(rospy.get_name()+"/calib_frame_z_pos")
		self.calib_frame_yaw = rospy.get_param(rospy.get_name()+"/calib_frame_yaw")
		self.calib_frame_pitch = rospy.get_param(rospy.get_name()+"/calib_frame_pitch")
		self.calib_frame_roll = rospy.get_param(rospy.get_name()+"/calib_frame_roll")

		## Set sensor starting parameters ##
		quat = Quaternion(rospy.get_param(rospy.get_name()+"/sensor_frame_q1"), rospy.get_param(rospy.get_name()+"/sensor_frame_q2"),
                    rospy.get_param(rospy.get_name()+"/sensor_frame_q3"), rospy.get_param(rospy.get_name()+"/sensor_frame_q4")).normalised
		quat = Quaternion(0, 0, 0, 1) if not quat.__nonzero__() else quat
		rospy.set_param(rospy.get_name()+"/sensor_frame_q1", float(quat[0]))
		rospy.set_param(rospy.get_name()+"/sensor_frame_q2", float(quat[1]))
		rospy.set_param(rospy.get_name()+"/sensor_frame_q3", float(quat[2]))
		rospy.set_param(rospy.get_name()+"/sensor_frame_q4", float(quat[3]))

		euler = tf.transformations.euler_from_quaternion(list(quat), axes='szyx')
		self.sensor_frame_yaw = euler[0]
		self.sensor_frame_pitch = euler[1]
		self.sensor_frame_roll = euler[2]

		## Initialize transform broadcaster ##
		self.timer = rospy.Timer(rospy.Duration(1.0/10.0),
                           self.tf2_broadcaster_callback)
		self.br = tf2_ros.TransformBroadcaster()

		## Initialize dynamic reconfigure server ##
		self.config = CalibFramesConfig.defaults
		self.srv = Server(CalibFramesConfig, self.dync_reconf_callback)

		## Update dynamic reconfigure server ##
		self.srv.update_configuration({
			"sensor_frame_yaw": self.sensor_frame_yaw,
			"sensor_frame_pitch": self.sensor_frame_pitch,
			"sensor_frame_roll": self.sensor_frame_roll
		})

	def dync_reconf_callback(self, config, level):
		"""Dynamic reconfigure callback function.

		Parameters
		----------
		config : dict
			Dictionary containing the dynamically reconfigured parameters.
		level : int
			A bitmask which will later be passed to the dynamic reconfigure callback.
			When the callback is called all of the level values for parameters that have
			been changed are added together and the resulting value is passed to the callback.
		"""

		## Print information about which values were changed##
		diff_items = {k: config[k]
                    for k in config if k in self.config and config[k] != self.config[k]}
		if not diff_items == {}:
			log_msg = ""
			for key, value in diff_items.items():
				log_msg = log_msg + ", {}: {}".format(key, diff_items[key])
			rospy.loginfo("Reconfigure Request: " + log_msg[2:])

		#########################################
		## Update calib frame parms #############
		#########################################

		## Update calib frame member variables ##
		self.calib_frame_x_pos = config["calib_frame_x_pos"]
		self.calib_frame_y_pos = config["calib_frame_y_pos"]
		self.calib_frame_z_pos = config["calib_frame_z_pos"]
		self.calib_frame_yaw = config["calib_frame_yaw"]
		self.calib_frame_pitch = config["calib_frame_pitch"]
		self.calib_frame_roll = config["calib_frame_roll"]

		## Update calib frame ros parameters ##
		rospy.set_param(rospy.get_name()+"/calib_frame_x_pos",
                  config["calib_frame_x_pos"])
		rospy.set_param(rospy.get_name()+"/calib_frame_y_pos",
                  config["calib_frame_y_pos"])
		rospy.set_param(rospy.get_name()+"/calib_frame_z_pos",
                  config["calib_frame_z_pos"])
		rospy.set_param(rospy.get_name()+"/calib_frame_yaw",
                  config["calib_frame_yaw"])
		rospy.set_param(rospy.get_name()+"/calib_frame_pitch",
                  config["calib_frame_pitch"])
		rospy.set_param(rospy.get_name()+"/calib_frame_roll",
                  config["calib_frame_roll"])

		#########################################
		## Update sensor frame parms ############
		#########################################

		## Get old sensor frame pose values ##
		sensor_pose_old = {
			"sensor_frame_x_pos": rospy.get_param(rospy.get_name()+"/sensor_frame_x_pos"),
			"sensor_frame_y_pos": rospy.get_param(rospy.get_name()+"/sensor_frame_y_pos"),
			"sensor_frame_z_pos": rospy.get_param(rospy.get_name()+"/sensor_frame_z_pos"),
			"sensor_frame_yaw": rospy.get_param(rospy.get_name()+"/sensor_frame_yaw"),
			"sensor_frame_pitch": rospy.get_param(rospy.get_name()+"/sensor_frame_pitch"),
			"sensor_frame_roll": rospy.get_param(rospy.get_name()+"/sensor_frame_roll")
		}
		quat_old = [rospy.get_param(rospy.get_name()+"/sensor_frame_q1"),
                    rospy.get_param(rospy.get_name()+"/sensor_frame_q2"),
                    rospy.get_param(rospy.get_name()+"/sensor_frame_q3"),
                    rospy.get_param(rospy.get_name()+"/sensor_frame_q4")]

		## Get relative rotaion ##
		if any([config[key] != sensor_pose_old[key] for key in config if key in sensor_pose_old]):

			## Get relative rotaion ##
			yaw_diff = config["sensor_frame_yaw"] - sensor_pose_old["sensor_frame_yaw"]
			pitch_diff = config["sensor_frame_pitch"] - \
				sensor_pose_old["sensor_frame_yaw"]
			roll_diff = config["sensor_frame_roll"] - \
				sensor_pose_old["sensor_frame_yaw"]

			## Apply relative rotation to old quaternion ##
			quat_diff = tf.transformations.quaternion_from_euler(
				yaw_diff, pitch_diff, roll_diff)
			quat = tf.transformations.quaternion_multiply(quat_diff, quat_old)

			## Set new quaternion values ##
			rospy.set_param(rospy.get_name()+"/sensor_frame_q1", float(quat[0]))
			rospy.set_param(rospy.get_name()+"/sensor_frame_q2", float(quat[1]))
			rospy.set_param(rospy.get_name()+"/sensor_frame_q3", float(quat[2]))
			rospy.set_param(rospy.get_name()+"/sensor_frame_q4", float(quat[3]))
			rospy.set_param(rospy.get_name()+"/sensor_frame_x_pos", config["sensor_frame_x_pos"])
			rospy.set_param(rospy.get_name()+"/sensor_frame_y_pos", config["sensor_frame_y_pos"])
			rospy.set_param(rospy.get_name()+"/sensor_frame_z_pos", config["sensor_frame_z_pos"])
			rospy.set_param(rospy.get_name()+"/sensor_frame_yaw", config["sensor_frame_yaw"])
			rospy.set_param(rospy.get_name()+"/sensor_frame_pitch", config["sensor_frame_pitch"])
			rospy.set_param(rospy.get_name()+"/sensor_frame_roll", config["sensor_frame_roll"])

		## Save current config ##
		self.config = config

		## Return possibly updated configuration ##
		return config

	def tf2_broadcaster_callback(self, event=None):
		"""TF2 broadcaster callback function. This function broadcast the tf2 frames.

		Parameters
		----------
		event : rospy.timer.TimerEvent, optional
			Structure passed in provides you timing information that can be useful when debugging or profiling. , by default None
		"""

		#########################################
		## Generate calib Frame msg #############
		#########################################
		calib_frame_tf_msg = geometry_msgs.msg.TransformStamped()
		calib_frame_tf_msg.header.stamp = rospy.Time.now()
		calib_frame_tf_msg.header.frame_id = self.parent_frame
		calib_frame_tf_msg.child_frame_id = self.child_id
		calib_frame_tf_msg.transform.translation.x = self.calib_frame_x_pos
		calib_frame_tf_msg.transform.translation.y = self.calib_frame_y_pos
		calib_frame_tf_msg.transform.translation.z = self.calib_frame_z_pos
		calib_quat = tf.transformations.quaternion_from_euler(
			float(self.calib_frame_yaw), float(self.calib_frame_pitch), float(self.calib_frame_roll), axes='rzyx')
		calib_frame_tf_msg.transform.rotation.x = calib_quat[0]
		calib_frame_tf_msg.transform.rotation.y = calib_quat[1]
		calib_frame_tf_msg.transform.rotation.z = calib_quat[2]
		calib_frame_tf_msg.transform.rotation.w = calib_quat[3]

		#########################################
		## Generate sensor Frame msg ############
		#########################################
		sensor_frame_tf_msg = geometry_msgs.msg.TransformStamped()
		sensor_frame_tf_msg.header.stamp = rospy.Time.now()
		sensor_frame_tf_msg.header.frame_id = "calib_frame"
		sensor_frame_tf_msg.child_frame_id = "kinect2_rgb_optical_frame"
		sensor_frame_tf_msg.transform.translation.x = rospy.get_param(rospy.get_name()+"/sensor_frame_x_pos")
		sensor_frame_tf_msg.transform.translation.y = rospy.get_param(rospy.get_name()+"/sensor_frame_y_pos")
		sensor_frame_tf_msg.transform.translation.z = rospy.get_param(rospy.get_name()+"/sensor_frame_z_pos")
		sensor_frame_tf_msg.transform.rotation.x = rospy.get_param(rospy.get_name()+"/sensor_frame_q1")
		sensor_frame_tf_msg.transform.rotation.y = rospy.get_param(rospy.get_name()+"/sensor_frame_q2")
		sensor_frame_tf_msg.transform.rotation.z = rospy.get_param(rospy.get_name()+"/sensor_frame_q3")
		sensor_frame_tf_msg.transform.rotation.w = rospy.get_param(rospy.get_name()+"/sensor_frame_q4")

		## Send tf message ##
		self.br.sendTransform([calib_frame_tf_msg, sensor_frame_tf_msg])


#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

	## Initialize TF2 broadcaster node ##
	rospy.init_node('tf2_broadcaster', anonymous=False)

	## Check if enough arguments are supplied ##
	if len(sys.argv) < 3:
		rospy.logerr('Invalid number of parameters\nusage: '
                    './tf2_broadcaster.py '
                    'frame_id child_frame_name')
		sys.exit(0)
	else:

		## Check if frame name is valid ##
		if sys.argv[2] == 'world':
			rospy.logerr('Your frame cannot be named "world"')
			sys.exit(0)

	## Initialize calib frame broadcaster ##
	tf2_broadcaster(sys.argv[1], sys.argv[2])

	## Spin forever ##
	rospy.spin()
