""" Moveit Pick and place
This sets up a service where you can give a goal position for the robot, and it will plan a path to it
(optionally executing it). This is standard moveit functionality, but this module wraps it specially
for the franka panda robot.
"""

## Import standard library packages ##
import sys
import os
import copy
import math

## Import ROS packages ##
import rospy
import random
import moveit_commander
import tf2_ros

## Import ROS messages and services ##
# from tf2_geometry_msgs.msg import tf2_geometry_msgs
from trajectory_msgs import JointTrajectory
from moveit_msgs import (Grasp, PlaceLocation, CollisionObject)

def open_gripper(posture):

	## Add both finger joints of panda robot. ##
	posture.joint_names.resize(2)
	posture.joint_names[0] = "panda_finger_joint1"
	posture.joint_names[1] = "panda_finger_joint2"

	## Set them as open, wide enough for the object to fit. ##
	posture.points.resize(1)
	posture.points[0].positions.resize(2)
	posture.points[0].positions[0] = 0.04
	posture.points[0].positions[1] = 0.04
	posture.points[0].time_from_start = rospy.Duration(2.5)

def closedGripper(posture):

	## Add both finger joints of panda robot. ##
	posture.joint_names.resize(2)
	posture.joint_names[0] = "panda_finger_joint1"
	posture.joint_names[1] = "panda_finger_joint2"

	## Set them as closed. ##
	posture.points.resize(1)
	posture.points[0].positions.resize(2)
	posture.points[0].positions[0] = 0.00
	posture.points[0].positions[1] = 0.00
	posture.points[0].time_from_start = rospy.Duration(2.5)

def pick(move_group):

	## Create grasps
	# Create a vector of grasps to be attempted, currently only creating single grasp.
	# This is essentially useful when using a grasp generator to generate and test multiple grasps.
	grasps = [Grasp()]

	## Setting grasp pose
	grasps[0].grasp_pose.header.frame_id = "panda_link0"
	orientation = tf2_py.Quaternion()
	orientation.setRPY(-math.pi / 2, -math.pi / 4, -math.pi / 2)
	grasps[0].grasp_pose.pose.orientation = tf2_py.toMsg(orientation)
	grasps[0].grasp_pose.pose.position.x = 0.415
	grasps[0].grasp_pose.pose.position.y = 0
	grasps[0].grasp_pose.pose.position.z = 0.5

	## Setting pre-grasp approach
	# Defined with respect to frame_id
	# Direction is set as positive x axis
	grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0"
	grasps[0].pre_grasp_approach.direction.vector.x = 1.0
	grasps[0].pre_grasp_approach.min_distance = 0.095
	grasps[0].pre_grasp_approach.desired_distance = 0.115

	## Setting post-grasp retreat
	## Defined with respect to frame_id
	## Direction is set as positive z axis
	grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0"
	grasps[0].post_grasp_retreat.direction.vector.z = 1.0
	grasps[0].post_grasp_retreat.min_distance = 0.1
	grasps[0].post_grasp_retreat.desired_distance = 0.25

	## Setting posture of eef before grasp ##
	openGripper(grasps[0].pre_grasp_posture)

	## Close gripper ##
	closedGripper(grasps[0].grasp_posture)

	## Set support surface ##
	# Set support surface as table1.
	move_group.setSupportSurfaceName("table1")

	## Call pick to pick up the object using the grasps given
	move_group.pick("object", grasps)

def place(group):

	## BEGIN_SUB_TUTORIAL place ##
	# location in
	# verbose mode." This is a known issue and we are working on fixing it. |br|
	# Create a vector of placings to be attempted, currently only creating single place location.
	place_location = [PlaceLocation()]

	## Setting place location pose ##
	place_location[0].place_pose.header.frame_id = "panda_link0"
	orientation = tf2_py.Quaternion()
	orientation.setRPY(0, 0, M_PI / 2)
	place_location[0].place_pose.pose.orientation = tf2_py.toMsg(orientation)

	## While placing it is the exact location of the center of the object. ##
	place_location[0].place_pose.pose.position.x = 0
	place_location[0].place_pose.pose.position.y = 0.5
	place_location[0].place_pose.pose.position.z = 0.5

	## Setting pre-place approach
	# Defined with respect to frame_id
	# Direction is set as negative z axis ##
	place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0"
	place_location[0].pre_place_approach.direction.vector.z = -1.0
	place_location[0].pre_place_approach.min_distance = 0.095
	place_location[0].pre_place_approach.desired_distance = 0.115

	## Setting post-grasp retreat
	# Defined with respect to frame_id
	# Direction is set as negative y axis
	place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0"
	place_location[0].post_place_retreat.direction.vector.y = -1.0
	place_location[0].post_place_retreat.min_distance = 0.1
	place_location[0].post_place_retreat.desired_distance = 0.25

	## Setting posture of eef after placing object
	# Similar to the pick case
	openGripper(place_location[0].post_place_posture)

	## Set support surface as table2.
	group.setSupportSurfaceName("table2")

	## Call place to place the object using the place locations given.
	group.place("object", place_location)

def addCollisionObjects(planning_scene_interface):

	## Create vector to hold 3 collision objects.
	collision_objects = [CollisionObject()]*3

	## Add the first table where the cube will originally be kept.
	collision_objects[0].id = "table1"
	collision_objects[0].header.frame_id = "panda_link0"

	## Define the primitive and its dimensions. ##
	collision_objects[0].primitives.resize(1)
	collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX
	collision_objects[0].primitives[0].dimensions.resize(3)
	collision_objects[0].primitives[0].dimensions[0] = 0.2
	collision_objects[0].primitives[0].dimensions[1] = 0.4
	collision_objects[0].primitives[0].dimensions[2] = 0.4

	## Define the pose of the table. ##
	collision_objects[0].primitive_poses.resize(1)
	collision_objects[0].primitive_poses[0].position.x = 0.5
	collision_objects[0].primitive_poses[0].position.y = 0
	collision_objects[0].primitive_poses[0].position.z = 0.2
	collision_objects[0].operation = collision_objects[0].ADD

	## BEGIN_SUB_TUTORIAL table2
	# Add the second table where we will be placing the cube.
	collision_objects[1].id = "table2"
	collision_objects[1].header.frame_id = "panda_link0"

	## Define the primitive and its dimensions. ##
	collision_objects[1].primitives.resize(1)
	collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX
	collision_objects[1].primitives[0].dimensions.resize(3)
	collision_objects[1].primitives[0].dimensions[0] = 0.4
	collision_objects[1].primitives[0].dimensions[1] = 0.2
	collision_objects[1].primitives[0].dimensions[2] = 0.4

	## Define the pose of the table. ##
	collision_objects[1].primitive_poses.resize(1)
	collision_objects[1].primitive_poses[0].position.x = 0
	collision_objects[1].primitive_poses[0].position.y = 0.5
	collision_objects[1].primitive_poses[0].position.z = 0.2
	collision_objects[1].operation = collision_objects[1].ADD

	## Define the object that we will be manipulating
	collision_objects[2].header.frame_id = "panda_link0"
	collision_objects[2].id = "object"

	## Define the primitive and its dimensions. ##
	collision_objects[2].primitives.resize(1)
	collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX
	collision_objects[2].primitives[0].dimensions.resize(3)
	collision_objects[2].primitives[0].dimensions[0] = 0.02
	collision_objects[2].primitives[0].dimensions[1] = 0.02
	collision_objects[2].primitives[0].dimensions[2] = 0.2

	## Define the pose of the object. ##
	collision_objects[2].primitive_poses.resize(1)
	collision_objects[2].primitive_poses[0].position.x = 0.5
	collision_objects[2].primitive_poses[0].position.y = 0
	collision_objects[2].primitive_poses[0].position.z = 0.5
	collision_objects[2].operation = collision_objects[2].ADD

	## Apply collision objects to scene ##
	planning_scene_interface.applyCollisionObjects(collision_objects)

if __name__ == "__main__":

	## Initialize ros node ##
	rospy.init_node('panda_arm_pick_place')
	rospy.WallDuration(1.0).sleep()

	## Create robot commander ##
	# Used to get information from the robot
	moveit_commander.RobotCommander(robot_description=robot_description, ns=rospy.get_namespace())

	## Create scene commanders ##
	# Used to get information about the world and update the robot
	# its understanding of the world.
	move_group = self.robot.get_group("panda_arm")
	planning_scene_interface = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
	move_group.setPlanningTime(45.0)

	## Add collision objects to the scene ##
	addCollisionObjects(planning_scene_interface)

	## Wait a bit for ROS things to initialize
	rospy.WallDuration(1.0).sleep()

	## Pick up object
	pick(group)
	rospy.WallDuration(1.0).sleep()

	## Place object
	place(group)

	## Loop till the service is shutdown. ##
	rospy.spin()
