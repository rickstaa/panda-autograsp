#!/usr/bin/env python

import sys
import rospy
import random
import moveit_commander

import actionlib

from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from geometry_msgs.msg import Pose, PoseStamped

"""
A script to try and control the robot in moveit mode.
"""

class IIWACommander:

    def __init__(self, group_name, args):
        moveit_commander.roscpp_initialize(args)

        self.robot_description = 'iiwa/robot_description'  # Where to find the URDF
        self.ns = 'iiwa'
        # Init node
        rospy.init_node('command_iiwa_gazebo', log_level=rospy.DEBUG)

        self.rc = moveit_commander.RobotCommander(robot_description=self.robot_description, ns=self.ns)
        rospy.loginfo("Robot Groups: %s", self.rc.get_group_names())

        self.grp = self.rc.get_group(group_name)
        self.sc = moveit_commander.PlanningSceneInterface(ns=self.ns)

        planner = 'RRTConnectkConfigDefault'
        self.grp.set_planner_id(planner)
        rospy.loginfo("Using planner: %s", planner)

        rospy.loginfo("Reference frame: %s", self.grp.get_planning_frame())
        rospy.loginfo("End effector: %s", self.grp.get_end_effector_link())

        rospy.wait_for_service('iiwa/apply_planning_scene')
        self.ps_srv = rospy.ServiceProxy('iiwa/apply_planning_scene', ApplyPlanningScene)


        if False:
            # TODO doesnt seem to work... why?
            boxpose = Pose()
            boxpose.position.x = 0.6
            boxpose.position.y = 0.6
            boxpose.position.z = 0.5
            boxpose.orientation.x = 0
            boxpose.orientation.y = 0
            boxpose.orientation.z = 0
            boxpose.orientation.w = 1

            boxpose_st = PoseStamped()
            boxpose_st.pose = boxpose
            boxpose_st.header.stamp = rospy.Time.now()
            self.sc.add_box("obstruction", boxpose_st) # , size=(0.3, 0.3, 0.3))
            rospy.sleep(3.0)

    def goto_joint(self, joints):
        self.grp.go(joints, wait=True)

    def goto(self, position):
        """

        :param position: a geometry_msgs/Pose goal position
        :return: false if failed
        """
        rospy.logdebug("Planning to %s", position)
        self.grp.set_pose_target(position)


        plan = self.grp.plan()
                                # TODO: update Rviz with current state, it keeps displaying the straight up...

        rospy.loginfo("Plan points: %d" % len(plan.joint_trajectory.points))
        if len(plan.joint_trajectory.points) > 1: # Clunky way to see if the planning succeeded
            rospy.loginfo("Plan found, go!")
            self.grp.execute(plan_msg=plan, wait=True)
            return True
        else:
            rospy.logwarn("No plan found")
            return False

    def move(self, xplus, yplus, zplus):
        """
        Moves the robot (translation only) relative to current position by the give amount
        :param xplus:[m]
        :return:
        """

        curpose = self.grp.get_current_pose()
        curpose.pose.position.x += xplus
        curpose.pose.position.y += yplus
        curpose.pose.position.z += zplus

        self.goto(curpose.pose)


def create_rand_pose():
    pos1 = Pose()
    pos1.position.x = random.randrange(-6, 6, 1)/10.0
    pos1.position.y = random.randrange(-6, 6, 1)/10.0
    pos1.position.z = random.randrange(2, 7, 1)/10.0
    pos1.orientation.x = random.random()
    pos1.orientation.y = random.random()
    pos1.orientation.z = random.random()
    pos1.orientation.w = random.random()
    return pos1


if __name__ == '__main__':

    ic = IIWACommander('manipulator', sys.argv)
    rospy.loginfo("Initialized commander")
    rospy.loginfo("Moving to start pos")
    ic.goto_joint(rospy.get_param('iiwa/init_pos', [0, 0, 0, 0, 0, 0, 0]))

    pos1 = Pose()
    pos1.position.x = 0.2
    pos1.position.y = -0.4
    pos1.position.z = 0.4
    pos1.orientation.x = 0
    pos1.orientation.y = 1
    pos1.orientation.z = 0
    pos1.orientation.w = 3.14

    ic.goto(pos1)

    try:
        req = ApplyPlanningSceneRequest()
        req.scene.is_diff = True
        obj = CollisionObject()
        obj.operation = obj.ADD
        obj.id = "Obstruction"

        prim = SolidPrimitive()
        prim.type = prim.BOX
        prim.dimensions = [0.3, 0.3, 0.8]
        obj.primitives.append(prim)

        boxpose = Pose()
        boxpose.position.x = 0.5
        boxpose.position.y = 0
        boxpose.position.z = 0.4
        boxpose.orientation.x = 0
        boxpose.orientation.y = 0
        boxpose.orientation.z = 0
        boxpose.orientation.w = 1
        obj.primitive_poses.append(boxpose)

        req.scene.world.collision_objects.append(obj)

        res = ic.ps_srv(req)
        rospy.loginfo(res)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

    pos1 = Pose()
    pos1.position.x = 0.5
    pos1.position.y = -0.3
    pos1.position.z = 0.4
    pos1.orientation.x = 0
    pos1.orientation.y = 0.707
    pos1.orientation.z = 0
    pos1.orientation.w = 0.707

    pos2 = Pose()
    pos2.position.x = 0.5
    pos2.position.y = 0.3
    pos2.position.z = 0.4
    pos2.orientation.x = 0.707
    pos2.orientation.y = 0
    pos2.orientation.z = 0
    pos2.orientation.w = 0.707

    goalPositions = [pos1, pos2, pos1, pos2, create_rand_pose(), create_rand_pose(), create_rand_pose(), create_rand_pose()]

    try:
        for p in goalPositions:
            ic.goto(p)

    except rospy.ROSInterruptException:
        print('Shutting down commander')

    print("Done")