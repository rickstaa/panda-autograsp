#!/usr/bin/env python
""" Moveit_service
This sets up a service where you can give a goal position for the robot, and it will plan a path to it
(optionally executing it). This is standard moveit functionality, but this module wraps it specially
for the franka panda robot.
"""

## Import standard library modules ##
import sys
import rospy
import random
import moveit_commander
# import moveit_python

## Import services and messages ##
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from command_iiwa_gazebo.srv import ExecutePlan
from command_iiwa_gazebo.srv import PlanToJoint
from command_iiwa_gazebo.srv import PlanToPoint, PlanToPointRequest


class PandaPlanningService:
    def __init__(self, robot_description, group_name, args, planner='RRTConnectkConfigDefault', ):
        """

        :param robot_description: Where to find the URDF
        :param group_name: name of the move group
        :param args: roscpp args, passed on
        """
        moveit_commander.roscpp_initialize(args)

        ## Init node ##
        rospy.init_node('moveit_planner_service', log_level=rospy.DEBUG)

        ## Connect to service ##
        rospy.logdebug("Waiting for planning scene service")
        rospy.wait_for_service('apply_planning_scene')
        self.planning_scene_srv = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
        rospy.logdebug("Found! Setting up own services")

        ## Create robot comander ##
        self.rc = moveit_commander.RobotCommander(robot_description=robot_description, ns=rospy.get_namespace())
        rospy.logdebug("Robot Groups: %s", self.rc.get_group_names())

        self.grp = self.rc.get_group(group_name)
        self.sc = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

        self.grp.set_planner_id(planner)
        rospy.loginfo("Using planner: %s", planner)

        rospy.logdebug("Reference frame: %s", self.grp.get_planning_frame())
        rospy.logdebug("End effector: %s", self.grp.get_end_effector_link())

        rospy.Service('plan_to_point', PlanToPoint, self.plan_to_point_service)
        rospy.Service('plan_to_joint', PlanToJoint, self.plan_to_joint_service)
        rospy.Service('execute_plan', ExecutePlan, self.execute_plan_service)
        rospy.logdebug("Done")
        self.current_plan = None

    def plan_to_joint_service(self, joints):
        """
        Plans to a given joint position (incl obstacle avoidance etc)
        :param joints:
        :return:
        """
        rospy.loginfo("Planning to %s", joints.target)
        self.grp.set_joint_value_target(list(joints.target))

        plan = self.grp.plan()

        rospy.logdebug("Plan points: %d" % len(plan.joint_trajectory.points))
        if len(plan.joint_trajectory.points) > 1:  # Clunky way to see if the planning succeeded
            rospy.loginfo("Plan found")
            self.current_plan = plan
            return True
        else:
            rospy.logwarn("No plan found")
            self.current_plan = None
            return False

    def plan_to_point_service(self, req):
        """
        Makes a plan to a given point
        """
        rospy.loginfo("Planning to %s", req.target)
        self.grp.set_pose_target(req.target)
        plans = []
        points = []
        for i in range(5):
            plans.append(self.grp.plan())
            points.append(len(plans[i].joint_trajectory.points))
            rospy.logdebug("Found plan %d with %d points" % (i, points[i]))
        # TODO: update Rviz with current state, it keeps displaying the straight up...
        plan = plans[points.index(min(points))]
        rospy.logdebug("Points of chosen plan: %d" % len(plan.joint_trajectory.points))
        if len(plan.joint_trajectory.points) > 1: # Clunky way to see if the planning succeeded
            rospy.loginfo("Plan found")
            self.current_plan = plan
            return True
        else:
            rospy.logwarn("No plan found")
            self.current_plan = None
            return False

    def execute_plan_service(self, arg):
        if self.current_plan is not None:
            result = self.grp.execute(plan_msg=self.current_plan, wait=True)
            rospy.loginfo("Execute result = %s" % result)
            self.current_plan = None
            return result
        else:
            rospy.logwarn("No plan available for execution")
            return False

    # Work in progress
    # def plan_to_point_with_mesh(self, request, filename):
    #     rospy.loginfo("Planning to point %s with mesh %s" % (request.target, filename))
    #
    #     planningScene = moveit_python.PlanningSceneInterface("root", ns=rospy.get_namespace())
    #     p = Pose()
    #     p.position = [0, 0, 0]
    #     p.orientation = [0, 0, 0, 1]
    #     col_obj = planningScene.makeMesh("Scanned world mesh", p, filename)
    #     col_obj.operation = col_obj.ADD
    #     rospy.loginfo("Mesh loaded")
    #
    #     req = ApplyPlanningSceneRequest()
    #     req.scene.is_diff = True
    #     req.scene.world.collision_objects.append(col_obj)
    #     res = self.planning_scene_srv(req)
    #     rospy.logdebug("Planning scene updated: %s" % res)
    #     # TODO: clear the planning scene?
    #     return self.plan_to_point_service(request)


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

    ic = PandaPlanningService(robot_description='robot_description', group_name='manipulator', args=sys.argv,
                             planner="TRRTkConfigDefault")

    rospy.spin()