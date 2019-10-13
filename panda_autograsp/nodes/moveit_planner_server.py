#!/usr/bin/env python
""" Moveit_service
This sets up a service where you can give a goal position for the robot, and it will plan a path to it
(optionally executing it). This is standard moveit functionality, but this module wraps it specially
for the franka panda robot.
"""

## Import standard library packages ##
import sys
import os
import copy

## Import ros packages ##
import rospy
import random
import moveit_commander
from moveit_commander import MoveItCommanderException

## Import services and messages ##
from moveit_msgs.msg import (CollisionObject, DisplayTrajectory)
from shape_msgs.msg import SolidPrimitive, Mesh
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene
from panda_autograsp.srv import ExecutePlan
from panda_autograsp.srv import (PlanToJoint, PlanToPoint, PlanToPath,
                                 PlanToRandomPoint, PlanToRandomJoint, PlanToRandomPath, VisualizePlan)

## Import BerkeleyAutomation packages packages ##
from autolab_core import YamlConfig

#################################################
## Script parameters ############################
#################################################

## Read panda_autograsp configuration file ##
MAIN_CFG = YamlConfig(os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../cfg/main_config.yaml")))
POINT_N_STEP = MAIN_CFG["planning"]["point"]["point_n_step"]
EEF_STEP = MAIN_CFG["planning"]["cartesian"]["eef_step"]
JUMPT_THRESHOLD = MAIN_CFG["planning"]["cartesian"]["jump_threshold"]

#################################################
## PandaPathPlanningService class ###############
#################################################
class PandaPathPlanningService:
    def __init__(self, robot_description, args, move_group="panda_arm_hand", move_group_end_effector_link="panda_gripper_center", move_group_gripper="panda_hand", pose_reference_frame="panda_link0", planner="TRRTkConfigDefault"):

        """PandaPathPlannerService class initialization.

        Parameters
        ----------
        robot_description : str
            Where to find the URDF.
        args : objects
            Roscpp args, passed on.
        move_group : str
            Name of the pose planning reference frame, by default "panda_link0".
        move_group_end_effector_link : str
            Name of the end effector link.
        move_group_gripper : str
            Name of the move group, by default "panda_arm_hand".
        pose_reference_frame : str
            Name of the planner reference frame.
        planner : str, optional
            The Path planning algorithm, by default 'RRTConnectkConfigDefault'.
        """

        ## initialize moveit_commander and robot commander ##
        moveit_commander.roscpp_initialize(args)

        ## Connect to moveit services ##
        rospy.loginfo(
            "Conneting moveit default moveit \'apply_planning_scene\' service.")
        rospy.wait_for_service('apply_planning_scene')
        try:
            self.planning_scene_srv = rospy.ServiceProxy(
                'apply_planning_scene', ApplyPlanningScene)
            rospy.loginfo("Moveit \'apply_planning_scene\' service found!")
        except rospy.ServiceException as e:
   		    rospy.logerr("Moveit \'apply_planning_scene\' service initialization failed: %s" % e)
		    shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(), self.planning_scene_srv.resolved_name)
		    rospy.logerr(shutdown_msg)
		    sys.exit(0)

        ## Create robot commander ##
        # Used to get information about the robot and control it.
        self.robot = moveit_commander.RobotCommander(
            robot_description=robot_description, ns="/")
        rospy.logdebug("Robot Groups: %s", self.robot.get_group_names())

        ## Initiate main move_group ##
        try: # Get main group
            self.move_group = self.robot.get_group(move_group)
        except MoveItCommanderException:
            rospy.logerr("There is no group named %s" % move_group)
        self.move_group.set_pose_reference_frame(pose_reference_frame) # Set pose reference frame
        self.move_group.set_planning_time(MAIN_CFG["planning"]["general"]["planning_time"])
        self.move_group.set_planner_id(planner)
        self.move_group.set_end_effector_link(move_group_end_effector_link)

        ## Create gripper move_group ##
        try:
            self.move_group_gripper = self.robot.get_group(move_group_gripper)
        except:
            rospy.logwarn("There is no group named %s. As a result you can not open or close the gripper." % move_group_gripper)
        self.move_group_gripper.set_planning_time(MAIN_CFG["planning"]["general"]["planning_time"])
        self.move_group_gripper.set_planner_id(planner)

        ## Create scene commanders ##
        # Used to get information about the world and update the robot
        # its understanding of the world.
        self.scene = moveit_commander.PlanningSceneInterface(ns="/")

        ## Create a `DisplayTrajectory`_ ROS publisher to display the plan in RVIZ ##
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            DisplayTrajectory,
                                                            queue_size=20)

        ## Print some information about the planner configuration ##
        rospy.loginfo("Using planner: %s", planner)
        rospy.logdebug("Reference frame: %s",
                       self.move_group.get_planning_frame())
        rospy.logdebug("End effector: %s", self.move_group.get_end_effector_link())
        rospy.logdebug("Current robot state: %s",
                       self.robot.get_current_state())

        ## Add our custom services ##
        rospy.loginfo("Initializing %s services.", rospy.get_name())
        rospy.Service('execute_plan', ExecutePlan, self.execute_plan_service)
        rospy.Service('plan_to_point', PlanToPoint, self.plan_to_point_service)
        rospy.Service('plan_to_joint', PlanToJoint, self.plan_to_joint_service)
        rospy.Service('plan_to_path', PlanToPath,
                      self.plan_cartesian_path_service)
        rospy.Service('plan_random_pose', PlanToRandomPoint,
                      self.plan_random_pose_service)
        rospy.Service('plan_random_joint', PlanToRandomJoint,
                      self.plan_random_joint_service)
        rospy.Service('plan_random_path', PlanToRandomPath,
                      self.plan_random_cartesian_path_service)
        rospy.Service('visualize_plan', VisualizePlan,
                      self.visualize_plan_service)
        rospy.loginfo(
            "\'%s\' services initialized successfully. Waiting for requests.", rospy.get_name())

        ## Create additional class members ##
        self.current_plan = None

    def plan_to_joint_service(self, joints):
        """Plans to a given joint position (incl obstacle avoidance etc)

        Parameters
        ----------
        joints : panda_autograsp.msg.PlanToJoint
            Joint message of the type float64[7].

        Returns
        -------
        Bool
            Boolean specifying whether the planning was successful.
        """

        ## Set joint targets and plan trajectory ##
        rospy.loginfo("Planning to: \n %s", joints.target)
        self.move_group.set_joint_value_target(list(joints.target))
        plan = self.move_group.plan()

        ## Validate whether planning was successful ##
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
        """Plan to a given pose.

        Parameters
        ----------
        req : geometry_msgs/Pose target
            Pose you want to plan to.

        Returns
        -------
        bool
            Boolean specifying whether the planning was successful.
        """

        ## Initialize local variables ##
        plans = []
        points = []

        ## Set the target position ##
        rospy.loginfo("Planning to: \n %s", req.target)
        self.move_group.set_pose_target(req.target)

        ## Perform planning ##
        # Perform multiple time to get the best path.
        for i in range(POINT_N_STEP):
            plans.append(self.move_group.plan())
            points.append(len(plans[i].joint_trajectory.points))
            rospy.logdebug("Found plan %d with %d points" % (i, points[i]))

        ## Find shortest path ##
        plan = plans[points.index(min(points))]
        rospy.logdebug("Points of chosen plan: %d" %
                       len(plan.joint_trajectory.points))

        ## Validate whether planning was successful ##
        if len(plan.joint_trajectory.points) > 1:
            rospy.loginfo("Plan found")
            self.current_plan = plan
            self.move_group.clear_pose_targets()  # Clear pose targets
            return True
        else:
            rospy.logwarn("No plan found")
            self.current_plan = None
            self.move_group.clear_pose_targets()  # Clear pose targets
            return False

    def plan_cartesian_path_service(self, req):
        """Plan to a given cartesian path.

        Parameters
        ----------
        req : array<geometry_msgs/Pose>
            An array of poses (waypoints) specifying a desired path.

        Returns
        -------
        bool
            Boolean specifying whether the planning was successful.
        """

        ## Plan cartesain path ##
        (plan, fraction) = self.move_group.compute_cartesian_path(
            req.waypoints,       # waypoints to follow
            EEF_STEP,            # eef_step
            JUMPT_THRESHOLD)     # jump_threshold

        ## Validate whether planning was successful ##
        if len(plan.joint_trajectory.points) > 1:
            rospy.loginfo("Plan found")
            rospy.loginfo("%s %% of the path can be executed.",
                          str(fraction*100))
            self.current_plan = plan
            return True
        else:
            rospy.logwarn("No plan found")
            self.current_plan = None
            return False

    def visualize_plan_service(self, req):
        """Execute the plan that has been computed by the other plan services.

        Parameters
        ----------
        req : empty service request
            Request dummy arguments. Needed since ros can be started with multiple srv's.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """
        ## Generate movit_msgs ##
        if self.current_plan is not None:
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(self.current_plan)

            ## Publish plan ##
            self.display_trajectory_publisher.publish(display_trajectory)
            return True
        else:
            rospy.loginfo(
                "No plan was found please first execute the planning service.")
            return False

    def execute_plan_service(self, req):
        """Execute the plan that has been computed by the other plan services.

        Parameters
        ----------
        req : empty service request
            Request dummy arguments. Needed since ros can be started with multiple srv's.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        ## Execute plan if available ##
        if self.current_plan is not None:
            result = self.move_group.execute(
                plan_msg=self.current_plan, wait=True)

            ## Check if execution was successful ##
            if result:
                rospy.loginfo("Plan execution was successful.")
            else:
                rospy.loginfo("Plan execution was unsuccessful.")
            return result
        else:
            rospy.logwarn("No plan available for execution.")
            return False

    def plan_random_pose_service(self, req):
        """Plan to a random pose goal.

        Parameters
        ----------
        req : empty service request
            Request dummy arguments. Needed since ros can be started with multiple srv's.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        ## Create random pose ##
        rospy.loginfo("Creating random pose.")
        rand_pose = self.move_group.get_random_pose()

        ## Plan to random pose ##
        req = (lambda: None)  # Create dumpy request function object
        req.target = rand_pose  # set random pose as a property
        result = self.plan_to_point_service(req)

        ## Check whether path planning was successful ##
        if result:
            return True
        else:
            return False

    def plan_random_joint_service(self, req):
        """Plan to a random joint goal.

        Parameters
        ----------
        req : empty service request
            Request dummy arguments. Needed since ros can be started with multiple srv's.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        ## Create random pose ##
        rospy.loginfo("Creating random joint goal.")
        rand_joint = self.move_group.get_random_joint_values()

        ## Plan to random pose ##
        req = (lambda: None)  # Create dumpy request function object
        req.target = rand_joint  # set random joint goal as a property
        result = self.plan_to_joint_service(req)

        ## Check whether path planning was successful ##
        if result:
            return True
        else:
            return False

    def plan_random_cartesian_path_service(self, req):
        """Plan to a random cartesian path.

        Parameters
        ----------
        req : PlanRandomPath.srv
            This message specifies the n_waypoints of the random path and the random path step
            scale.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """
        ## Create local variables ##
        waypoints = []
        w_pose = self.move_group.get_current_pose().pose

        ## Create a cartesian path ##
        for _ in range(req.n_waypoints):

            ## Generate random pose ##
            rand_pose = self.move_group.get_random_pose()
            w_pose.position.x = rand_pose.pose.position.x
            w_pose.position.y = rand_pose.pose.position.y
            w_pose.position.z = rand_pose.pose.position.z
            waypoints.append(copy.deepcopy(w_pose))

        ## Plan cartesian path ##
        req = (lambda: None)  # Create dumpy request function object
        req.waypoints = waypoints  # set random pose as a property
        result = self.plan_cartesian_path_service(req)

        ## Check whether path planning was successful ##
        if result:
            return True
        else:
            return False

    def openGripper(self, posture):
        """Open the gripper.

        Parameters
        ----------
        posture : trajectory_msgs.msg.JointTrajectory
                Gripper posture.
        """

        ## Add both finger joints of panda robot ##
        posture.joint_names = [str for i in range(2)]
        posture.joint_names[0] = "panda_finger_joint1"
        posture.joint_names[1] = "panda_finger_joint2"

        ## Set them as open, wide enough for the object to fit. ##
        posture.points = [JointTrajectoryPoint()]
        posture.points[0].positions = [float for i in range(2)]
        posture.points[0].positions[0] = 0.04
        posture.points[0].positions[1] = 0.04
        posture.points[0].time_from_start = rospy.Duration(0.5)

    def closedGripper(self, posture):
        """Close the gripper.

        Parameters
        ----------
        posture : trajectory_msgs.msg.JointTrajectory
                Gripper posture.
        """

        ## Add both finger joints of panda robot. ##
        posture.joint_names = [str for i in range(2)]
        posture.joint_names[0] = "panda_finger_joint1"
        posture.joint_names[1] = "panda_finger_joint2"

        ## Set them as closed. ##
        posture.points = [JointTrajectoryPoint()]
        posture.points[0].positions = [float for i in range(2)]
        posture.points[0].positions[0] = 0.00
        posture.points[0].positions[1] = 0.00
        posture.points[0].time_from_start = rospy.Duration(0.5)


#################################################
## Main script ##################################
#################################################
if __name__ == '__main__':

    ## Init service node ##
    rospy.init_node('moveit_planner_server')

    ## Create service object ##
    path_planning_service = PandaPathPlanningService(robot_description='robot_description', move_group="panda_arm", move_group_gripper="hand", pose_reference_frame="panda_link0", planner="TRRTkConfigDefault", args=sys.argv)

    ## Spin forever ##
    rospy.spin()
