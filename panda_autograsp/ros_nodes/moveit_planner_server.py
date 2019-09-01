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

## Import services and messages ##
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene
from panda_autograsp.srv import ExecutePlan
from panda_autograsp.srv import (PlanToJoint, PlanToPoint, PlanToPath, PlanToRandomPath, PlanToRandomPoint)

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
    def __init__(self, robot_description, group_name, args, planner='RRTConnectkConfigDefault', ):
        """PandaPathPlannerService class initialization.

        Parameters
        ----------
        robot_description : str
            Where to find the URDF.
        group_name : str
            name of the move group.
        args : objects
            roscpp args, passed on.
        planner : str, optional
            The Path planning algorithm, by default 'RRTConnectkConfigDefault'.
        """

        ## initialize moveit_commander and robot commander ##
        moveit_commander.roscpp_initialize(args)

        ## Init service node ##
        rospy.init_node('moveit_planner_server')

        ## Connect to moveit services ##
        rospy.loginfo("Conneting moveit default moveit \'apply_planning_scene\' service.")
        rospy.wait_for_service('apply_planning_scene')
        try:
            self.planning_scene_srv = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
            rospy.loginfo("Moveit \'apply_planning_scene\' service found!")
        except rospy.ServiceException as e:
            rospy.loginfo("Moveit \'apply_planning_scene\' service initialization failed: %s" % e)
            shutdown_msg = "Shutting down %s node because %s connection failed." % (
                rospy.get_name(), self.planning_scene_srv)
            rospy.signal_shutdown(shutdown_msg)  # Shutdown ROS node
            return

        ## Create robot commander ##
        # Used to get information from the robot
        self.robot = moveit_commander.RobotCommander(robot_description=robot_description, ns=rospy.get_namespace())
        rospy.logdebug("Robot Groups: %s", self.robot.get_group_names())

        ## Create scene commanders ##
        # Used to get information about the world and update the robot
        # its understanding of the world.
        self.move_group = self.robot.get_group(group_name)
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

        ## Specify the planner we want to use ##
        self.move_group.set_planner_id(planner)

        ## Get end effector ##
        self.eef_link = self.move_group.get_end_effector_link()

        ## Print some information about the planner configuration ##
        rospy.loginfo("Using planner: %s", planner)
        rospy.logdebug("Reference frame: %s", self.move_group.get_planning_frame())
        rospy.logdebug("End effector: %s", self.eef_link)
        rospy.logdebug("Current robot state: %s", self.robot.get_current_state())

        ## Add our custom services ##
        rospy.loginfo("Initializing %s services.", rospy.get_name())
        rospy.Service('execute_plan', ExecutePlan, self.execute_plan_service)
        rospy.Service('plan_to_point', PlanToPoint, self.plan_to_point_service)
        rospy.Service('plan_to_joint', PlanToJoint, self.plan_to_joint_service)
        rospy.Service('plan_to_path', PlanToPath, self.plan_cartesian_path_service)
        rospy.Service('plan_random_pose', PlanToRandomPoint, self.plan_random_pose_service)
        rospy.Service('plan_random_path', PlanToRandomPath, self.plan_random_cartesian_path_service)
        rospy.loginfo("%s services initialized successfully. Waiting for requests.", rospy.get_name())

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
        rospy.loginfo("Planning to %s", joints.target)
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

        ## Set the target possition ##
        rospy.loginfo("Planning to %s", req.target)
        self.move_group.set_pose_target(req.target)

        ## Perform planning ##
        # Perform multiple time to get the best path.
        for i in range(POINT_N_STEP):
            plans.append(self.move_group.plan())
            points.append(len(plans[i].joint_trajectory.points))
            rospy.logdebug("Found plan %d with %d points" % (i, points[i]))

        ## Find shortest path ##
        plan = plans[points.index(min(points))]
        rospy.logdebug("Points of chosen plan: %d" % len(plan.joint_trajectory.points))

        ## Validate whether planning was successful ##
        if len(plan.joint_trajectory.points) > 1:
            rospy.loginfo("Plan found")
            self.current_plan = plan
            self.move_group.clear_pose_targets() # Clear pose targets
            return True
        else:
            rospy.logwarn("No plan found")
            self.current_plan = None
            self.move_group.clear_pose_targets() # Clear pose targets
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
                                           EEF_STEP,        # eef_step
                                           JUMPT_THRESHOLD) # jump_threshold

        ## Validate whether planning was successful ##
        if len(plan.joint_trajectory.points) > 1:
            rospy.loginfo("Plan found")
            rospy.loginfo("%s of the path can be executed.", fraction)
            self.current_plan = plan
            return True
        else:
            rospy.logwarn("No plan found")
            self.current_plan = None
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
            Returns a bool to specify whether the plan was executed successfully
        """

        ## Execute plan if available ##
        if self.current_plan is not None:
            result = self.move_group.execute(plan_msg=self.current_plan, wait=True)
            rospy.loginfo("Execute result = %s" % result)
            # self.current_plan
            return result
        else:
            rospy.logwarn("No plan available for execution.")
            return False

    def plan_random_pose_service(self, req):
        """Plan to a random pose.

        Parameters
        ----------
        req : empty service request
            Request dummy arguments. Needed since ros can be started with multiple srv's.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully
        """

        ## Create random pose ##
        rand_pose = self.create_rand_pose(req.x_range, req.y_range, req.z_range)

        ## Plan to random pose ##
        req = lambda: None # Create dumpy request function object
        req.target = rand_pose # set random pose as a property
        result = self.plan_to_point_service(req)

        ## Check whether path planning was successful ##
        if result:
            rospy.loginfo("Plan found")
            return True
        else:
            rospy.loginfo("No plan found")
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
        [type]
            [description]
        """
        ## Create local variables ##
        waypoints = []
        w_pose = self.move_group.get_current_pose().pose

        ## Create a cartesian path ##
        for i in range(req.n_waypoints):

            w_pose.position.x += req.n_scale * random.uniform(-1,1)
            w_pose.position.y += req.n_scale * random.uniform(-1,1)
            w_pose.position.z += req.n_scale * random.uniform(-1,1)
            waypoints.append(copy.deepcopy(w_pose))

        ## Plan cartesian path ##
        result = self.plan_cartesian_path_service(waypoints)

        ## Check whether path planning was successful ##
        if result:
            rospy.loginfo("Plan found")
            return True
        else:
            rospy.loginfo("No plan found")
            return False

    def create_rand_pose(self, x_range=(-2, 2), y_range=(-2, 2), z_range=(-2, 2)):
        """Function used to create random robot poses.

        Parameters
        ----------
        x_range : tuple, optional
            The range of the x_position=(x_min, x_max), by default (-2, 2)
        y_range : tuple, optional
            The range of the y_position(y_min, y_max), by default (-2, 2)
        z_range : tuple, optional
            The range of the z_position=(z_min, z_max), by default (-2, 2)

        Returns
        -------
        geometry_msgs/Pose.msg
            Random robot pose
        """

        ## Construct random robot pose ##
        random_pose = Pose()
        random_pose.position.x = (random.random()-x_range[0])*x_range[1]
        random_pose.position.y = (random.random()-y_range[0])*y_range[1]
        random_pose.position.z = (random.random()-z_range[0])*z_range[1]
        random_pose.orientation.x = random.random()
        random_pose.orientation.y = random.random()
        random_pose.orientation.z = random.random()
        random_pose.orientation.w = random.random()

        ## Return robot pose ##
        return random_pose

#################################################
## Main script ##################################
#################################################
if __name__ == '__main__':

    ## Create service object ##
    path_planning_service = PandaPathPlanningService(robot_description='robot_description', group_name='panda_arm', args=sys.argv,
                             planner="TRRTkConfigDefault")

    # ## Initialize the ROS services ##
    # execute_plan_srv = rospy.Service("execute_plan", ExecutePlan,
    #                                        path_planning_service.execute_plan_service)
    # plan_to_joint_srv = rospy.Service("plan_to_joint", PlanToJoint,
    #                                        path_planning_service.plan_to_joint_service)
    # plan_to_point_srv = rospy.Service("plan_to_point", PlanToPoint,
    #                                        path_planning_service.plan_to_point_service)
    # plan_to_path_srv = rospy.Service("plan_to_joint", PlanToPath,
    #                                    path_planning_service.plan_cartesian_path_service)
    # plan_to_random_path_srv = rospy.Service("plan_to_joint", PlanToRandomPath,
    #                                    path_planning_service.plan_random_cartesian_path_service)
    # plan_to_random_point_srv = rospy.Service("plan_to_joint", PlanToRandomPoint,
    #                                    path_planning_service.plan_random_pose_service)
    # rospy.loginfo("Moveit planner initialized")

    ## Spin forever. ##
    rospy.spin()