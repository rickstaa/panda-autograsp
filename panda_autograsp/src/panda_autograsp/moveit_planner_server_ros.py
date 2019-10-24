"""This module contains the :py:class:`MoveitPlannerServer` class. This class
sets up a number of services that can be used to control the Panda Emika Franka
robot.
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
import sys
import os
import copy
import time
from autolab_core import YamlConfig

# ROS python packages
import rospy
import moveit_commander
from moveit_commander import MoveItCommanderException

# ROS messages and services
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene

# Panda_autograsp modules, msgs and srvs
from panda_autograsp.functions.moveit import (
    get_trajectory_duration,
    plan_exists,
    at_joint_target,
    add_collision_objects,
)
from panda_autograsp.srv import (
    ExecutePlan,
    PlanToJoint,
    PlanToPoint,
    PlanToPath,
    PlanToRandomPoint,
    PlanToRandomJoint,
    PlanToRandomPath,
    VisualizePlan,
    CloseGripper,
    OpenGripper,
    SetGripperWidth,
    SetGripperOpen,
    SetGripperClosed,
    PlanGripper,
    ExecuteGripperPlan,
)

#################################################
# Script parameters #############################
#################################################

# Get file path
FILE_PATH = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))

# Read panda_autograsp configuration file
MAIN_CFG = YamlConfig(
    os.path.abspath(os.path.join(FILE_PATH, "../../cfg/main_config.yaml"))
)
POINT_N_STEP = MAIN_CFG["planning"]["point"]["point_n_step"]
EEF_STEP = MAIN_CFG["planning"]["cartesian"]["eef_step"]
JUMP_THRESHOLD = MAIN_CFG["planning"]["cartesian"]["jump_threshold"]

# Read collision object configuration file
COLLISION_OBJ_CFG = YamlConfig(
    os.path.abspath(os.path.join(FILE_PATH, "../../cfg/moveit_scene_constraints.yaml"))
)


#################################################
# MoveitPlannerServer class #####################
#################################################
class MoveitPlannerServer:
    """Class used for controlling the panda emika franka robot.

    Attributes
    ------------
        robot : :py:obj:`!moveit_commander.RobotCommander`
            The moveit robot commander.
        move_group : :py:obj:`!moveit_commander.MoveGroupCommander`
            The main robot move group.
        move_group_gripper : :py:obj:`!moveit_commander.MoveGroupCommander`
            The gripper move group.
        scene : :py:obj:`!moveit_commander.PlanningSceneInterface`
            The moveit scene commander.
        current_plan :
            The last computed plan of the main move group.
        current_plan_gripper :
            The last computed plan of the gripper.
        desired_pose : :py:obj:`list`
            The main move group goal pose.
        desired_joint_values: :py:obj:`list`
            The main move group target joint values.
        desired_gripper_joint_values : :py:obj:`list`
            The gripper target joint values.
    """

    def __init__(
        self,
        robot_description,
        args,
        move_group="panda_arm",
        move_group_end_effector_link="panda_gripper_center",
        move_group_gripper="hand",
        pose_reference_frame="panda_link0",
        planner="TRRTkConfigDefault",
    ):
        """PandaPathPlannerService class initialization.

        Parameters
        ----------
        robot_description : :py:obj:`str`
                Where to find the URDF.
        args : objects
                Roscpp args, passed on.
        move_group : :py:obj:`str`
                Name of the pose planning reference frame, by default "panda_link0".
        move_group_end_effector_link : :py:obj:`str`
                Name of the end effector link.
        move_group_gripper : :py:obj:`str`
                Name of the move group, by default "panda_arm_hand".
        pose_reference_frame : :py:obj:`str`
                Name of the planner reference frame.
        planner : :py:obj:`str`, optional
                The Path planning algorithm, by default 'RRTConnectkConfigDefault'.
        """

        # Initialize class attributes
        self.current_plan = RobotTrajectory()  # Empty plan
        self.current_plan_gripper = RobotTrajectory()  # Empty plan
        self.desired_pose = []
        self.desired_joint_values = []
        self.desired_gripper_joint_values = {}

        # initialize moveit_commander and robot commander
        moveit_commander.roscpp_initialize(args)

        # Connect to moveit services
        rospy.loginfo(
            "Connecting moveit default moveit 'apply_planning_scene' " "service."
        )
        rospy.wait_for_service("apply_planning_scene")
        try:
            self._moveit_apply_planning_srv = rospy.ServiceProxy(
                "apply_planning_scene", ApplyPlanningScene
            )
            rospy.loginfo("Moveit 'apply_planning_scene' service found!")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Moveit 'apply_planning_scene' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._moveit_apply_planning_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Create robot commander
        # Used to get information about the robot and control it.
        self.robot = moveit_commander.RobotCommander(
            robot_description=robot_description, ns="/"
        )
        rospy.logdebug("Robot Groups: %s", self.robot.get_group_names())

        # Initiate main move_group
        try:
            # Get main move group
            self.move_group = self.robot.get_group(move_group)

            # Set move group settings
            self.move_group.set_pose_reference_frame(pose_reference_frame)
            self.move_group.set_planning_time(
                MAIN_CFG["planning"]["general"]["planning_time"]
            )
            self.move_group.set_planner_id(planner)

            # Set end effector
            if (
                move_group_end_effector_link in self.robot.get_link_names()
            ):  # Try to set end to user specified link (Keep default if fails).k
                self.move_group.set_end_effector_link(move_group_end_effector_link)
        except MoveItCommanderException:
            self.move_group = None
            shutdown_msg = (
                "Shutting down %s node because main move_group could not be created."
                "Please check if move group %s exists." % (rospy.get_name(), move_group)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Create gripper move_group
        try:
            self.move_group_gripper = self.robot.get_group(move_group_gripper)
            self.move_group_gripper.set_planning_time(
                MAIN_CFG["planning"]["general"]["planning_time"]
            )
            self.move_group_gripper.set_planner_id(planner)
        except MoveItCommanderException:
            self.move_group_gripper = None
            rospy.logwarn(
                "Gripper move_group creation failed. Check if move group %s exists. "
                % move_group
            )

        # Create scene commanders
        # Used to get information about the world and update the robot
        # its understanding of the world.
        self.scene = moveit_commander.PlanningSceneInterface(ns="/")

        # Create a `DisplayTrajectory`_ ROS publisher to display the plan in RVIZ
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )

        # Print some information about the planner configuration
        rospy.loginfo("Using planner: %s", planner)
        rospy.logdebug("Reference frame: %s", self.move_group.get_planning_frame())
        rospy.logdebug("End effector: %s", self.move_group.get_end_effector_link())
        rospy.logdebug("Current robot state: %s", self.robot.get_current_state())

        # Add our custom services
        rospy.loginfo("Initializing %s services.", rospy.get_name())
        rospy.Service(
            "%s/plan_to_point" % rospy.get_name()[1:],
            PlanToPoint,
            self.plan_to_point_service,
        )
        rospy.Service(
            "%s/plan_to_joint" % rospy.get_name()[1:],
            PlanToJoint,
            self.plan_to_joint_service,
        )
        rospy.Service(
            "%s/plan_to_path" % rospy.get_name()[1:],
            PlanToPath,
            self.plan_cartesian_path_service,
        )
        rospy.Service(
            "%s/plan_random_pose" % rospy.get_name()[1:],
            PlanToRandomPoint,
            self.plan_random_pose_service,
        )
        rospy.Service(
            "%s/plan_random_joint" % rospy.get_name()[1:],
            PlanToRandomJoint,
            self.plan_random_joint_service,
        )
        rospy.Service(
            "%s/plan_random_path" % rospy.get_name()[1:],
            PlanToRandomPath,
            self.plan_random_cartesian_path_service,
        )
        rospy.Service(
            "%s/visualize_plan" % rospy.get_name()[1:],
            VisualizePlan,
            self.visualize_plan_service,
        )
        rospy.Service(
            "%s/execute_plan" % rospy.get_name()[1:],
            ExecutePlan,
            self.execute_plan_service,
        )
        rospy.Service(
            "%s/plan_gripper" % rospy.get_name()[1:],
            PlanGripper,
            self.plan_gripper_service,
        )
        rospy.Service(
            "%s/open_gripper" % rospy.get_name()[1:],
            OpenGripper,
            self.open_gripper_service,
        )
        rospy.Service(
            "%s/close_gripper" % rospy.get_name()[1:],
            CloseGripper,
            self.close_gripper_service,
        )
        rospy.Service(
            "%s/set_gripper_width" % rospy.get_name()[1:],
            SetGripperWidth,
            self.set_gripper_width_service,
        )
        rospy.Service(
            "%s/set_gripper_open" % rospy.get_name()[1:],
            SetGripperOpen,
            self.set_gripper_open_service,
        )
        rospy.Service(
            "%s/set_gripper_closed" % rospy.get_name()[1:],
            SetGripperClosed,
            self.set_gripper_closed_service,
        )
        rospy.Service(
            "%s/execute_gripper_plan" % rospy.get_name()[1:],
            ExecuteGripperPlan,
            self.execute_gripper_plan_service,
        )

        # Display service initiation success message
        rospy.loginfo(
            "'%s' services initialized successfully. Waiting for requests.",
            rospy.get_name(),
        )

        # Wait some time to give moveit the time to initialize
        rospy.sleep(2)

        # Add collision objects to scene
        self._collision_obj_cfg = COLLISION_OBJ_CFG
        rospy.loginfo("Adding collision objects to the planning scene...")
        add_collision_objects(self.scene, self._collision_obj_cfg)
        rospy.loginfo("Collision objects added to the planning scene.")

    def plan_to_joint_service(self, req):
        """Plan to a given joint position.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanToJoint`
            The service request message containing the joint targets you
            want to plan to.

        Returns
        -------
        Bool
            Boolean specifying whether the planning was successful.
        """

        # Set joint targets and plan trajectory
        rospy.loginfo("Planning to: \n %s", req.target)
        self.move_group.set_joint_value_target(list(req.target))
        plan = self.move_group.plan()
        self.desired_joint_values = req.target

        # Validate whether planning was successful
        rospy.logdebug("Plan points: %d" % len(plan.joint_trajectory.points))
        if plan_exists(plan):
            rospy.loginfo("Plan to joint path planning successful.")
            self.current_plan = plan
            return True
        else:
            rospy.logwarn("Plan to joint path planning failed.")
            self.current_plan = None
            return False

    def plan_to_point_service(self, req):
        """Plan to a given pose.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanToPoint`
            The service request message containing the pose you want to plan to.

        Returns
        -------
        bool
            Boolean specifying whether the planning was successful.
        """

        # Initialize local variables
        plans = []
        points = []

        # Set the target position
        rospy.loginfo("Planning to: \n %s", req.target)
        self.move_group.set_pose_target(req.target)
        self.desired_pose = req.target

        # Perform planning
        # Perform multiple time to get the best path.
        for i in range(POINT_N_STEP):
            plans.append(self.move_group.plan())
            points.append(len(plans[i].joint_trajectory.points))
            rospy.logdebug("Found plan %d with %d points" % (i, points[i]))

        # Find shortest path
        self.current_plan = plans[points.index(min(points))]
        rospy.logdebug(
            "Points of chosen plan: %d" % len(self.current_plan.joint_trajectory.points)
        )

        # Validate whether planning was successful
        if plan_exists(self.current_plan):
            rospy.loginfo("Plan to point planning successful.")
            self.move_group.clear_pose_targets()  # Clear pose targets
            return True
        else:
            rospy.logwarn("Plan to point planning failed.")
            self.move_group.clear_pose_targets()  # Clear pose targets
            return False

    def plan_cartesian_path_service(self, req):
        """Plan to a given cartesian path.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanToPath`
            The service request message containing the path you want to plan to.

        Returns
        -------
        bool
            Boolean specifying whether the planning was successful.
        """

        # Plan cartesian path
        (self.current_plan, fraction) = self.move_group.compute_cartesian_path(
            req.waypoints, EEF_STEP, JUMP_THRESHOLD  # waypoints to follow  # eef_step
        )  # jump_threshold
        self.desired_pose = req.waypoints[-1]

        # Validate whether planning was successful
        if plan_exists(self.current_plan):
            rospy.loginfo("Cartesian path planning successful.")
            rospy.loginfo("%s %% of the path can be executed.", str(fraction * 100))
            return True
        else:
            rospy.logwarn("Cartesian path planning failed.")
            return False

    def plan_random_pose_service(self, req):
        """Plan to a random pose goal.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanToRandomPoint`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Create random pose
        rospy.loginfo("Creating random pose.")
        rand_pose = self.move_group.get_random_pose()

        # Plan to random pose
        def req():
            None  # Create dumpy request function object

        req.target = rand_pose  # set random pose as a property
        result = self.plan_to_point_service(req)

        # Check whether path planning was successful
        if result:
            rospy.loginfo("Random pose path planning successful.")
            return True
        else:
            rospy.logwarn("Random pose path planning failed.")
            return False

    def plan_random_joint_service(self, req):
        """Plan to a random joint goal.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanToRandomJoint`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Create random pose
        rospy.loginfo("Creating random joint goal.")
        rand_joint = self.move_group.get_random_joint_values()

        # Plan to random pose
        def req():
            None  # Create dumpy request function object

        req.target = rand_joint  # set random joint goal as a property
        result = self.plan_to_joint_service(req)

        # Check whether path planning was successful
        if result:
            rospy.loginfo("Random cartesian path planning successful.")
            return True
        else:
            rospy.logwarn("Random cartesian path planning failed.")
            return False

    def plan_random_cartesian_path_service(self, req):
        """Plan to a random cartesian path.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanToRandomPath`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Create local variables
        waypoints = []
        w_pose = self.move_group.get_current_pose().pose

        # Create a cartesian path
        for _ in range(req.n_waypoints):

            # Generate random pose
            rand_pose = self.move_group.get_random_pose()
            w_pose.position.x = rand_pose.pose.position.x
            w_pose.position.y = rand_pose.pose.position.y
            w_pose.position.z = rand_pose.pose.position.z
            waypoints.append(copy.deepcopy(w_pose))

        # Plan cartesian path
        def req():
            None  # Create dumpy request function object

        req.waypoints = waypoints  # set random pose as a property
        result = self.plan_cartesian_path_service(req)

        # Check whether path planning was successful
        if result:
            rospy.loginfo("Random cartesian path planning successful.")
            return True
        else:
            rospy.logwarn("Random cartesian path planning failed.")
            return False

    def visualize_plan_service(self, req):
        """Visualize the plan that has been computed by the other plan services.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.VisualizePlan`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Generate moveit_msgs
        if plan_exists(self.current_plan_gripper) or plan_exists(self.current_plan):

            # Display trajectory
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(self.current_plan_gripper)
            display_trajectory.trajectory.append(self.current_plan)

            # Get visualization duration
            duration = get_trajectory_duration(display_trajectory)

            # Publish plan
            self._display_trajectory_publisher.publish(display_trajectory)

            # Sleep till trajectory is visualized
            time.sleep(duration)

            # Return success bool
            rospy.loginfo("Movement plan visualization successful.")
            return True
        else:
            rospy.logwarn("No movement plan available for the visualization.")
            return False

    def execute_plan_service(self, req):
        """Execute the plan that has been computed for the main move group by the other
        plan services.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.ExecutePlan`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper plan is set
        if plan_exists(self.current_plan):

            # Execute plan
            result = self.move_group.execute(plan_msg=self.current_plan, wait=True)
            self.current_plan = (
                RobotTrajectory()
            )  # Empty plan (Needed for visualization).

            # Check if execution was successful
            if result:
                rospy.loginfo("Plan execution was successful.")
                return True
            else:
                rospy.logwarn("Plan execution was unsuccessful.")
                return False  # Return success bool
        else:

            # Check if the robot state is already at the desired state
            # joint_now = self.move_group.get_current_joint_values()
            # pose_now = self.move_group.get_current_pose()
            # # if (joint_now =)
            rospy.logwarn("No movement plan available for the execution.")
            return False

    def plan_gripper_service(self, req):
        """Compute plan for the currently set gripper target joint values.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanGripper.`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper can "
                "not be controlled."
            )
            return False

        # Plan for gripper command
        desired_values = self.desired_gripper_joint_values
        self.move_group_gripper.set_joint_value_target(desired_values.values())
        rospy.sleep(0.1)
        plan = self.move_group_gripper.plan()

        # Validate whether planning was successful
        if plan_exists(plan):
            rospy.loginfo("Gripper plan found.")
            return True  # Return success bool
        elif at_joint_target(
            self.move_group_gripper.get_current_joint_values(),
            self.desired_gripper_joint_values.values(),
            self.move_group_gripper.get_goal_joint_tolerance(),
        ):  # Plan empty because already at goal
            rospy.loginfo("Gripper already at desired location.")
            return True
        else:  # Plan empty
            rospy.logwarn("No gripper plan found.")
            return False

    def execute_gripper_plan_service(self, req):
        """Execute previously planned gripper plan.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.ExecuteGripperPlan.`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper "
                "can not be controlled."
            )
            return False

        # Check if gripper is already at desired location
        if at_joint_target(
            self.move_group_gripper.get_current_joint_values(),
            self.desired_gripper_joint_values.values(),
            self.move_group_gripper.get_goal_joint_tolerance(),
        ):  # Plan empty because already at goal
            rospy.loginfo("Gripper already at desired location.")
            return True
        else:

            # Execute gripper plan
            result = self.move_group_gripper.execute(
                self.current_plan_gripper, wait=True
            )
            self.current_plan_gripper = RobotTrajectory()  # Reset plan
            if not result:
                rospy.logwarn("Gripper plan could not be executed.")
                return False
            else:
                return True

    def open_gripper_service(self, req):
        """Open the gripper.

        Parameters
        ----------
        req :  :py:obj:`panda_autograsp.msg.OpenGripper.`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper "
                "can not be controlled."
            )
            return False
        # Set named target
        try:
            desired_values = self.move_group_gripper.get_named_target_values("open")
        except MoveItCommanderException as e:
            rospy.logwarn(e)
            return False

        # Plan and execute
        # Note: I used execute instead of go since it failed in some cases.
        plan = self.move_group_gripper.plan(desired_values.values())
        result = self.move_group_gripper.execute(plan, wait=True)
        if not result:
            return False
        else:
            return True  # Return success bool

    def close_gripper_service(self, req):
        """Close the gripper.

        Parameters
        ----------
        req :  :py:obj:`panda_autograsp.msg.CloseGripper.`
            Empty service request.
        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper "
                "can not be controlled."
            )
            return False

        # Set named target
        try:
            desired_values = self.move_group_gripper.get_named_target_values("close")
        except MoveItCommanderException as e:
            rospy.logwarn(e)
            return False

        # Plan and execute
        # Note: I used execute instead of go since it failed in some cases.
        plan = self.move_group_gripper.plan(desired_values.values())
        result = self.move_group_gripper.execute(plan, wait=True)
        if not result:
            return False
        else:
            return True  # Return success bool

    def set_gripper_open_service(self, req):
        """Set gripper joint targets values to open.

        Parameters
        ----------
        req :  :py:obj:`panda_autograsp.msg.SetGripperOpen.`
            Empty service request.

        Returns
        -------
        bool
                Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper "
                "can not be controlled."
            )
            return False

        # Get gripper target value to open
        try:
            # Get desired joint targets
            desired_joint_values = self.move_group_gripper.get_named_target_values(
                "open"
            )

            # Set the named joint targets
            self.move_group_gripper.set_named_target("open")

            # Save desired joint targets
            self.desired_gripper_joint_values = desired_joint_values
            return True
        except MoveItCommanderException as e:
            rospy.logwarn(e)
            return False

    def set_gripper_closed_service(self, req):
        """Set gripper joint target values to closed.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.SetGripperClosed.`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper "
                "can not be controlled."
            )
            return False

        # Get gripper target value to closed
        try:
            # Get desired joint targets
            desired_joint_values = self.move_group_gripper.get_named_target_values(
                "close"
            )

            # Set the named joint targets
            self.move_group_gripper.set_named_target("close")

            # Save desired joint targets
            self.desired_gripper_joint_values = desired_joint_values
            return True
        except MoveItCommanderException as e:
            rospy.logwarn(e)
            return False

    def set_gripper_width_service(self, req):
        """Set gripper joint target values to a given value.

        Parameters
        ----------
        req :  :py:obj:`panda_autograsp.msg.SetGripperWidth.`
            This message specifies the gripper width.

        Returns
        -------
        bool
                Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper "
                "can not be controlled."
            )
            return False

        # Try to set the joint values
        dict_keys = self.move_group_gripper.get_named_target_values("open").keys()
        dict_values = [req.gripper_width / 2.0] * 2
        try:
            # Save desired joint targets
            self.desired_gripper_joint_values = dict(zip(dict_keys, dict_values))
            return True
        except MoveItCommanderException as e:
            rospy.logwarn(e)
            return False
