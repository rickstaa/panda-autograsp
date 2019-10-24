"""Additional moveit helper functions.
"""

# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# ROS python packages
import rospy
from pyassimp.errors import AssimpError

# ROS messages and services
from moveit_msgs.msg import DisplayTrajectory

# Panda_autograsp modules, msgs and srvs
from panda_autograsp import Box, Plane, Cylinder, Sphere, Mesh

# Valid collision objects types
COLLISION_OBJ_TYPES = ["box", "plane", "cylinder", "sphere", "mesh"]


#################################################
# Functions #####################################
#################################################
def get_trajectory_duration(trajectory):
    """This function returns the duration of a given moveit trajectory.

    Parameters
    ----------
    trajectory : :py:obj:`!moveit_msgs.msg.DisplayTrajectory`
        The computed display trajectory.

    Returns
    -------
    :py:obj:`float`
        Trajectory duration in seconds.
    """

    # Test if trajectory is given as input
    if not isinstance(trajectory, DisplayTrajectory):
        raise TypeError(
            "Argument save must be of type bool, not {type}".format(
                type=type(trajectory)
            )
        )

    # initiate duration parameters
    duration = 0

    # Loop through trajectory segments
    for test in trajectory.trajectory:

        # Retrieve the duration of each trajectory segment
        if len(test.joint_trajectory.points) >= 1:
            duration += test.joint_trajectory.points[-1].time_from_start.to_sec()
        if len(test.multi_dof_joint_trajectory.points) >= 1:
            duration += test.multi_dof_joint_trajectory.points[
                -1
            ].time_from_start.to_sec()

    # Return duration in seconds
    return duration


def plan_exists(plan):
    """This function can be used to check if a plan trajectory was computed.

    Parameters
    ----------
    plan : :py:obj:`!moveit_msgs.msg.RobotTrajectory`
        The computed robot trajectory.

    Returns
    -------
    :py:obj:`bool`
        Bool specifying if a trajectory is present
    """

    # Check if a trajectory is present on the plan object
    if not all(
        [
            not (
                len(plan.joint_trajectory.points) >= 1
            ),  # True when no trajectory was found
            not (
                len(plan.multi_dof_joint_trajectory.points) >= 1
            ),  # True when no trajectory was found
        ]
    ):
        # A trajectory was found
        return True
    else:

        # No trajectory was found
        return False


def at_joint_target(current, desired, goal_tolerance):
    """This function can be used to check if the move_group is already at the desired
    joint target.

    Parameters
    ----------
    current : :py:obj:`list`
        The current joint configuration.
    desired : :py:obj:`list`
        The desired joint target.
    goal_tolerance : :py:obj:`float`
        The planner target goal tolerance.

    Returns
    -------
    :py:obj:`bool`
        Bool specifying if move_group is already at the goal target.
    """

    # Round to the goal tolerance
    accuracy = str(goal_tolerance)[::-1].find(".")
    current = [round(item, accuracy) for item in current]
    desired = [round(item, accuracy) for item in desired]

    # Check if move_group is at joint target
    if current == desired:
        return True  # Already at goal
    else:
        return False  # Not already at goal


def add_collision_objects(scene_commander, collision_cfg):
    """Add the collision objects that are specified in the
    collision_cfg :py:obj:`collections.OrderedDict` to the
    moveit planner scene. Supported constraint types are:
        * Box
        * Cube
        * Cylinder
        * Mesh
    For more information see http://bit.ly/32GuuMN.

    Parameters
    ----------
    scene_commander : :py:obj:`!moveit_commander.PlanningSceneInterface`
        The moveit scene commander object.
    collision_cfg : :py:obj:`collections.OrderedDict`
        The dictionary specifying all the moveit scene
        constraints.
    """

    # Initialize collision lists
    collision_objects = []

    # Create collision_objects list from config
    for (key, value) in collision_cfg["constraints"].items():

        # Check if collision object contains key field
        try:
            collision_type = value["type"]
        except KeyError as e:
            rospy.logwarn(
                "Collision object %s has no %s field. "
                "As a result this collision object is not created." % (key, e.args[0])
            )
            continue  # Go to next value

        # Check if collision object has valid type
        if collision_type not in COLLISION_OBJ_TYPES:
            rospy.logwarn(
                "Collision object %s has no %s field. "
                "As a result this collision object is not created." % (key, e.args[0])
            )
            continue  # Go to next value

        # Create box collision object
        if collision_type.lower() == "box":

            # Initiate box collision object
            box = Box(name=key)

            # Retrieve optional attributes
            try:
                box.size = (
                    value["dimensions"]["x"],
                    value["dimensions"]["y"],
                    value["dimensions"]["z"],
                )
            except KeyError:
                pass
            try:
                box.pose.pose.position.x = value["position"]["x"]
            except KeyError:
                pass
            try:
                box.pose.pose.position.y = value["position"]["y"]
            except KeyError:
                pass
            try:
                box.pose.pose.position.z = value["position"]["z"]
            except KeyError:
                pass
            try:
                box.pose.pose.orientation.x = value["orientation"]["x"]
            except KeyError:
                pass
            try:
                box.pose.pose.orientation.y = value["orientation"]["y"]
            except KeyError:
                pass
            try:
                box.pose.pose.orientation.z = value["orientation"]["z"]
            except KeyError:
                pass
            try:
                box.pose.pose.orientation.w = value["orientation"]["w"]
            except KeyError:
                pass
            try:
                box.pose.header.frame_id = value["reference_frame"]
            except KeyError:
                pass

            # Append collision object to
            collision_objects.append(box)

        # Generate plane  collision object
        elif collision_type.lower() == "plane":

            # Initiate plane collision object
            plane = Plane(name=key)

            # Retrieve optional attributes
            try:
                plane.normal = (
                    value["normal"]["x"],
                    value["normal"]["y"],
                    value["normal"]["z"],
                )
            except KeyError:
                pass
            try:
                plane.offset = value["offset"]
            except KeyError:
                pass
            try:
                plane.pose.header.frame_id = value["reference_frame"]
            except KeyError:
                pass  # Use class default reference frame

            # Append collision object to
            collision_objects.append(plane)

        # Generate Cylinder collision object
        elif collision_type.lower() == "cylinder":

            # Initiate sphere collision object
            cylinder = Cylinder(name=key)

            # Retrieve optional attributes
            try:
                cylinder.height = value["height"]
            except KeyError:
                pass
            try:
                cylinder.radius = value["radius"]
            except KeyError:
                pass
            try:
                cylinder.pose.pose.position.x = value["position"]["x"]
            except KeyError:
                pass
            try:
                cylinder.pose.pose.position.y = value["position"]["y"]
            except KeyError:
                pass
            try:
                cylinder.pose.pose.position.z = value["position"]["z"]
            except KeyError:
                pass
            try:
                cylinder.pose.pose.orientation.x = value["orientation"]["x"]
            except KeyError:
                pass
            try:
                cylinder.pose.pose.orientation.y = value["orientation"]["y"]
            except KeyError:
                pass
            try:
                cylinder.pose.pose.orientation.z = value["orientation"]["z"]
            except KeyError:
                pass
            try:
                cylinder.pose.pose.orientation.w = value["orientation"]["w"]
            except KeyError:
                pass
            try:
                cylinder.pose.header.frame_id = value["reference_frame"]
            except KeyError:
                pass

            # Append collision object to
            collision_objects.append(cylinder)

        # Generate sphere collision object
        elif collision_type.lower() == "sphere":

            # Initiate sphere collision object
            sphere = Sphere(name=key)

            # Retrieve optional attributes
            try:
                sphere.radius = value["radius"]
            except KeyError:
                pass
            try:
                sphere.pose.pose.position.x = value["position"]["x"]
            except KeyError:
                pass
            try:
                sphere.pose.pose.position.y = value["position"]["y"]
            except KeyError:
                pass
            try:
                sphere.pose.pose.position.z = value["position"]["z"]
            except KeyError:
                pass
            try:
                sphere.pose.pose.orientation.x = value["orientation"]["x"]
            except KeyError:
                pass
            try:
                sphere.pose.pose.orientation.y = value["orientation"]["y"]
            except KeyError:
                pass
            try:
                sphere.pose.pose.orientation.z = value["orientation"]["z"]
            except KeyError:
                pass
            try:
                sphere.pose.pose.orientation.w = value["orientation"]["w"]
            except KeyError:
                pass
            try:
                sphere.pose.header.frame_id = value["reference_frame"]
            except KeyError:
                pass

            # Append collision object to
            collision_objects.append(sphere)

        # Generate mesh collions object
        elif collision_type.lower() == "mesh":

            # Initiate sphere collision object
            mesh = Mesh(name=key, file_name=value["file_name"])

            # Retrieve optional attributes
            try:
                mesh.pose.pose.position.x = value["position"]["x"]
            except KeyError:
                pass
            try:
                mesh.pose.pose.position.y = value["position"]["y"]
            except KeyError:
                pass
            try:
                mesh.pose.pose.position.z = value["position"]["z"]
            except KeyError:
                pass
            try:
                mesh.pose.pose.orientation.x = value["orientation"]["x"]
            except KeyError:
                pass
            try:
                mesh.pose.pose.orientation.y = value["orientation"]["y"]
            except KeyError:
                pass
            try:
                mesh.pose.pose.orientation.z = value["orientation"]["z"]
            except KeyError:
                pass
            try:
                mesh.pose.pose.orientation.w = value["orientation"]["w"]
            except KeyError:
                pass
            try:
                mesh.pose.header.frame_id = value["reference_frame"]
            except KeyError:
                pass

            # Append collision object to
            collision_objects.append(mesh)
        else:
            rospy.logwarn(
                "Collision object type %s is not vallid. Please "
                "check your configuration dictionary and try again." % value["type"]
            )

    # Add collision objects to scene
    for collision_obj in collision_objects:

        # Add boxes
        if collision_obj.type.lower() == "box":
            scene_commander.add_box(
                name=collision_obj.name,
                pose=collision_obj.pose,
                size=collision_obj.size,
            )
            result = wait_for_state_update(scene_commander, collision_obj)
            if not result:
                rospy.logwarn(
                    "Adding collision object %s of type %s failed."
                    % (collision_obj.name, collision_obj.type)
                )

        # Add planes
        elif collision_obj.type.lower() == "plane":
            scene_commander.add_plane(
                name=collision_obj.name,
                pose=collision_obj.pose,
                normal=collision_obj.normal,
                offset=collision_obj.offset,
            )
            result = wait_for_state_update(scene_commander, collision_obj)
            if not result:
                rospy.logwarn(
                    "Adding collision object %s of type %s failed."
                    % (collision_obj.name, collision_obj.type)
                )

        # Add cylinders
        elif collision_obj.type.lower() == "cylinder":
            scene_commander.add_cylinder(
                name=collision_obj.name,
                pose=collision_obj.pose,
                height=collision_obj.height,
                radius=collision_obj.radius,
            )
            result = wait_for_state_update(scene_commander, collision_obj)
            if not result:
                rospy.logwarn(
                    "Adding collision object %s of type %s failed."
                    % (collision_obj.name, collision_obj.type)
                )

        # Add sphere
        elif collision_obj.type.lower() == "sphere":
            scene_commander.add_sphere(
                name=collision_obj.name,
                pose=collision_obj.pose,
                radius=collision_obj.radius,
            )
            result = wait_for_state_update(scene_commander, collision_obj)
            if not result:
                rospy.logwarn(
                    "Adding collision object %s of type %s failed."
                    % (collision_obj.name, collision_obj.type)
                )

        # Add mesh
        # FIXME: When a mesh is given it crashes the node
        elif collision_obj.type.lower() == "mesh":
            try:
                scene_commander.add_mesh(
                    name=collision_obj.name,
                    pose=collision_obj.pose,
                    filename=collision_obj.file_name,
                )
            except AssimpError as e:
                rospy.logwarn(
                    "Adding collision object %s of type %s failed "
                    "because %s mesh could not be loaded. Please "
                    "check the moveit_scene_constraints.yaml file "
                    "and try again"
                    % (collision_obj.name, collision_obj.type, collision_obj.file_name)
                )
                continue
            result = wait_for_state_update(scene_commander, collision_obj)
            if not result:
                rospy.logwarn(
                    "Adding collision object %s of type %s failed."
                    % (collision_obj.name, collision_obj.type)
                )
        else:
            rospy.logwarn(
                "Collision object type %s is not vallid. As a result the %s "
                "collision object will be scripted"
                % (collision_obj.type, collision_obj.name)
            )
            continue  # Go to next value


def wait_for_state_update(
    scene_commander, collision_obj, is_known=False, is_attached=False, timeout=4
):
    """Validate if collision object state change was successful.

    Parameters
    ----------
    scene_commander : :py:obj:`!moveit_commander.PlanningSceneInterface`
        The moveit scene commander object.
    collision_obj : :py:obj:`moveit_collision_objects`
        The moveit collision object.
    is_known : bool, optional
        Bool specifying whether an object is known, by default False
    is_attached : bool, optional
        Bool specifying whether an object is attached to something, by default False
    timeout : int, optional
        The validation timeout time, by default 4

    Returns
    -------
    bool
        Boolean specifying whether the scene update was successful.

    .. note::
        Note that attaching the box will remove it from known_objects.
    """

    # Ensuring Collision Updates Are Received
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():

        # Test if the box is in attached objects
        attached_objects = scene_commander.get_attached_objects(
            list(collision_obj.name)
        )
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        is_known = collision_obj.name in scene_commander.get_known_object_names()

        # Test if we are in the expected state
        if (is_attached == is_attached) and (is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
