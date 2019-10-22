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

# ROS messages and services
from moveit_msgs.msg import DisplayTrajectory

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
