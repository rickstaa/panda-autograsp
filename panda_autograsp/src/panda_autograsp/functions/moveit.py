"""Additional moveit helper functions.
"""

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
