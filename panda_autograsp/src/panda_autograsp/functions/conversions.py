"""A number of usefull ros data_type conversion functions.
"""

# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# main python packages
import numpy as np

# ROS transformation packages
import tf_conversions


#################################################
# Functions #####################################
#################################################
def transform_stamped_2_matrix(transform_stamped):
    """Generate 4x4 homogeneous transformation matrix from a transform_stamped msg.

    Parameters
    ----------
    transform_stamped : :py:obj:`!geometry_msgs.msgs.TransformStamped`
        Stamped transform.

    Returns
    -------
    :py:obj:`numpy.array`
        Homogeneous 4x4 transformation matrix.
    """

    # Unpack transform stamped object
    trans_obj = transform_stamped.transform.translation
    rot_obj = transform_stamped.transform.rotation
    trans = tf_conversions.transformations.translation_matrix(
        [trans_obj.x, trans_obj.y, trans_obj.z]
    )
    rot = tf_conversions.transformations.quaternion_matrix(
        [rot_obj.x, rot_obj.y, rot_obj.z, rot_obj.w]
    )

    # Generate homogeneous transformation matrix
    H = np.dot(trans, rot)

    # " Return homogeneous transformation matrix "
    return H


def pose_msg_stamped_2_matrix(pose_msg):
    """Generate 4x4 homogeneous transformation matrix from a pose_stamped msg.

    Parameters
    ----------
    pose_msg : :py:obj:`!geometry_msgs.msgs.PoseStamped`
        Stamped pose message.

    Returns
    -------
    :py:obj:`numpy.array`
        Homogeneous 4x4 transformation matrix.
    """

    # Unpack transform stamped object
    position_obj = pose_msg.pose.position
    rot_obj = pose_msg.pose.orientation
    trans = tf_conversions.transformations.translation_matrix(
        [position_obj.x, position_obj.y, position_obj.z]
    )
    rot = tf_conversions.transformations.quaternion_matrix(
        [rot_obj.x, rot_obj.y, rot_obj.z, rot_obj.w]
    )

    # Generate homogeneous transformation matrix
    H = np.dot(trans, rot)

    # Return homogeneous transformation matrix
    return H
