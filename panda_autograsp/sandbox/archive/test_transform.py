import tf
import numpy as np
import tf_conversions

## Create homogenious matrix ##
R_tmp = tf.transformations.random_rotation_matrix()
R = R_tmp[:3,:3]
T = np.array([ 1. ,  2. ,  0.5])
M = np.empty((4, 4))
M[:3, :3] = R
M[:3, 3] = T
M[3, :] = [0, 0, 0, 1]