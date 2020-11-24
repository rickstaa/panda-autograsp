Sensor calibration instructions
====================================

Camera calibration
----------------------------------------

The Kinect camera can be calibrated using the `kinect2_calibration module of the IAI_kinect2 package <iai_kinect2>`_.
The instructions for performing this calibration can be
`found here <_calib_instructions>`_.

.. warning::

    Be sure to have the camera on for at least 15 min before performing the calibration in order
    for the camera parameters to fully stabilize.

Robot eye-hand calibration
---------------------------------

To be able to control the real robot, we also need to know the
location of the robot relative to the camera. A robot eye-hand calibration is
therefore performed at the start of the `panda-autograsp` solution.
During this eye-hand calibration, you are asked to place a calibration
pattern on the upper left corner of the table. The position of this pattern
and the robot has to be measured and specified in the
`./cfg/calib_frames_poses.yaml <https://github.com/rickstaa/panda-autograsp/blob/melodic-devel/panda-autograsp/cfg/main_config.yaml>`__.
You can also adjust the position and orientation of both the sensor
and calibration frames by using the ``dynamic reconfigure`` window
that is opened when the `panda-autograsp`_ solution is started.
The changes made in the dynamic reconfigure window are saved to the
`./cfg/calib_frames_poses.yaml <https://github.com/rickstaa/panda-autograsp/blob/melodic-devel/panda-autograsp/cfg/calib_frames_poses.yaml>`__.
file when you close the `panda-autograsp`_ solution.

.. note::

    The `panda-autograsp`_ algorithm
    supports two types of calibration patterns, an Aruco Board and a
    chessboard. Due to its higher calibration accuracy, by default, the algorithm
    assumes you are using an Aruco Board. If you want to use the chessboard
    instead, you have to change the ``pose_estimation_calib_board`` parameter
    in the `./cfg/main_config.cfg <https://github.com/rickstaa/panda-autograsp/blob/melodic-devel/panda-autograsp/cfg/main_config.yaml>`_
    file.

Dynamic reconfigure window
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As specified above you can use the dynamic reconfigure gui (shown below)
to position and orientation of both the calibration and sensor frame. This
is done by going to the ``tf_broadcaster`` tab and adjusting the sliders.

.. figure:: https://user-images.githubusercontent.com/17570430/69071747-b6343380-0a2a-11ea-9192-a7ff86501bad.png
    :alt: dynamic reconfigure window
    :target: https://user-images.githubusercontent.com/17570430/69071747-b6343380-0a2a-11ea-9192-a7ff86501bad.png

    Dynamic reconfigure window that can be used to change the sensor and calibration frames.

Generate chessboard pattern
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Different chessboard patterns can be found in the
`IAI_kinect2/kinect2_calibration <iai_kinect2>`_ repository.
If you specified, you are using a chessboard the algorithm
currently assumed you are using 5x7x0.03 chessboard.
You can change it by changing the ``chessboard_settings``
of the `./cfg/main_config.cfg <https://github.com/rickstaa/panda-autograsp/blob/melodic-devel/panda-autograsp/cfg/main_config.yaml>`_
file.

Generate Aruco Board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To generate the Aruco Board please run the :py:mod:`generate_arucoboard.py`
python script. This script is found in the ``./scripts`` folder.

.. _panda-autograsp: https://github.com/rickstaa/panda-autograsp
.. _iai_kinect2: https://github.com/code-iai/iai_kinect2
.. _calib_instructions: https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration>`_images
