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
therefore performed at the start of the `panda_autograsp` solution.
During this eye-hand calibration, you are asked asked to place a calibration
pattern on the upper left corner of the table. The `panda_autograsp`_ algorithm
supports two types of calibration patterns, an Aruco Board and a
chessboard. Due to its higher calibration accuracy, by default, the algorithm
assumes you are using an Aruco Board. If you want to use the chessboard
instead, you have to change the ``pose_estimation_calib_board`` parameter
in the `./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_ 
file.


Generate chessboard pattern
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Different chessboard patterns can be found in the
`IAI_kinect2/kinect2_calibration <iai_kinect2>`_ repository.
If you specified, you are using a chessboard the algorithm
currently assumed you are using 5x7x0.03 chessboard.
You can change it by changing the ``chessboard_settings``
of the `./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_
file.

Generate Aruco Board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To generate the Aruco Board please run the :py:mod:`generate_arucoboard.py`
python script. This script is found in the ``./scripts`` folder.


.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp
.. _iai_kinect2: https://github.com/code-iai/iai_kinect2
.. _calib_instructions: https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration>`_images
