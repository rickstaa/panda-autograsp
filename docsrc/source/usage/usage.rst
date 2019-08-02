.. usage:

.. _panda_autograsp: https://github.com/BerkeleyAutomation/gqcnn
.. _Dex-Net 4.0: https://berkeleyautomation.github.io/dex-net/#dexnet_4
.. _FC-GQ-CNN: https://berkeleyautomation.github.io/fcgqcnn

How to use
====================

The panda_autograsp solution can be started both with a real physical system
alternatively, with a simulated system.

Physical robot
--------------------

You can start the `panda_autograsp`_ solution with the real robot using
the following command:

.. code-block:: bash

    roslaunch panda_autograsp autograsp.launch solution:=<GRASPING_SOLUTION>

Simulated robot
-------------------

To start the `panda_autograsp`_ solution with the simulation use this command:

.. code-block:: bash

    roslaunch panda_autograsp autograsp.launch real:=True solution:=<GRASPING_SOLUTIOn>

Available grasping solutions
=====================================

Currently the following grasping solutions are supported:

#. **GQCNN-4.0-PJ**: For `Dex-Net 4.0`_, trained on images of objects in clutter with parameters for a PhotoNeo PhoXi S.
#. **FC-GQCNN-4.0-PJ**: For `FC-GQ-CNN`_, trained on images of objects in clutter with parameters for a PhotoNeo PhoXi S.

Sensor calibration
========================

Calibration instructions
------------------------------------
The Kinect camera is calibrated using the
:py:mod:`tools.chessboard_calibration`.

Checkerboard pattern
------------------------------------

.. figure:: ../_images/chessboard.png
    :scale: 7%
    :alt: Calibration checkerboard

The calibration ``checkerboard_A3.svg`` were downloaded from  created
using the `rgbdemo repository <https://github.com/rgbdemo/rgbdemo>`_
and can be found in the ``./data/calib/checkerboard`` folder.

Calibration configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The calibration configuration settings can be found in the
``./cfg/calib/register_camera.yaml`` file.
