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
