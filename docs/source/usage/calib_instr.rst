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

Generate ar markers
----------------------------------------
To generate the ar markers run the ``rosrun ar_track_alvar createMarker -s 4.5`` command. The 4.5 stands for the size.
Then following use 0,1,2 as the tag ids. 

- This toolbox asks for the [x,y] position of the bottom left corner of the marker.

-Print the pattern
-Measure the marker size and adjust the relative positions in the xml so that the cluster is well defined.

positive-z comes out of the front of the tag toward the viewer, positive-x is to the right, and positive-y is up.