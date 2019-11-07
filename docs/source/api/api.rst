
.. _api:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Python Code API
======================

All python modules, classes, script and functions of the `panda_autograsp`_
package will be documented here.

Panda autograsp modules
------------------------------------
.. autosummary::
   :toctree: _autosummary

    panda_autograsp.loggers
    panda_autograsp.grasp_planners
    panda_autograsp.functions
    panda_autograsp.moveit_collision_objects
    panda_autograsp.moveit_planner_server_ros
    panda_autograsp.panda_autograsp_server_ros
    panda_autograsp.tf2_broadcaster_ros

Panda autograsp scripts
------------------------------------
.. autosummary::
   :toctree: _autosummary

    aruco_pose_estimation
    chessboard_calibration
    generate_arucoboard
    kinect_processing
    plan_grasp

Panda_autograsp ROS nodes
------------------------------------
.. autosummary::
   :toctree: _autosummary

    grasp_planner_server
    moveit_planner_server
    moveit_random_planner_client
    panda_autograsp_cli
    panda_autograsp_server
    tf2_broadcaster
