Package use instructions
===================================

Configuration instructions
-----------------------------------

All of the `panda_autograsp`_ settings can be found in the
`./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_
file. As described below you can temporarily override some of These
settings by supplying launch arguments.

Run instructions
----------------------------------

To start the `panda_autograsp`_ solution, please open a terminal
and start the `panda_autograsp`_ solution using the following command:

.. code-block:: bash

    $ roslaunch panda_autograsp panda_autograsp.launch

Additionally, this launch file can also be supplied with a number of
launch file arguments. These arguments can be used to temporarily
override the settings you configured in the `./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_
file. The available launch file arguments can be found below. After the
`panda_autograsp`_ solution has launched, you can launch the command-line
interface  (CLI), in a second terminal, using the following command in a
second terminal window:

.. code-block:: bash

    $ rosrun panda_autograsp panda_autograsp_cli.physical

.. note::

 Before executing the commands mentioned above make sure you the `panda_autograsp`_ package is build
 and sourced.

Launch file arguments
---------------------------------------

The panda_autograsp launch file accepts the following launch file arguments:

    - **real**: Specifies whether you want to use the `panda_autograsp`_ solution on the real robot, by default False.
    - **calib_type**: The robot hand-eye calibration board type (chessboard vs arucoboard), overwrites the default that is set in the `./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_ file.
    - **debug**: If true the verbosity of the ROS log messages will be increased, and the process name will be displayed with each log messages, by default set to false.
    - **moveit_perception**: This enables the Moveit perception module which integrates sensor data into the path planning, by default set to false.
    - **moveit_add_scene_collision_objects**: This specifies if the robot table should be added as a collision object to the moveit planning space, By default set to true.
    - **grasping_solution**: Specify which grasping solution you want to use, this overwrites the solution that is set in the `./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_ file.

Grasping solutions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Currently, the following grasping solutions are supported:

    - **GQCNN-2.0**: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics. Trained on `Dex-Net 2.0`_.
    - **GQCNN-2.1**: Extension off GQ-CNN 2.0. The network that was trained on `Dex-Net 2.0`_ is improved using RL in simulations.
    - **GQCNN-4.0-PJ**: Improvement of the GQCNN-2.0 which computes more accurate grasps. This network module does additionally state whether it is better to grasp an object using a suction or a parallel jaw gripper.
    - **FC-GQCNN-4.0-PJ**: Modification of GQCNN-4.0-PJ in which a fully connected grasp quality CNN (`FC-GQ-CNN`_) is used. This model has a faster grasp computation time and a more accurate grasp.

.. _Dex-net 2.0: https://berkeleyautomation.github.io/dex-net/#dexnet_2
.. _Dex-Net 4.0: https://berkeleyautomation.github.io/dex-net/#dexnet_4
.. _FC-GQ-CNN: https://berkeleyautomation.github.io/fcgqcnn
.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp
