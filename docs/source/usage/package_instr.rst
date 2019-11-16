Package use instructions
===================================

Configuration instructions
-----------------------------------

All of the `panda_autograsp`_ settings can be found in the
`./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_
file. As described below you can temporarily override some of These
settings by supplying launch arguments.

Add planning constraints
---------------------------------------

To add extra planning constraints to the scene you can add additional
``json`` files to the ``/home/<USER>/.panda_simulation`` folder. This
folder is created the first time the `panda_autograsp`_ solution is launched.
A guide on how this can be done can be found `here <https://erdalpekel.de/?p=123>`_.

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
    - **gazebo**: Specifies wheter the gazebo gui should be loaded.
    - **rviz_bui**: Specifies wheter the rviz gui should be loaded.
    - **calib_type**: The robot hand-eye calibration board type (chessboard vs arucoboard), overwrites the default that is set in the `./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_ file.
    - **debug**: If true the verbosity of the ROS log messages will be increased, and the process name will be displayed with each log messages, by default set to false.
    - **moveit_perception**: This enables the Moveit perception module which integrates sensor data into the path planning, by default set to false.
    - **octomap_type**: The data type used by the Moveit perception module (pointcloud or depthmap), defaults to depthmap.
    - **octomap_resolution**: The resolution of the octomap.
    - **moveit_add_scene_collision_objects**: This specifies if the robot table should be added as a collision object to the moveit planning space, By default set to true.
    - **grasp_solution**: Specify which grasping solution you want to use, this overwrites the solution that is set in the `./cfg/main_config.cfg <https://github.com/rickstaa/panda_autograsp/blob/melodic-devel/panda_autograsp/cfg/main_config.yaml>`_ file.

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

Moveit perception
^^^^^^^^^^^^^^^^^^^^^^^^

Change settings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Appart from the launch file setings additional moveit perception
settings can be found in the
``./panda_moveit_config/config/sensor_kinect_deptmap.yaml``
and ``sensor_kinect_pointcloud.yaml`` files.

Clear the octomap
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The octomap can be reset during execution by calling the ``reset_octomap``
service. This is done using the ``rosservice call /reset_octomap``
command.

Panda simulations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A gazebo simulation of the panda robot can be used by setting the
``real`` and ``gazebo`` launch file arguments.

.. image:: http://my.url.com/my-picture1.png
    :alt: gaebo_simulation

.. warning::

    As the simulated kinect outputs the `openni_launch <https://wiki.ros.org/openni_launch>`_
    to simulate the kinect while I am using the
    `iai_kinect2 package <https://github.com/code-iai/iai_kinect2>`_ the
    simulation is not fully ready. To get it to work one has to or replace
    the iai_kinect2 package in the real setup with the openni_launch or
    write an modified version of the ``iai_kienct2/kinect2_bridge`` that
    works with the simulated camera instead of looking for a real one.
